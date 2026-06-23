/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "jy61p.h"
#include "motor.h"
#include "oled.h"
#include "pid.h"
#include "menu.h"
#include "line.h"
#include "encoder.h"
#include "bt_remote.h"
#include "line_task.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BASE_DRIVE_PWM 1400
#define MAX_TURN_CORRECTION 600
#define DRIVE_STRAIGHT_RUN 0
#define DRIVE_YAW_HOLD 1
#define SOFTSTART_STEP 40
#define OLED_REFRESH_PERIOD_MS 100U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t g_oledstring[50];
uint8_t g_mode = 0;
extern float g_yaw_jy61;
extern volatile uint8_t g_usart2_receivedata;
extern volatile uint32_t g_jy61p_angle_count;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int limit_turn_pwm(float pwm)
{
  float abs_pwm = (pwm >= 0.0f) ? pwm : -pwm;

  if (pwm > 3200.0f)
  {
    pwm = 3200.0f;
  }
  else if (pwm < -3200.0f)
  {
    pwm = -3200.0f;
  }

  if ((pwm > -180.0f) && (pwm < 180.0f))
  {
    pwm = 0.0f;
  }
  else if (abs_pwm < 1800.0f)
  {
    pwm = (pwm > 0.0f) ? 1800.0f : -1800.0f;
  }

  return (int)pwm;
}

static int clamp_motor_pwm(int pwm)
{
  if (pwm > 2400)
  {
    pwm = 2400;
  }
  else if (pwm < -2400)
  {
    pwm = -2400;
  }

  return pwm;
}

static uint8_t is_remote_control_mode(uint8_t mode)
{
  return ((mode == MENU_MODE_BT_REMOTE) || (mode == MENU_MODE_VOICE_REMOTE)) ? 1U : 0U;
}

static void reset_yaw_target(float current_yaw, float *target_yaw)
{
  *target_yaw = current_yaw;
  set_pid_target(&g_pid_turn_angle, current_yaw);
  g_pid_turn_angle.integral = 0.0f;
  g_pid_turn_angle.err_last = 0.0f;
}

static uint8_t key_pressed(GPIO_TypeDef *port, uint16_t pin, uint8_t *last_state)
{
  uint8_t current_state = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t pressed = 0U;

  if ((*last_state == 1U) && (current_state == 0U))
  {
    HAL_Delay(10);
    current_state = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1U : 0U;
    if (current_state == 0U)
    {
      pressed = 1U;
    }
  }

  *last_state = current_state;
  return pressed;
}

static void buzzer_beep(uint16_t duration_ms)
{
  HAL_GPIO_WritePin(TASK_LED_GPIO_Port, TASK_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
  HAL_Delay(duration_ms);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TASK_LED_GPIO_Port, TASK_LED_Pin, GPIO_PIN_SET);
}

static void task_led_on(void)
{
  HAL_GPIO_WritePin(TASK_LED_GPIO_Port, TASK_LED_Pin, GPIO_PIN_RESET);
}

static void task_led_off(void)
{
  HAL_GPIO_WritePin(TASK_LED_GPIO_Port, TASK_LED_Pin, GPIO_PIN_SET);
}

static void motors_stop_safe(void)
{
  load_motor_pwm(0, 0);
  set_motor1_disable();
  set_motor2_disable();
}

static void task_finish_beep(void)
{
  uint8_t i;

  for (i = 0; i < 3U; i++)
  {
    task_led_on();
    buzzer_beep(80);
    task_led_off();
    HAL_Delay(80);
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  uint8_t yaw_target_ready = 0;
  uint32_t last_angle_count = 0;
  uint8_t yaw_stale_count = 0;
  uint32_t loop_count = 0;
  float target_yaw = 0.0f;
  int turn_pwm = 0;
  int left_pwm = 0;
  int right_pwm = 0;
  uint8_t s1_event = 0;
  uint8_t s2_event = 0;
  uint8_t manual_count_event = 0;
  uint8_t s1_state = 1;
  uint8_t s2_state = 1;
  uint8_t manual_count_state = 1;
  uint8_t drive_enabled = 0;
  uint8_t drive_mode = DRIVE_STRAIGHT_RUN;
  int current_drive_pwm = 0;
  int target_drive_pwm = 0;
  uint32_t last_oled_tick = 0;
  line_task_t line_task;
  bt_remote_state_t bt_remote = {0};
  uint8_t last_mode = 0xFFU;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  pid_param_init();
  set_p_i_d(&g_pid_turn_angle, 22.0f, 0.08f, 6.0f);
  line_task_reset(&line_task);

  bt_remote_start_rx();
  g_jy61p_uart_start_status = HAL_UART_Receive_IT(&huart3, (uint8_t *)&g_usart2_receivedata, 1);

  OLED_Init();
  OLED_ColorTurn(0);
  OLED_DisplayTurn(0);
  g_mode = menu_start_select();
  if (is_remote_control_mode(g_mode) != 0U)
  {
    bt_remote_reset(&bt_remote);
    left_pwm = bt_remote.left_pwm;
    right_pwm = bt_remote.right_pwm;
  }
  last_mode = g_mode;

  while (1)
  {
    loop_count++;
    if ((loop_count % 25U) == 0U)
    {
      HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    }

    if (is_remote_control_mode(g_mode) == 0U)
    {
      if (g_jy61p_angle_count != last_angle_count)
      {
        last_angle_count = g_jy61p_angle_count;
        yaw_stale_count = 0;

        if (yaw_target_ready == 0U)
        {
          reset_yaw_target(g_yaw_jy61, &target_yaw);
          yaw_target_ready = 1U;
        }

        turn_pwm = limit_turn_pwm(turn_angle_pid_realize(&g_pid_turn_angle, g_yaw_jy61));
        if (turn_pwm > MAX_TURN_CORRECTION)
        {
          turn_pwm = MAX_TURN_CORRECTION;
        }
        else if (turn_pwm < -MAX_TURN_CORRECTION)
        {
          turn_pwm = -MAX_TURN_CORRECTION;
        }

        if (drive_mode == DRIVE_STRAIGHT_RUN)
        {
          left_pwm = clamp_motor_pwm(current_drive_pwm - turn_pwm);
          right_pwm = clamp_motor_pwm(current_drive_pwm + turn_pwm);
        }
        else
        {
          left_pwm = clamp_motor_pwm(-turn_pwm);
          right_pwm = clamp_motor_pwm(turn_pwm);
        }

        if (drive_enabled != 0U)
        {
          load_motor_pwm(left_pwm, right_pwm);
        }
        else
        {
          left_pwm = 0;
          right_pwm = 0;
          load_motor_pwm(0, 0);
        }
      }
      else if (yaw_target_ready == 0U)
      {
        left_pwm = 0;
        right_pwm = 0;
        load_motor_pwm(0, 0);
      }
      else if (++yaw_stale_count > 10U)
      {
        yaw_stale_count = 10U;
        turn_pwm = 0;
        left_pwm = 0;
        right_pwm = 0;
        g_pid_turn_angle.integral = 0.0f;
        g_pid_turn_angle.err_last = 0.0f;
        load_motor_pwm(0, 0);
      }
    }

    s1_event = key_pressed(KEY0_GPIO_Port, KEY0_Pin, &s1_state);
    s2_event = key_pressed(KEY1_GPIO_Port, KEY1_Pin, &s2_state);
    manual_count_event = key_pressed(MANUAL_COUNT_KEY_GPIO_Port, MANUAL_COUNT_KEY_Pin, &manual_count_state);

    if (manual_count_event != 0U)
    {
      buzzer_beep(40);
      line_task_increment_target(&line_task, g_mode, drive_enabled);
    }

    if (s1_event != 0U)
    {
      buzzer_beep(40);
      if (is_remote_control_mode(g_mode) != 0U)
      {
        if (drive_enabled == 0U)
        {
          bt_remote_reset(&bt_remote);
          left_pwm = bt_remote.left_pwm;
          right_pwm = bt_remote.right_pwm;
          drive_enabled = 1U;
          set_motor1_enable();
          set_motor2_enable();
          task_led_on();
        }
        else
        {
          drive_enabled = 0U;
          bt_remote_reset(&bt_remote);
          left_pwm = bt_remote.left_pwm;
          right_pwm = bt_remote.right_pwm;
          motors_stop_safe();
          task_led_off();
        }
      }
      else if (drive_enabled == 0U)
      {
        line_task_start(&line_task, g_mode);
        task_led_off();

        drive_enabled = 1U;
        set_motor1_enable();
        set_motor2_enable();
        if (yaw_target_ready != 0U)
        {
          reset_yaw_target(g_yaw_jy61, &target_yaw);
        }
      }
      else
      {
        if (line_task.active != 0U)
        {
          drive_enabled = 0U;
          line_task_stop(&line_task);
          current_drive_pwm = 0;
          motors_stop_safe();
        }
        else
        {
          drive_mode = (drive_mode == DRIVE_STRAIGHT_RUN) ? DRIVE_YAW_HOLD : DRIVE_STRAIGHT_RUN;
          if (yaw_target_ready != 0U)
          {
            reset_yaw_target(g_yaw_jy61, &target_yaw);
          }
        }
      }
    }

    if (s2_event != 0U)
    {
      buzzer_beep(80);
      drive_enabled = 0U;
      drive_mode = DRIVE_STRAIGHT_RUN;
      line_task_reset(&line_task);
      current_drive_pwm = 0;
      target_drive_pwm = 0;
      bt_remote_reset(&bt_remote);
      left_pwm = 0;
      right_pwm = 0;
      turn_pwm = 0;
      motors_stop_safe();
      task_led_off();

      OLED_Clear();
      OLED_ShowString(0, 0, (uint8_t *)"Exit To Menu", 16, 1);
      OLED_ShowString(0, 24, (uint8_t *)"Motors stopped", 8, 1);
      OLED_Refresh();
      HAL_Delay(500);

      g_mode = menu_start_select();
      if (is_remote_control_mode(g_mode) != 0U)
      {
        bt_remote_reset(&bt_remote);
        left_pwm = bt_remote.left_pwm;
        right_pwm = bt_remote.right_pwm;
      }
      last_mode = g_mode;
      last_oled_tick = 0;
      line_task_reset(&line_task);
      bt_remote_reset(&bt_remote);
      if (yaw_target_ready != 0U)
      {
        reset_yaw_target(g_yaw_jy61, &target_yaw);
      }
      continue;
    }

    if (is_remote_control_mode(g_mode) != 0U)
    {
      if (last_mode != g_mode)
      {
        drive_enabled = 0U;
        bt_remote_reset(&bt_remote);
        left_pwm = bt_remote.left_pwm;
        right_pwm = bt_remote.right_pwm;
        motors_stop_safe();
        task_led_off();
        last_mode = g_mode;
      }

      if (drive_enabled != 0U)
      {
        set_motor1_enable();
        set_motor2_enable();
      }
      else
      {
        set_motor1_disable();
        set_motor2_disable();
      }

      bt_remote_update(&bt_remote, drive_enabled, g_yaw_jy61, g_jy61p_angle_count);
      left_pwm = bt_remote.left_pwm;
      right_pwm = bt_remote.right_pwm;

      if ((HAL_GetTick() - last_oled_tick) >= OLED_REFRESH_PERIOD_MS)
      {
        last_oled_tick = HAL_GetTick();
        OLED_Clear();
        bt_remote_show_status(g_mode, drive_enabled, &bt_remote);
        OLED_Refresh();
      }

      HAL_Delay(20);
      continue;
    }

    if ((drive_enabled != 0U) && (line_task.active != 0U))
    {
      if (line_task_update(&line_task, g_mode, &left_pwm, &right_pwm) != 0U)
      {
        drive_enabled = 0U;
        current_drive_pwm = 0;
        motors_stop_safe();
        task_finish_beep();
      }
    }


    target_drive_pwm = ((drive_enabled != 0U) && (line_task.active == 0U) && (drive_mode == DRIVE_STRAIGHT_RUN)) ? BASE_DRIVE_PWM : 0;
    if (current_drive_pwm < target_drive_pwm)
    {
      current_drive_pwm += SOFTSTART_STEP;
      if (current_drive_pwm > target_drive_pwm)
      {
        current_drive_pwm = target_drive_pwm;
      }
    }
    else if (current_drive_pwm > target_drive_pwm)
    {
      current_drive_pwm -= SOFTSTART_STEP;
      if (current_drive_pwm < target_drive_pwm)
      {
        current_drive_pwm = target_drive_pwm;
      }
    }

    if (drive_enabled == 0U)
    {
      set_motor1_disable();
      set_motor2_disable();
    }

    if ((HAL_GetTick() - last_oled_tick) >= OLED_REFRESH_PERIOD_MS)
    {
      last_oled_tick = HAL_GetTick();
      line_task.line_offset = line_err();
      line_task.line_state = g_line_sensor_state;

      OLED_Clear();
      menu_show_status(g_mode, drive_enabled, g_yaw_jy61, target_yaw,
                       line_task.line_offset, line_task.line_state,
                       line_task.corner_count, line_task.turn_active,
                       line_task.manual_target_count, left_pwm, right_pwm);
      OLED_Refresh();
    }

    HAL_Delay(20);
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    for (volatile uint32_t i = 0; i < 500000; i++)
    {
    }
  }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

