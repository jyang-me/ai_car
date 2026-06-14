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
#define DRIVE_YAW_HOLD     1
#define SOFTSTART_STEP    40
#define OLED_REFRESH_PERIOD_MS 100U
#define BASIC_LINE_BASE_PWM 1200
#define BASIC_LINE_KP       22
#define BASIC_LINE_MAX_CORR 700
#define BASIC_TURN_SEARCH_PWM 1050
#define BASIC_TURN_FORWARD_PWM 450
#define BASIC_CIRCLE_TIMEOUT_MS 18000U
#define BASIC_CIRCLE_CORNER_DONE_COUNT 4U
#define ADV_DOUBLE_CORNER_DONE_COUNT 9U
#define ADV_DOUBLE_TIMEOUT_MS 36000U
#define ADV_DIAGONAL_BASE_PWM 900
#define ADV_DIAGONAL_TARGET_YAW -48.0f
#define ADV_DIAGONAL_YAW_MAX_CORR 180
#define ADV_DIAGONAL_TURN_SEARCH_PWM 1300
#define ADV_DIAGONAL_TURN_FORWARD_PWM 0
#define ADV_DIAGONAL_C_TURN_PWM 850
#define ADV_DIAGONAL_CORNER_FORWARD_MS DOUBLE_CORNER_FORWARD_MS
#define ADV_DIAGONAL_CORNER_STOP_MS DOUBLE_CORNER_STOP_MS
#define ADV_DIAGONAL_C_TURN_DEG 125.0f
#define ADV_DIAGONAL_C_TURN_DONE_ERR 10.0f
#define ADV_DIAGONAL_C_TURN_STOP_MS 200U
#define ADV_DIAGONAL_ENCODER_KP 1
#define ADV_DIAGONAL_ENCODER_MAX_CORR 120
#define ADV_DIAGONAL_ALIGN_DONE_ERR 3.0f
#define ADV_DIAGONAL_ALIGN_STABLE_COUNT 8U
#define ADV_DIAGONAL_ALIGN_TIMEOUT_MS 1500U
#define ADV_DIAGONAL_START_IGNORE_MS 800U
#define ADV_DIAGONAL_CORNER_SENSOR_MIN 2U
#define ADV_DIAGONAL_CORNER_CONFIRM_COUNT 3U
#define ADV_DIAGONAL_DONE_COUNT 3U
#define ADV_DIAGONAL_COUNT_IGNORE_MS 1500U
#define ADV_DIAGONAL_TIMEOUT_MS 30000U
#define ADV_MANUAL_CW_DONE_COUNT 4U
#define BASIC_LINE_LOST_STOP_COUNT 3U
#define DOUBLE_CCW_START_COUNT_ONCE_MS 1200U
#define DOUBLE_CORNER_COUNT_IGNORE_MS 1500U
#define DOUBLE_CORNER_FORWARD_MS 200U
#define DOUBLE_CORNER_STOP_MS 150U
#define DOUBLE_TURN_BACK_DEG 150.0f
#define DOUBLE_TURN_BACK_DONE_ERR 12.0f
#define DOUBLE_PHASE_CW 0U
#define DOUBLE_PHASE_TURN_BACK 1U
#define DOUBLE_PHASE_CCW 2U
#define DOUBLE_CORNER_WAIT_NONE 0U
#define DOUBLE_CORNER_WAIT_FORWARD 1U
#define DOUBLE_CORNER_WAIT_STOP 2U
#define DOUBLE_CORNER_NEXT_TURN 0U
#define DOUBLE_CORNER_NEXT_TURN_BACK 1U
#define DOUBLE_CORNER_NEXT_FINISH 2U
#define BASIC_CORNER_FORWARD_MS DOUBLE_CORNER_FORWARD_MS
#define BASIC_CORNER_STOP_MS DOUBLE_CORNER_STOP_MS
#define BASIC_CORNER_WAIT_NONE 0U
#define BASIC_CORNER_WAIT_FORWARD 1U
#define BASIC_CORNER_WAIT_STOP 2U
#define BASIC_SINGLE_FORWARD_MS DOUBLE_CORNER_FORWARD_MS
#define BASIC_SINGLE_STOP_MS DOUBLE_CORNER_STOP_MS
#define BASIC_SINGLE_WAIT_NONE 0U
#define BASIC_SINGLE_WAIT_FORWARD 1U
#define BASIC_SINGLE_WAIT_STOP 2U
#define DIAGONAL_PHASE_ALIGN 0U
#define DIAGONAL_PHASE_GYRO_STRAIGHT 1U
#define DIAGONAL_PHASE_LINE_TRACK 2U
#define DIAGONAL_PHASE_CORNER_FORWARD 3U
#define DIAGONAL_PHASE_CORNER_STOP 4U
#define DIAGONAL_PHASE_C_TURN_ANGLE 5U
#define DIAGONAL_PHASE_C_TURN_STOP 6U
#define DIAGONAL_PHASE_FINISH_FORWARD 7U
#define DIAGONAL_PHASE_FINISH_STOP 8U
#define DIAGONAL_PHASE_TURN_CCW 9U
#define BT_REMOTE_DEFAULT_PWM 1200
#define BT_REMOTE_TURN_PWM    1000
#define BT_REMOTE_TIMEOUT_MS  1500U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t g_oledstring[50];
uint8_t g_mode = 0;
extern uint8_t g_usart2_receivedata;
extern uint8_t g_bt_receivedata;
extern volatile uint8_t g_bt_cmd;
extern volatile uint32_t g_bt_rx_count;
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

static uint8_t is_basic_task(uint8_t mode)
{
  return ((mode <= MENU_MODE_BASIC_CCW_CIRCLE) ||
          (mode == MENU_MODE_ADV_DOUBLE_LOOP) ||
          (mode == MENU_MODE_ADV_DIAGONAL_AC) ||
          (mode == MENU_MODE_ADV_RANDOM_POINTS)) ? 1U : 0U;
}

static uint8_t is_circle_task(uint8_t mode)
{
  return ((mode == MENU_MODE_BASIC_CW_CIRCLE) || (mode == MENU_MODE_BASIC_CCW_CIRCLE)) ? 1U : 0U;
}

static uint8_t is_lost_line_turn_task(uint8_t mode)
{
  return ((is_circle_task(mode) != 0U) || (mode == MENU_MODE_ADV_DOUBLE_LOOP)) ? 1U : 0U;
}

static uint8_t is_manual_count_task(uint8_t mode)
{
  return (mode == MENU_MODE_ADV_RANDOM_POINTS) ? 1U : 0U;
}

static int8_t circle_turn_dir(uint8_t mode)
{
  return (mode == MENU_MODE_BASIC_CW_CIRCLE) ? -1 : 1;
}

static int8_t double_phase_turn_dir(uint8_t phase)
{
  if (phase == DOUBLE_PHASE_CW)
  {
    return circle_turn_dir(MENU_MODE_BASIC_CW_CIRCLE);
  }

  return circle_turn_dir(MENU_MODE_BASIC_CCW_CIRCLE);
}

static float yaw_add_deg(float yaw, float add_deg)
{
  yaw += add_deg;

  while (yaw > 180.0f)
  {
    yaw -= 360.0f;
  }
  while (yaw < -180.0f)
  {
    yaw += 360.0f;
  }

  return yaw;
}

static float yaw_abs_error(float target_yaw, float current_yaw)
{
  float err = target_yaw - current_yaw;

  while (err > 180.0f)
  {
    err -= 360.0f;
  }
  while (err < -180.0f)
  {
    err += 360.0f;
  }

  return (err >= 0.0f) ? err : -err;
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

static uint8_t bt_remote_normalize_cmd(uint8_t cmd)
{
  if ((cmd >= 'a') && (cmd <= 'z'))
  {
    cmd = (uint8_t)(cmd - ('a' - 'A'));
  }

  switch (cmd)
  {
    case 'F':
    case 'B':
    case 'L':
    case 'R':
    case 'S':
      return cmd;

    default:
      return 0U;
  }
}

static void bt_remote_apply(uint8_t motion, int speed, int *left_pwm, int *right_pwm)
{
  int turn_pwm = BT_REMOTE_TURN_PWM;

  if (turn_pwm > speed)
  {
    turn_pwm = speed;
  }

  switch (motion)
  {
    case 'F':
      *left_pwm = clamp_motor_pwm(speed);
      *right_pwm = clamp_motor_pwm(speed);
      break;

    case 'B':
      *left_pwm = clamp_motor_pwm(-speed);
      *right_pwm = clamp_motor_pwm(-speed);
      break;

    case 'L':
      *left_pwm = clamp_motor_pwm(-turn_pwm);
      *right_pwm = clamp_motor_pwm(turn_pwm);
      break;

    case 'R':
      *left_pwm = clamp_motor_pwm(turn_pwm);
      *right_pwm = clamp_motor_pwm(-turn_pwm);
      break;

    default:
      *left_pwm = 0;
      *right_pwm = 0;
      break;
  }

  load_motor_pwm(*left_pwm, *right_pwm);
}

static void bt_remote_show_status(uint8_t drive_enabled,
                                  uint8_t motion,
                                  int speed,
                                  int left_pwm,
                                  int right_pwm)
{
  OLED_ShowString(0, 0, (uint8_t *)menu_get_mode_name(MENU_MODE_BT_REMOTE), 8, 1);

  sprintf((char *)g_oledstring, "Run:%s RX:%lu",
          (drive_enabled == 0U) ? "SAFE" : "ON  ",
          (unsigned long)g_bt_rx_count);
  OLED_ShowString(0, 10, g_oledstring, 8, 1);

  sprintf((char *)g_oledstring, "Cmd:%c Spd:%4d",
          (motion == 0U) ? 'S' : motion,
          speed);
  OLED_ShowString(0, 22, g_oledstring, 8, 1);

  sprintf((char *)g_oledstring, "L:%4d R:%4d", left_pwm, right_pwm);
  OLED_ShowString(0, 34, g_oledstring, 8, 1);

  OLED_ShowString(0, 46, (uint8_t *)"F B L R S 0-9", 8, 1);
  OLED_ShowString(0, 56, (uint8_t *)"K0 RUN K1 MENU", 8, 1);
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

static void task_count_beep(void)
{
  buzzer_beep(50);
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
  int32_t line_offset = 0;
  uint8_t line_state = 0;
  uint32_t last_oled_tick = 0;
  uint8_t task_active = 0;
  uint8_t task_finished = 0;
  uint32_t task_start_tick = 0;
  uint32_t double_turn_back_start_tick = 0;
  uint32_t double_ccw_start_count_once_until = 0;
  uint32_t double_corner_ignore_until = 0;
  uint32_t double_corner_wait_start_tick = 0;
  uint8_t circle_corner_count = 0;
  uint8_t circle_turn_active = 0;
  uint8_t line_lost_count = 0;
  uint8_t double_turn_phase = DOUBLE_PHASE_CW;
  uint8_t double_turn_back_angle_done = 0;
  uint8_t double_ccw_start_counted = 0;
  uint8_t double_corner_wait_phase = DOUBLE_CORNER_WAIT_NONE;
  uint8_t double_corner_next_action = DOUBLE_CORNER_NEXT_TURN;
  uint8_t basic_corner_wait_phase = BASIC_CORNER_WAIT_NONE;
  uint32_t basic_corner_wait_start_tick = 0;
  uint8_t basic_single_wait_phase = BASIC_SINGLE_WAIT_NONE;
  uint32_t basic_single_wait_start_tick = 0;
    uint8_t diagonal_phase = DIAGONAL_PHASE_GYRO_STRAIGHT;
    uint8_t diagonal_align_stable_count = 0;
    uint32_t diagonal_align_start_tick = 0;
    uint32_t diagonal_straight_start_tick = 0;
    uint32_t diagonal_corner_wait_start_tick = 0;
  uint32_t diagonal_ignore_until = 0;
  int32_t diagonal_encoder_corr = 0;
  int diagonal_yaw_corr = 0;
  uint8_t manual_target_count = ADV_MANUAL_CW_DONE_COUNT;
  int8_t circle_dir = 0;
  int line_corr = 0;
  int turn_pwm_search = 0;
  uint8_t bt_motion = 'S';
  int bt_speed = BT_REMOTE_DEFAULT_PWM;
  uint32_t bt_last_rx_tick = 0;

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

  HAL_UART_Receive_IT(&huart1, &g_bt_receivedata, 1);
  g_jy61p_uart_start_status = HAL_UART_Receive_IT(&huart3, &g_usart2_receivedata, 1);

  OLED_Init();
  OLED_ColorTurn(0);
  OLED_DisplayTurn(0);
  g_mode = menu_start_select();

  while (1)
  {
    loop_count++;
    if ((loop_count % 25U) == 0U)
    {
      HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    }

    if (g_mode != MENU_MODE_BT_REMOTE)
    {
      if (g_jy61p_angle_count != last_angle_count)
      {
        last_angle_count = g_jy61p_angle_count;
        yaw_stale_count = 0;

        if (yaw_target_ready == 0U)
        {
                reset_yaw_target(g_yaw_jy61, &target_yaw);
                if (g_mode == MENU_MODE_ADV_DIAGONAL_AC)
                {
                    target_yaw = ADV_DIAGONAL_TARGET_YAW;
                    set_pid_target(&g_pid_turn_angle, target_yaw);
                    g_pid_turn_angle.integral = 0.0f;
                    g_pid_turn_angle.err_last = 0.0f;
                }
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
      if ((g_mode == MENU_MODE_ADV_RANDOM_POINTS) && (drive_enabled == 0U))
      {
        manual_target_count++;
      }
    }

    if (s1_event != 0U)
    {
      buzzer_beep(40);
      if (g_mode == MENU_MODE_BT_REMOTE)
      {
        if (drive_enabled == 0U)
        {
          drive_enabled = 1U;
          set_motor1_enable();
          set_motor2_enable();
          task_led_on();
        }
        else
        {
          drive_enabled = 0U;
          bt_motion = 'S';
          left_pwm = 0;
          right_pwm = 0;
          motors_stop_safe();
          task_led_off();
        }
      }
      else if (drive_enabled == 0U)
      {
        task_active = is_basic_task(g_mode);
        task_finished = 0U;
        circle_corner_count = 0U;
        circle_turn_active = 0U;
        line_lost_count = 0U;
        double_turn_phase = DOUBLE_PHASE_CW;
        double_turn_back_angle_done = 0U;
        double_ccw_start_counted = 0U;
        double_corner_ignore_until = 0U;
        double_corner_wait_start_tick = 0U;
        double_corner_wait_phase = DOUBLE_CORNER_WAIT_NONE;
        double_corner_next_action = DOUBLE_CORNER_NEXT_TURN;
        basic_corner_wait_phase = BASIC_CORNER_WAIT_NONE;
        basic_corner_wait_start_tick = 0U;
        basic_single_wait_phase = BASIC_SINGLE_WAIT_NONE;
        basic_single_wait_start_tick = 0U;
                if (g_mode == MENU_MODE_ADV_DIAGONAL_AC)
                {
                    diagonal_phase = DIAGONAL_PHASE_ALIGN;
                    diagonal_align_start_tick = HAL_GetTick();
                    diagonal_straight_start_tick = diagonal_align_start_tick;
                    diagonal_align_stable_count = 0;
                }
                else
                {
                    diagonal_phase = DIAGONAL_PHASE_GYRO_STRAIGHT;
                }
        diagonal_ignore_until = 0U;
        diagonal_corner_wait_start_tick = 0U;
        diagonal_encoder_corr = 0;
        circle_dir = ((g_mode == MENU_MODE_ADV_DOUBLE_LOOP) || (g_mode == MENU_MODE_ADV_RANDOM_POINTS)) ?
                     circle_turn_dir(MENU_MODE_BASIC_CW_CIRCLE) : circle_turn_dir(g_mode);
        task_start_tick = HAL_GetTick();
        double_turn_back_start_tick = 0U;
        double_ccw_start_count_once_until = 0U;
        task_led_off();

        drive_enabled = 1U;
        set_motor1_enable();
        set_motor2_enable();
        if (yaw_target_ready != 0U)
        {
          reset_yaw_target(g_yaw_jy61, &target_yaw);
          if (g_mode == MENU_MODE_ADV_DIAGONAL_AC)
          {
            target_yaw = ADV_DIAGONAL_TARGET_YAW;
            set_pid_target(&g_pid_turn_angle, target_yaw);
            g_pid_turn_angle.integral = 0.0f;
            g_pid_turn_angle.err_last = 0.0f;
          }
        }
      }
      else
      {
        if (task_active != 0U)
        {
          drive_enabled = 0U;
          task_active = 0U;
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
      task_active = 0U;
      task_finished = 0U;
      circle_corner_count = 0U;
      circle_turn_active = 0U;
      line_lost_count = 0U;
      double_turn_phase = DOUBLE_PHASE_CW;
      double_turn_back_angle_done = 0U;
      double_turn_back_start_tick = 0U;
      double_ccw_start_count_once_until = 0U;
      double_ccw_start_counted = 0U;
      double_corner_ignore_until = 0U;
      double_corner_wait_start_tick = 0U;
      double_corner_wait_phase = DOUBLE_CORNER_WAIT_NONE;
      double_corner_next_action = DOUBLE_CORNER_NEXT_TURN;
      basic_corner_wait_phase = BASIC_CORNER_WAIT_NONE;
      basic_corner_wait_start_tick = 0U;
      basic_single_wait_phase = BASIC_SINGLE_WAIT_NONE;
      basic_single_wait_start_tick = 0U;
      diagonal_phase = DIAGONAL_PHASE_GYRO_STRAIGHT;
      diagonal_ignore_until = 0U;
      diagonal_corner_wait_start_tick = 0U;
      diagonal_encoder_corr = 0;
      manual_target_count = ADV_MANUAL_CW_DONE_COUNT;
      current_drive_pwm = 0;
      target_drive_pwm = 0;
      bt_motion = 'S';
      bt_speed = BT_REMOTE_DEFAULT_PWM;
      bt_last_rx_tick = 0U;
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
      last_oled_tick = 0;
      task_active = 0U;
      task_finished = 0U;
      circle_corner_count = 0U;
      circle_turn_active = 0U;
      line_lost_count = 0U;
      double_turn_phase = DOUBLE_PHASE_CW;
      double_turn_back_angle_done = 0U;
      double_turn_back_start_tick = 0U;
      double_ccw_start_count_once_until = 0U;
      double_ccw_start_counted = 0U;
      double_corner_ignore_until = 0U;
      double_corner_wait_start_tick = 0U;
      double_corner_wait_phase = DOUBLE_CORNER_WAIT_NONE;
      double_corner_next_action = DOUBLE_CORNER_NEXT_TURN;
      basic_corner_wait_phase = BASIC_CORNER_WAIT_NONE;
      basic_corner_wait_start_tick = 0U;
      basic_single_wait_phase = BASIC_SINGLE_WAIT_NONE;
      basic_single_wait_start_tick = 0U;
      diagonal_phase = DIAGONAL_PHASE_GYRO_STRAIGHT;
      diagonal_ignore_until = 0U;
      diagonal_corner_wait_start_tick = 0U;
      diagonal_encoder_corr = 0;
      manual_target_count = ADV_MANUAL_CW_DONE_COUNT;
      bt_motion = 'S';
      bt_speed = BT_REMOTE_DEFAULT_PWM;
      bt_last_rx_tick = 0U;
      if (yaw_target_ready != 0U)
      {
        reset_yaw_target(g_yaw_jy61, &target_yaw);
      }
      continue;
    }

    if (g_mode == MENU_MODE_BT_REMOTE)
    {
      uint8_t bt_cmd = g_bt_cmd;

      if (bt_cmd != 0U)
      {
        g_bt_cmd = 0U;
        bt_last_rx_tick = HAL_GetTick();

        if ((bt_cmd >= '0') && (bt_cmd <= '9'))
        {
          bt_speed = (bt_cmd == '0') ? 0 : (400 + ((int)(bt_cmd - '0') * 200));
          if (bt_motion != 'S')
          {
            bt_remote_apply(bt_motion, bt_speed, &left_pwm, &right_pwm);
          }
        }
        else
        {
          bt_cmd = bt_remote_normalize_cmd(bt_cmd);
          if (bt_cmd != 0U)
          {
            bt_motion = bt_cmd;
            if ((drive_enabled == 0U) && (bt_motion != 'S'))
            {
              drive_enabled = 1U;
              set_motor1_enable();
              set_motor2_enable();
              task_led_on();
            }

            if (bt_motion == 'S')
            {
              left_pwm = 0;
              right_pwm = 0;
              load_motor_pwm(0, 0);
            }
            else if (drive_enabled != 0U)
            {
              bt_remote_apply(bt_motion, bt_speed, &left_pwm, &right_pwm);
            }
          }
        }
      }

      if ((drive_enabled != 0U) &&
          (bt_motion != 'S') &&
          ((HAL_GetTick() - bt_last_rx_tick) >= BT_REMOTE_TIMEOUT_MS))
      {
        bt_motion = 'S';
        left_pwm = 0;
        right_pwm = 0;
        load_motor_pwm(0, 0);
      }

      if (drive_enabled == 0U)
      {
        left_pwm = 0;
        right_pwm = 0;
        load_motor_pwm(0, 0);
        set_motor1_disable();
        set_motor2_disable();
      }

      if ((HAL_GetTick() - last_oled_tick) >= OLED_REFRESH_PERIOD_MS)
      {
        last_oled_tick = HAL_GetTick();
        OLED_Clear();
        bt_remote_show_status(drive_enabled, bt_motion, bt_speed, left_pwm, right_pwm);
        OLED_Refresh();
      }

      HAL_Delay(20);
      continue;
    }

    if ((drive_enabled != 0U) && (task_active != 0U))
    {
      line_offset = line_err();
      line_state = g_line_sensor_state;

      if (g_mode == MENU_MODE_BASIC_SINGLE_AB)
      {
        if (basic_single_wait_phase != BASIC_SINGLE_WAIT_NONE)
        {
          if (basic_single_wait_phase == BASIC_SINGLE_WAIT_FORWARD)
          {
            left_pwm = BASIC_LINE_BASE_PWM;
            right_pwm = BASIC_LINE_BASE_PWM;
            load_motor_pwm(left_pwm, right_pwm);
            if ((HAL_GetTick() - basic_single_wait_start_tick) >= BASIC_SINGLE_FORWARD_MS)
            {
              basic_single_wait_phase = BASIC_SINGLE_WAIT_STOP;
              basic_single_wait_start_tick = HAL_GetTick();
              load_motor_pwm(0, 0);
              left_pwm = 0;
              right_pwm = 0;
            }
          }
          else
          {
            load_motor_pwm(0, 0);
            left_pwm = 0;
            right_pwm = 0;
            if ((HAL_GetTick() - basic_single_wait_start_tick) >= BASIC_SINGLE_STOP_MS)
            {
              basic_single_wait_phase = BASIC_SINGLE_WAIT_NONE;
              task_finished = 1U;
            }
          }
        }
        else if (g_line_sensor_count == 0U)
        {
          if (++line_lost_count >= BASIC_LINE_LOST_STOP_COUNT)
          {
            line_lost_count = 0U;
            basic_single_wait_phase = BASIC_SINGLE_WAIT_FORWARD;
            basic_single_wait_start_tick = HAL_GetTick();
            left_pwm = BASIC_LINE_BASE_PWM;
            right_pwm = BASIC_LINE_BASE_PWM;
            load_motor_pwm(left_pwm, right_pwm);
          }
        }
        else
        {
          line_lost_count = 0U;
        }
      }
      else if (g_mode == MENU_MODE_ADV_DIAGONAL_AC)
      {
                if (diagonal_phase == DIAGONAL_PHASE_ALIGN)
                {
                    left_pwm = clamp_motor_pwm(-turn_pwm);
                    right_pwm = clamp_motor_pwm(turn_pwm);
                    load_motor_pwm(left_pwm, right_pwm);

                    if (yaw_abs_error(target_yaw, g_yaw_jy61) <= ADV_DIAGONAL_ALIGN_DONE_ERR)
                    {
                        if (diagonal_align_stable_count < ADV_DIAGONAL_ALIGN_STABLE_COUNT)
                        {
                            diagonal_align_stable_count++;
                        }
                    }
                    else
                    {
                        diagonal_align_stable_count = 0;
                    }

                    if ((diagonal_align_stable_count >= ADV_DIAGONAL_ALIGN_STABLE_COUNT) ||
                        ((HAL_GetTick() - diagonal_align_start_tick) >= ADV_DIAGONAL_ALIGN_TIMEOUT_MS))
                    {
                        diagonal_phase = DIAGONAL_PHASE_GYRO_STRAIGHT;
                        diagonal_straight_start_tick = HAL_GetTick();
                        task_start_tick = diagonal_straight_start_tick;
      diagonal_encoder_corr = 0;
                    }
                }
                else if (diagonal_phase == DIAGONAL_PHASE_GYRO_STRAIGHT)
        {
          diagonal_encoder_corr = 0;

          diagonal_yaw_corr = turn_pwm;
          if (diagonal_yaw_corr > ADV_DIAGONAL_YAW_MAX_CORR)
          {
            diagonal_yaw_corr = ADV_DIAGONAL_YAW_MAX_CORR;
          }
          else if (diagonal_yaw_corr < -ADV_DIAGONAL_YAW_MAX_CORR)
          {
            diagonal_yaw_corr = -ADV_DIAGONAL_YAW_MAX_CORR;
          }

          left_pwm = clamp_motor_pwm(ADV_DIAGONAL_BASE_PWM - diagonal_yaw_corr - diagonal_encoder_corr);
          right_pwm = clamp_motor_pwm(ADV_DIAGONAL_BASE_PWM + diagonal_yaw_corr + diagonal_encoder_corr);
          load_motor_pwm(left_pwm, right_pwm);

          if ((HAL_GetTick() - task_start_tick) >= ADV_DIAGONAL_START_IGNORE_MS)
          {
            if (g_line_sensor_count >= ADV_DIAGONAL_CORNER_SENSOR_MIN)
            {
              diagonal_phase = DIAGONAL_PHASE_LINE_TRACK;
              line_lost_count = 0U;
                        diagonal_encoder_corr = 0;
            }
          }
        }
        else if (diagonal_phase == DIAGONAL_PHASE_TURN_CCW)
        {
          if (g_line_sensor_count != 0U)
          {
            diagonal_phase = DIAGONAL_PHASE_LINE_TRACK;
            circle_turn_active = 0U;
            line_lost_count = 0U;
            reset_yaw_target(g_yaw_jy61, &target_yaw);
          }
        }
        else if (diagonal_phase == DIAGONAL_PHASE_C_TURN_ANGLE)
        {
          left_pwm = clamp_motor_pwm(-circle_dir * ADV_DIAGONAL_C_TURN_PWM);
          right_pwm = clamp_motor_pwm(circle_dir * ADV_DIAGONAL_C_TURN_PWM);
          load_motor_pwm(left_pwm, right_pwm);

          if (yaw_abs_error(target_yaw, g_yaw_jy61) <= ADV_DIAGONAL_C_TURN_DONE_ERR)
          {
            diagonal_phase = DIAGONAL_PHASE_C_TURN_STOP;
            diagonal_corner_wait_start_tick = HAL_GetTick();
            circle_turn_active = 0U;
            load_motor_pwm(0, 0);
            left_pwm = 0;
            right_pwm = 0;
          }
        }
        else if (diagonal_phase == DIAGONAL_PHASE_C_TURN_STOP)
        {
          load_motor_pwm(0, 0);
          left_pwm = 0;
          right_pwm = 0;
          if ((HAL_GetTick() - diagonal_corner_wait_start_tick) >= ADV_DIAGONAL_C_TURN_STOP_MS)
          {
            diagonal_phase = DIAGONAL_PHASE_TURN_CCW;
            circle_turn_active = 1U;
            circle_dir = circle_turn_dir(MENU_MODE_BASIC_CCW_CIRCLE);
            diagonal_ignore_until = HAL_GetTick() + ADV_DIAGONAL_COUNT_IGNORE_MS;
          }
        }
        else if (diagonal_phase == DIAGONAL_PHASE_FINISH_FORWARD)
        {
          left_pwm = ADV_DIAGONAL_BASE_PWM;
          right_pwm = ADV_DIAGONAL_BASE_PWM;
          load_motor_pwm(left_pwm, right_pwm);
          if ((HAL_GetTick() - diagonal_corner_wait_start_tick) >= ADV_DIAGONAL_CORNER_FORWARD_MS)
          {
            diagonal_phase = DIAGONAL_PHASE_FINISH_STOP;
            diagonal_corner_wait_start_tick = HAL_GetTick();
            load_motor_pwm(0, 0);
            left_pwm = 0;
            right_pwm = 0;
          }
        }
        else if (diagonal_phase == DIAGONAL_PHASE_FINISH_STOP)
        {
          load_motor_pwm(0, 0);
          left_pwm = 0;
          right_pwm = 0;
          if ((HAL_GetTick() - diagonal_corner_wait_start_tick) >= ADV_DIAGONAL_CORNER_STOP_MS)
          {
            task_finished = 1U;
          }
        }
        else if (diagonal_phase == DIAGONAL_PHASE_CORNER_FORWARD)
        {
          left_pwm = ADV_DIAGONAL_BASE_PWM;
          right_pwm = ADV_DIAGONAL_BASE_PWM;
          load_motor_pwm(left_pwm, right_pwm);
          if ((HAL_GetTick() - diagonal_corner_wait_start_tick) >= ADV_DIAGONAL_CORNER_FORWARD_MS)
          {
            diagonal_phase = DIAGONAL_PHASE_CORNER_STOP;
            diagonal_corner_wait_start_tick = HAL_GetTick();
            load_motor_pwm(0, 0);
            left_pwm = 0;
            right_pwm = 0;
          }
        }
        else if (diagonal_phase == DIAGONAL_PHASE_CORNER_STOP)
        {
          load_motor_pwm(0, 0);
          left_pwm = 0;
          right_pwm = 0;
          if ((HAL_GetTick() - diagonal_corner_wait_start_tick) >= ADV_DIAGONAL_CORNER_STOP_MS)
          {
            diagonal_phase = DIAGONAL_PHASE_C_TURN_ANGLE;
            circle_dir = circle_turn_dir(MENU_MODE_BASIC_CCW_CIRCLE);
            circle_turn_active = 1U;
            target_yaw = yaw_add_deg(g_yaw_jy61, ADV_DIAGONAL_C_TURN_DEG);
            set_pid_target(&g_pid_turn_angle, target_yaw);
            g_pid_turn_angle.integral = 0.0f;
            g_pid_turn_angle.err_last = 0.0f;
          }
        }
        else
        {
          if ((diagonal_ignore_until != 0U) &&
              ((int32_t)(HAL_GetTick() - diagonal_ignore_until) < 0))
          {
            line_lost_count = 0U;
          }
          else if (g_line_sensor_count == 0U)
          {
            if (++line_lost_count >= BASIC_LINE_LOST_STOP_COUNT)
            {
              line_lost_count = 0U;
              circle_corner_count++;
              task_count_beep();

              if (circle_corner_count >= ADV_DIAGONAL_DONE_COUNT)
              {
                diagonal_phase = DIAGONAL_PHASE_FINISH_FORWARD;
                diagonal_corner_wait_start_tick = HAL_GetTick();
                left_pwm = ADV_DIAGONAL_BASE_PWM;
                right_pwm = ADV_DIAGONAL_BASE_PWM;
                load_motor_pwm(left_pwm, right_pwm);
              }
              else if (circle_corner_count == 1U)
              {
                diagonal_phase = DIAGONAL_PHASE_CORNER_FORWARD;
                diagonal_corner_wait_start_tick = HAL_GetTick();
                left_pwm = ADV_DIAGONAL_BASE_PWM;
                right_pwm = ADV_DIAGONAL_BASE_PWM;
                load_motor_pwm(left_pwm, right_pwm);
              }
              else
              {
                diagonal_phase = DIAGONAL_PHASE_TURN_CCW;
                circle_dir = circle_turn_dir(MENU_MODE_BASIC_CCW_CIRCLE);
                circle_turn_active = 1U;
                diagonal_ignore_until = HAL_GetTick() + ADV_DIAGONAL_COUNT_IGNORE_MS;
              }
            }
          }
          else
          {
            line_lost_count = 0U;
          }
        }

        if (((HAL_GetTick() - task_start_tick) >= ADV_DIAGONAL_TIMEOUT_MS) && (task_finished == 0U))
        {
          task_finished = 1U;
        }
      }
      else if (is_manual_count_task(g_mode) != 0U)
      {
        if (circle_turn_active != 0U)
        {
          if (g_line_sensor_count != 0U)
          {
            circle_turn_active = 0U;
            line_lost_count = 0U;
          }
        }
        else if ((task_finished == 0U) && (g_line_sensor_count == 0U))
        {
          if (++line_lost_count >= BASIC_LINE_LOST_STOP_COUNT)
          {
            line_lost_count = 0U;
            circle_corner_count++;
            task_count_beep();

            if (circle_corner_count >= manual_target_count)
            {
              task_finished = 1U;
            }
            else
            {
              circle_dir = circle_turn_dir(MENU_MODE_BASIC_CW_CIRCLE);
              circle_turn_active = 1U;
            }
          }
        }
        else
        {
          line_lost_count = 0U;
        }
      }
      else if (is_lost_line_turn_task(g_mode) != 0U)
      {
        if ((g_mode == MENU_MODE_ADV_DOUBLE_LOOP) &&
            (circle_corner_count >= ADV_DOUBLE_CORNER_DONE_COUNT) &&
            (double_corner_wait_phase == DOUBLE_CORNER_WAIT_NONE) &&
            (task_finished == 0U))
        {
          circle_turn_active = 0U;
          double_corner_wait_phase = DOUBLE_CORNER_WAIT_FORWARD;
          double_corner_wait_start_tick = HAL_GetTick();
          double_corner_next_action = DOUBLE_CORNER_NEXT_FINISH;
          left_pwm = BASIC_LINE_BASE_PWM;
          right_pwm = BASIC_LINE_BASE_PWM;
          load_motor_pwm(left_pwm, right_pwm);
        }

        if ((is_circle_task(g_mode) != 0U) && (basic_corner_wait_phase != BASIC_CORNER_WAIT_NONE))
        {
          if (basic_corner_wait_phase == BASIC_CORNER_WAIT_FORWARD)
          {
            left_pwm = BASIC_LINE_BASE_PWM;
            right_pwm = BASIC_LINE_BASE_PWM;
            load_motor_pwm(left_pwm, right_pwm);
            if ((HAL_GetTick() - basic_corner_wait_start_tick) >= BASIC_CORNER_FORWARD_MS)
            {
              basic_corner_wait_phase = BASIC_CORNER_WAIT_STOP;
              basic_corner_wait_start_tick = HAL_GetTick();
              load_motor_pwm(0, 0);
              left_pwm = 0;
              right_pwm = 0;
            }
          }
          else
          {
            load_motor_pwm(0, 0);
            left_pwm = 0;
            right_pwm = 0;
            if ((HAL_GetTick() - basic_corner_wait_start_tick) >= BASIC_CORNER_STOP_MS)
            {
              basic_corner_wait_phase = BASIC_CORNER_WAIT_NONE;
              task_finished = 1U;
            }
          }
        }
        else if ((g_mode == MENU_MODE_ADV_DOUBLE_LOOP) && (double_corner_wait_phase != DOUBLE_CORNER_WAIT_NONE))
        {
          if (double_corner_wait_phase == DOUBLE_CORNER_WAIT_FORWARD)
          {
            left_pwm = BASIC_LINE_BASE_PWM;
            right_pwm = BASIC_LINE_BASE_PWM;
            load_motor_pwm(left_pwm, right_pwm);
            if ((HAL_GetTick() - double_corner_wait_start_tick) >= DOUBLE_CORNER_FORWARD_MS)
            {
              double_corner_wait_phase = DOUBLE_CORNER_WAIT_STOP;
              double_corner_wait_start_tick = HAL_GetTick();
              load_motor_pwm(0, 0);
              left_pwm = 0;
              right_pwm = 0;
            }
          }
          else
          {
            load_motor_pwm(0, 0);
            left_pwm = 0;
            right_pwm = 0;
            if ((HAL_GetTick() - double_corner_wait_start_tick) >= DOUBLE_CORNER_STOP_MS)
            {
              double_corner_wait_phase = DOUBLE_CORNER_WAIT_NONE;
              if (double_corner_next_action == DOUBLE_CORNER_NEXT_TURN_BACK)
              {
                double_turn_phase = DOUBLE_PHASE_TURN_BACK;
                double_turn_back_angle_done = 0U;
                double_turn_back_start_tick = HAL_GetTick();
                target_yaw = yaw_add_deg(g_yaw_jy61, DOUBLE_TURN_BACK_DEG);
                set_pid_target(&g_pid_turn_angle, target_yaw);
                g_pid_turn_angle.integral = 0.0f;
                g_pid_turn_angle.err_last = 0.0f;
                circle_dir = double_phase_turn_dir(double_turn_phase);
              }
              if (double_corner_next_action == DOUBLE_CORNER_NEXT_FINISH)
              {
                task_finished = 1U;
              }
              else
              {
                circle_turn_active = 1U;
              }
            }
          }
        }
        else if (circle_turn_active != 0U)
        {
          if ((g_mode == MENU_MODE_ADV_DOUBLE_LOOP) && (double_turn_phase == DOUBLE_PHASE_TURN_BACK))
          {
            if ((double_turn_back_angle_done == 0U) &&
                (yaw_abs_error(target_yaw, g_yaw_jy61) <= DOUBLE_TURN_BACK_DONE_ERR))
            {
              double_turn_back_angle_done = 1U;
            }

            if (double_turn_back_angle_done != 0U)
            {
              circle_turn_active = 0U;
              line_lost_count = 0U;
              double_turn_phase = DOUBLE_PHASE_CCW;
              circle_dir = double_phase_turn_dir(double_turn_phase);
              double_turn_back_start_tick = 0U;
              double_ccw_start_count_once_until = HAL_GetTick() + DOUBLE_CCW_START_COUNT_ONCE_MS;
              double_ccw_start_counted = 0U;
              reset_yaw_target(g_yaw_jy61, &target_yaw);
            }
          }
          else if (g_line_sensor_count != 0U)
          {
            circle_turn_active = 0U;
            line_lost_count = 0U;
          }
        }
        else if (g_line_sensor_count == 0U)
        {
          if ((g_mode == MENU_MODE_ADV_DOUBLE_LOOP) &&
              (double_corner_ignore_until != 0U) &&
              ((int32_t)(HAL_GetTick() - double_corner_ignore_until) < 0))
          {
            line_lost_count = 0U;
          }
          else if ((g_mode == MENU_MODE_ADV_DOUBLE_LOOP) &&
              (double_turn_phase == DOUBLE_PHASE_CCW) &&
              (double_ccw_start_counted != 0U) &&
              (double_ccw_start_count_once_until != 0U) &&
              ((int32_t)(HAL_GetTick() - double_ccw_start_count_once_until) < 0))
          {
            line_lost_count = 0U;
          }
          else if (++line_lost_count >= BASIC_LINE_LOST_STOP_COUNT)
          {
            line_lost_count = 0U;
            circle_corner_count++;
            task_count_beep();
            if (g_mode == MENU_MODE_ADV_DOUBLE_LOOP)
            {
              double_corner_ignore_until = HAL_GetTick() + DOUBLE_CORNER_COUNT_IGNORE_MS;
            }

            if ((g_mode == MENU_MODE_ADV_DOUBLE_LOOP) &&
                (double_turn_phase == DOUBLE_PHASE_CCW) &&
                (double_ccw_start_count_once_until != 0U) &&
                ((int32_t)(HAL_GetTick() - double_ccw_start_count_once_until) < 0))
            {
              double_ccw_start_counted = 1U;
            }

            if ((g_mode == MENU_MODE_ADV_DOUBLE_LOOP) && (circle_corner_count == BASIC_CIRCLE_CORNER_DONE_COUNT))
            {
              double_corner_wait_phase = DOUBLE_CORNER_WAIT_FORWARD;
              double_corner_wait_start_tick = HAL_GetTick();
              double_corner_next_action = DOUBLE_CORNER_NEXT_TURN_BACK;
            }
            else if (((g_mode == MENU_MODE_ADV_DOUBLE_LOOP) && (circle_corner_count >= ADV_DOUBLE_CORNER_DONE_COUNT)) ||
                     ((is_circle_task(g_mode) != 0U) && (circle_corner_count >= BASIC_CIRCLE_CORNER_DONE_COUNT)))
            {
              if (g_mode == MENU_MODE_ADV_DOUBLE_LOOP)
              {
                double_corner_wait_phase = DOUBLE_CORNER_WAIT_FORWARD;
                double_corner_wait_start_tick = HAL_GetTick();
                double_corner_next_action = DOUBLE_CORNER_NEXT_FINISH;
                left_pwm = BASIC_LINE_BASE_PWM;
                right_pwm = BASIC_LINE_BASE_PWM;
                load_motor_pwm(left_pwm, right_pwm);
              }
              else
              {
                basic_corner_wait_phase = BASIC_CORNER_WAIT_FORWARD;
                basic_corner_wait_start_tick = HAL_GetTick();
                left_pwm = BASIC_LINE_BASE_PWM;
                right_pwm = BASIC_LINE_BASE_PWM;
                load_motor_pwm(left_pwm, right_pwm);
              }
            }
            else
            {
              if (g_mode == MENU_MODE_ADV_DOUBLE_LOOP)
              {
                circle_dir = double_phase_turn_dir(double_turn_phase);
                circle_turn_active = 1U;
              }
              else
              {
                circle_turn_active = 1U;
              }
            }
          }
        }
        else
        {
          line_lost_count = 0U;
        }
      }

      if (((g_mode != MENU_MODE_ADV_DIAGONAL_AC) || (diagonal_phase == DIAGONAL_PHASE_LINE_TRACK)) &&
          (circle_turn_active == 0U) &&
          !((g_mode == MENU_MODE_BASIC_SINGLE_AB) && (basic_single_wait_phase != BASIC_SINGLE_WAIT_NONE)) &&
          !((is_circle_task(g_mode) != 0U) && (basic_corner_wait_phase != BASIC_CORNER_WAIT_NONE)) &&
          !((g_mode == MENU_MODE_ADV_DOUBLE_LOOP) && (double_corner_wait_phase != DOUBLE_CORNER_WAIT_NONE)) &&
          (task_finished == 0U))
      {
        line_corr = (int)(line_offset * BASIC_LINE_KP);
        if (line_corr > BASIC_LINE_MAX_CORR)
        {
          line_corr = BASIC_LINE_MAX_CORR;
        }
        else if (line_corr < -BASIC_LINE_MAX_CORR)
        {
          line_corr = -BASIC_LINE_MAX_CORR;
        }

        left_pwm = clamp_motor_pwm(BASIC_LINE_BASE_PWM + line_corr);
        right_pwm = clamp_motor_pwm(BASIC_LINE_BASE_PWM - line_corr);
        load_motor_pwm(left_pwm, right_pwm);
      }
      else if ((circle_turn_active != 0U) && (task_finished == 0U))
      {
        turn_pwm_search = (double_turn_phase == DOUBLE_PHASE_TURN_BACK) ? (BASIC_TURN_SEARCH_PWM + 300) : BASIC_TURN_SEARCH_PWM;

        if ((g_mode == MENU_MODE_ADV_DOUBLE_LOOP) && (double_turn_phase == DOUBLE_PHASE_TURN_BACK))
        {
          left_pwm = clamp_motor_pwm(-circle_dir * turn_pwm_search);
          right_pwm = clamp_motor_pwm(circle_dir * turn_pwm_search);
        }
        else
        {
          left_pwm = clamp_motor_pwm(BASIC_TURN_FORWARD_PWM - circle_dir * turn_pwm_search);
          right_pwm = clamp_motor_pwm(BASIC_TURN_FORWARD_PWM + circle_dir * turn_pwm_search);
        }
        load_motor_pwm(left_pwm, right_pwm);
      }

      if ((g_mode == MENU_MODE_ADV_DOUBLE_LOOP) &&
          (double_turn_phase == DOUBLE_PHASE_TURN_BACK) &&
          (double_turn_back_start_tick != 0U) &&
          ((HAL_GetTick() - double_turn_back_start_tick) >= 6000U))
      {
        double_turn_back_angle_done = 1U;
      }

      if (((is_circle_task(g_mode) != 0U) &&
           (basic_corner_wait_phase == BASIC_CORNER_WAIT_NONE) &&
           ((HAL_GetTick() - task_start_tick) >= BASIC_CIRCLE_TIMEOUT_MS)) ||
          ((g_mode == MENU_MODE_ADV_DOUBLE_LOOP) &&
           (double_corner_wait_phase == DOUBLE_CORNER_WAIT_NONE) &&
           ((HAL_GetTick() - task_start_tick) >= ADV_DOUBLE_TIMEOUT_MS)))
      {
        task_finished = 1U;
      }

      if (task_finished != 0U)
      {
        drive_enabled = 0U;
        task_active = 0U;
        left_pwm = 0;
        right_pwm = 0;
        current_drive_pwm = 0;
        motors_stop_safe();
        task_finish_beep();
      }
    }

    target_drive_pwm = ((drive_enabled != 0U) && (task_active == 0U) && (drive_mode == DRIVE_STRAIGHT_RUN)) ? BASE_DRIVE_PWM : 0;
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
      line_offset = line_err();
      line_state = g_line_sensor_state;

      OLED_Clear();
      menu_show_status(g_mode, drive_enabled, g_yaw_jy61, target_yaw, line_offset, line_state,
                       circle_corner_count, circle_turn_active, manual_target_count, left_pwm, right_pwm);
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

#ifdef  USE_FULL_ASSERT
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
