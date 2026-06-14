/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "jy61p.h"
#include "protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BT_RX_BUF_SIZE 512U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t dr;
volatile uint8_t sr_status;
uint8_t g_usart2_receivedata = 0;
uint8_t g_bt_receivedata = 0;
static uint8_t g_bt_rx_buf[BT_RX_BUF_SIZE];
static volatile uint16_t g_bt_rx_w = 0;
static volatile uint16_t g_bt_rx_r = 0;
volatile uint32_t g_bt_rx_count = 0;
volatile uint32_t g_bt_error_count = 0;
volatile uint32_t g_bt_overflow_count = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef huart3;

static void jy61p_restart_uart_rx(void)
{
  HAL_UART_AbortReceive(&huart3);
  __HAL_UART_CLEAR_PEFLAG(&huart3);
  huart3.ErrorCode = HAL_UART_ERROR_NONE;
  g_jy61p_uart_start_status = HAL_UART_Receive_IT(&huart3, &g_usart2_receivedata, 1);
  g_jy61p_restart_count++;
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/

void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

void TIM1_UP_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim4);
}

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart3);
}

void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(ENC_A2_Pin);
}

void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(ENC_A1_Pin);
}

void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(ENC_B2_Pin);
}

void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(ENC_B1_Pin);
}

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  encoder_exti_callback(GPIO_Pin);
}

uint8_t bt_uart_read_byte(uint8_t *data)
{
  if (g_bt_rx_r == g_bt_rx_w)
  {
    return 0U;
  }

  *data = g_bt_rx_buf[g_bt_rx_r];
  g_bt_rx_r = (uint16_t)((g_bt_rx_r + 1U) % BT_RX_BUF_SIZE);
  return 1U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    uint16_t next_w = (uint16_t)((g_bt_rx_w + 1U) % BT_RX_BUF_SIZE);

    if (next_w != g_bt_rx_r)
    {
      g_bt_rx_buf[g_bt_rx_w] = g_bt_receivedata;
      g_bt_rx_w = next_w;
    }
    else
    {
      g_bt_overflow_count++;
    }
    g_bt_rx_count++;
    HAL_UART_Receive_IT(&huart1, &g_bt_receivedata, 1);
  }
  else if (huart == &huart3)
  {
    jy61p_ReceiveData(g_usart2_receivedata);
    g_jy61p_uart_start_status = HAL_UART_Receive_IT(&huart3, &g_usart2_receivedata, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    g_bt_error_count++;
    HAL_UART_AbortReceive(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    huart1.ErrorCode = HAL_UART_ERROR_NONE;
    HAL_UART_Receive_IT(&huart1, &g_bt_receivedata, 1);
  }
  else if (huart == &huart3)
  {
    g_jy61p_error_count++;
    g_jy61p_last_uart_error = huart->ErrorCode;
    jy61p_restart_uart_rx();
  }
}
/* USER CODE END 1 */
