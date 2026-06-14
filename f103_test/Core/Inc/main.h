/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOC
#define MANUAL_COUNT_KEY_Pin GPIO_PIN_15
#define MANUAL_COUNT_KEY_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOB
#define TASK_LED_Pin GPIO_PIN_1
#define TASK_LED_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_0
#define BIN2_GPIO_Port GPIOA
#define ENC_A2_Pin GPIO_PIN_1
#define ENC_A2_GPIO_Port GPIOA
#define ENC_A2_EXTI_IRQn EXTI1_IRQn
#define ENC_A1_Pin GPIO_PIN_2
#define ENC_A1_GPIO_Port GPIOA
#define ENC_A1_EXTI_IRQn EXTI2_IRQn
#define BIN1_Pin GPIO_PIN_3
#define BIN1_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_4
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_5
#define AIN2_GPIO_Port GPIOA
#define PWM_A_Pin GPIO_PIN_6
#define PWM_A_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_7
#define PWM_B_GPIO_Port GPIOA
#define ENC_B2_Pin GPIO_PIN_8
#define ENC_B2_GPIO_Port GPIOA
#define ENC_B2_EXTI_IRQn EXTI9_5_IRQn
#define KEY0_Pin GPIO_PIN_5
#define KEY0_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_14
#define KEY1_GPIO_Port GPIOC
#define LINE_R3_Pin GPIO_PIN_3
#define LINE_R3_GPIO_Port GPIOB
#define LINE_R4_Pin GPIO_PIN_4
#define LINE_R4_GPIO_Port GPIOB
#define LINE_R1_Pin GPIO_PIN_6
#define LINE_R1_GPIO_Port GPIOB
#define LINE_L4_Pin GPIO_PIN_12
#define LINE_L4_GPIO_Port GPIOB
#define LINE_L3_Pin GPIO_PIN_13
#define LINE_L3_GPIO_Port GPIOB
#define LINE_L2_Pin GPIO_PIN_14
#define LINE_L2_GPIO_Port GPIOB
#define LINE_L1_Pin GPIO_PIN_15
#define LINE_L1_GPIO_Port GPIOB
#define LINE_R2_Pin GPIO_PIN_15
#define LINE_R2_GPIO_Port GPIOA
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB
#define JY61P_RX_Pin GPIO_PIN_10
#define JY61P_RX_GPIO_Port GPIOB
#define JY61P_TX_Pin GPIO_PIN_11
#define JY61P_TX_GPIO_Port GPIOB
#define ENC_B1_Pin GPIO_PIN_11
#define ENC_B1_GPIO_Port GPIOA
#define ENC_B1_EXTI_IRQn EXTI15_10_IRQn

#define HW1_Pin LINE_L1_Pin
#define HW1_GPIO_Port LINE_L1_GPIO_Port
#define HW2_Pin LINE_L2_Pin
#define HW2_GPIO_Port LINE_L2_GPIO_Port
#define HW3_Pin LINE_L3_Pin
#define HW3_GPIO_Port LINE_L3_GPIO_Port
#define HW4_Pin LINE_L4_Pin
#define HW4_GPIO_Port LINE_L4_GPIO_Port
#define SDA_6050_Pin LINE_R3_Pin
#define SDA_6050_GPIO_Port LINE_R3_GPIO_Port
#define SCL_6050_Pin LINE_R4_Pin
#define SCL_6050_GPIO_Port LINE_R4_GPIO_Port

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
