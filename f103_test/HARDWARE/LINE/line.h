#ifndef __LINE_H
#define __LINE_H

#include "main.h"

#define LINE_L4   HAL_GPIO_ReadPin(LINE_L4_GPIO_Port, LINE_L4_Pin)
#define LINE_L3   HAL_GPIO_ReadPin(LINE_L3_GPIO_Port, LINE_L3_Pin)
#define LINE_L2   HAL_GPIO_ReadPin(LINE_L2_GPIO_Port, LINE_L2_Pin)
#define LINE_L1   HAL_GPIO_ReadPin(LINE_L1_GPIO_Port, LINE_L1_Pin)
#define LINE_R1   HAL_GPIO_ReadPin(LINE_R1_GPIO_Port, LINE_R1_Pin)
#define LINE_R2   HAL_GPIO_ReadPin(LINE_R2_GPIO_Port, LINE_R2_Pin)
#define LINE_R3   HAL_GPIO_ReadPin(LINE_R3_GPIO_Port, LINE_R3_Pin)
#define LINE_R4   HAL_GPIO_ReadPin(LINE_R4_GPIO_Port, LINE_R4_Pin)

#define LINE_SENSOR_NUM 8U
#define LINE_LOST_ERR   0

extern uint8_t g_line_sensor_state;
extern uint8_t g_line_sensor_count;
extern int32_t g_line_last_err;

uint8_t line_read_state(void);
int32_t line_err(void);
int32_t yaw_err0(void);
int32_t yaw_err180(void);

#endif
