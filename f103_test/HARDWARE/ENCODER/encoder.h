#ifndef  __ENCODER_H
#define  __ENCODER_H

#include  "main.h"

int read_pluse(TIM_HandleTypeDef *htim);
void encoder_exti_callback(uint16_t GPIO_Pin);

#endif


