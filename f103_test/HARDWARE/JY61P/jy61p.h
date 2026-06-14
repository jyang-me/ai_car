#ifndef __JY61P_H
#define __JY61P_H

#include "main.h" 

void jy61p_ReceiveData(uint8_t RxData);

extern float g_roll_jy61,g_pitch_jy61,g_yaw_jy61;
extern volatile uint32_t g_jy61p_rx_count;
extern volatile uint32_t g_jy61p_angle_count;
extern volatile uint32_t g_jy61p_error_count;
extern volatile uint32_t g_jy61p_protocol_error_count;
extern volatile uint32_t g_jy61p_current_baud;
extern volatile uint8_t g_jy61p_last_byte;
extern volatile uint8_t g_jy61p_uart_start_status;
extern volatile uint8_t g_jy61p_last_frame_type;
extern volatile uint32_t g_jy61p_last_uart_error;
extern volatile uint32_t g_jy61p_restart_count;

#endif
