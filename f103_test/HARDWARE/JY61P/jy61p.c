#include "jy61p.h"

static uint8_t RxBuffer[11];
static uint8_t RxIndex = 0;

float g_roll_jy61, g_pitch_jy61, g_yaw_jy61;
volatile uint32_t g_jy61p_rx_count = 0;
volatile uint32_t g_jy61p_angle_count = 0;
volatile uint32_t g_jy61p_error_count = 0;
volatile uint32_t g_jy61p_protocol_error_count = 0;
volatile uint32_t g_jy61p_current_baud = 9600;
volatile uint8_t g_jy61p_last_byte = 0;
volatile uint8_t g_jy61p_uart_start_status = 0;
volatile uint8_t g_jy61p_last_frame_type = 0;
volatile uint32_t g_jy61p_last_uart_error = 0;
volatile uint32_t g_jy61p_restart_count = 0;

static int16_t jy61p_to_int16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

void jy61p_ReceiveData(uint8_t RxData)
{
    uint8_t i;
    uint8_t sum = 0;

    g_jy61p_rx_count++;
    g_jy61p_last_byte = RxData;

    if ((RxIndex != 0U) && (RxData == 0x55U))
    {
        RxIndex = 0;
    }

    if (RxIndex == 0)
    {
        if (RxData != 0x55)
        {
            return;
        }
        RxBuffer[RxIndex++] = RxData;
        return;
    }

    RxBuffer[RxIndex++] = RxData;

    if (RxIndex < sizeof(RxBuffer))
    {
        return;
    }

    for (i = 0; i < 10; i++)
    {
        sum += RxBuffer[i];
    }

    if ((sum == RxBuffer[10]) && (RxBuffer[1] == 0x53))
    {
        g_jy61p_last_frame_type = RxBuffer[1];
        g_roll_jy61 = (float)jy61p_to_int16(RxBuffer[2], RxBuffer[3]) / 32768.0f * 180.0f;
        g_pitch_jy61 = (float)jy61p_to_int16(RxBuffer[4], RxBuffer[5]) / 32768.0f * 180.0f;
        g_yaw_jy61 = (float)jy61p_to_int16(RxBuffer[6], RxBuffer[7]) / 32768.0f * 180.0f;
        g_jy61p_angle_count++;
    }
    else if (sum == RxBuffer[10])
    {
        g_jy61p_last_frame_type = RxBuffer[1];
    }
    else
    {
        g_jy61p_protocol_error_count++;
    }

    RxIndex = 0;
}
