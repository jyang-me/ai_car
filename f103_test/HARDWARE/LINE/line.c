#include "line.h"
#include "jy61p.h"

uint8_t g_line_sensor_state = 0;
uint8_t g_line_sensor_count = 0;
int32_t g_line_last_err = 0;

static uint8_t line_is_black(GPIO_PinState pin_state)
{
    return (pin_state == GPIO_PIN_RESET) ? 1U : 0U;
}

uint8_t line_read_state(void)
{
    uint8_t state = 0;

    if (line_is_black(LINE_L4) != 0U) { state |= (1U << 0); }
    if (line_is_black(LINE_L3) != 0U) { state |= (1U << 1); }
    if (line_is_black(LINE_L2) != 0U) { state |= (1U << 2); }
    if (line_is_black(LINE_L1) != 0U) { state |= (1U << 3); }
    if (line_is_black(LINE_R1) != 0U) { state |= (1U << 4); }
    if (line_is_black(LINE_R2) != 0U) { state |= (1U << 5); }
    if (line_is_black(LINE_R3) != 0U) { state |= (1U << 6); }
    if (line_is_black(LINE_R4) != 0U) { state |= (1U << 7); }

    g_line_sensor_state = state;
    return state;
}

int32_t line_err(void)
{
    static const int8_t weight[LINE_SENSOR_NUM] = {-35, -25, -15, -5, 5, 15, 25, 35};
    uint8_t state = line_read_state();
    int32_t weight_sum = 0;
    uint8_t active_count = 0;
    uint8_t i;

    for (i = 0; i < LINE_SENSOR_NUM; i++)
    {
        if ((state & (1U << i)) != 0U)
        {
            weight_sum += weight[i];
            active_count++;
        }
    }

    g_line_sensor_count = active_count;

    if (active_count == 0U)
    {
        g_line_last_err = LINE_LOST_ERR;
        return LINE_LOST_ERR;
    }

    g_line_last_err = weight_sum / active_count;
    return g_line_last_err;
}

int32_t yaw_err0(void)
{
    int32_t yaw_num;

    if ((g_yaw_jy61 > 0) && (g_yaw_jy61 < 90))
    {
        yaw_num = g_yaw_jy61;
    }
    else if ((g_yaw_jy61 > 270) && (g_yaw_jy61 < 360))
    {
        yaw_num = g_yaw_jy61 - 360;
    }
    else
    {
        yaw_num = 0;
    }

    return yaw_num;
}

int32_t yaw_err180(void)
{
    int32_t yaw_num;

    if ((g_yaw_jy61 > 180) && (g_yaw_jy61 < 270))
    {
        yaw_num = g_yaw_jy61 - 180;
    }
    else if ((g_yaw_jy61 > 90) && (g_yaw_jy61 < 180))
    {
        yaw_num = g_yaw_jy61 - 180;
    }
    else
    {
        yaw_num = 0;
    }

    return yaw_num;
}
