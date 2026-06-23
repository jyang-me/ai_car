#ifndef __MENU_H
#define __MENU_H

#include "main.h"

typedef enum
{
    MENU_MODE_LINE_CW = 0,
    MENU_MODE_LINE_CCW,
    MENU_MODE_LINE_TARGET_CORNERS,
    MENU_MODE_BT_REMOTE,
    MENU_MODE_VOICE_REMOTE,
    MENU_MODE_COUNT
} menu_mode_t;

uint8_t menu_start_select(void);
const char *menu_get_mode_name(uint8_t mode);
void menu_show_status(uint8_t mode,
                      uint8_t drive_enabled,
                      float yaw,
                      float target_yaw,
                      int32_t line_err,
                      uint8_t line_state,
                      uint8_t circle_corner_count,
                      uint8_t circle_turn_active,
                      uint8_t manual_target_count,
                      int left_pwm,
                      int right_pwm);

#endif
