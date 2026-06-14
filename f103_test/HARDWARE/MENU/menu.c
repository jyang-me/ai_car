#include "menu.h"
#include "key.h"
#include "oled.h"
#include "stdio.h"
#include "line.h"
#include "jy61p.h"

#define MENU_GROUP_BASIC   0U
#define MENU_GROUP_ADVANCE 1U
#define MENU_GROUP_REMOTE  2U
#define MENU_GROUP_COUNT   3U
#define MENU_TASK_PER_GROUP 3U

extern uint8_t g_oledstring[];

static void menu_beep(uint16_t duration_ms)
{
    HAL_GPIO_WritePin(TASK_LED_GPIO_Port, TASK_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    HAL_Delay(duration_ms);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TASK_LED_GPIO_Port, TASK_LED_Pin, GPIO_PIN_SET);
}

static const char *s_mode_names[MENU_MODE_COUNT] =
{
    "1. Single A-B",
    "2. CW Circle",
    "3. CCW Circle",
    "1. Double Loop",
    "2. Diagonal AC",
    "3. Random Pnts",
    "BT Remote"
};

static uint8_t menu_group_task_count(uint8_t group)
{
    return (group == MENU_GROUP_REMOTE) ? 1U : MENU_TASK_PER_GROUP;
}

static uint8_t menu_group_to_mode(uint8_t group, uint8_t task)
{
    if (group == MENU_GROUP_ADVANCE)
    {
        return (uint8_t)(MENU_MODE_ADV_DOUBLE_LOOP + task);
    }
    else if (group == MENU_GROUP_REMOTE)
    {
        return MENU_MODE_BT_REMOTE;
    }

    return (uint8_t)(MENU_MODE_BASIC_SINGLE_AB + task);
}

const char *menu_get_mode_name(uint8_t mode)
{
    if (mode >= MENU_MODE_COUNT)
    {
        return "UNKNOWN";
    }

    return s_mode_names[mode];
}

static void menu_draw_group(uint8_t group)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Main Menu", 16, 1);

    OLED_ShowString(0, 16, (uint8_t *)((group == MENU_GROUP_BASIC) ? ">[1] Basic Tasks" : " [1] Basic Tasks"), 8, 1);
    OLED_ShowString(0, 28, (uint8_t *)((group == MENU_GROUP_ADVANCE) ? ">[2] Advance Task" : " [2] Advance Task"), 8, 1);
    OLED_ShowString(0, 40, (uint8_t *)((group == MENU_GROUP_REMOTE) ? ">[3] BT Remote" : " [3] BT Remote"), 8, 1);

    (void)line_err();
    OLED_ShowString(0, 56, (uint8_t *)"K0 NEXT K1 OK", 8, 1);
    OLED_Refresh();
}

static void menu_draw_task(uint8_t group, uint8_t task)
{
    uint8_t i;
    uint8_t mode;

    OLED_Clear();
    if (group == MENU_GROUP_REMOTE)
    {
        OLED_ShowString(0, 0, (uint8_t *)"Remote", 16, 1);
    }
    else
    {
        OLED_ShowString(0, 0, (uint8_t *)((group == MENU_GROUP_BASIC) ? "Basic Tasks" : "Advance Tasks"), 16, 1);
    }

    for (i = 0; i < menu_group_task_count(group); i++)
    {
        mode = menu_group_to_mode(group, i);
        sprintf((char *)g_oledstring, "%c%s", (i == task) ? '>' : ' ', menu_get_mode_name(mode));
        OLED_ShowString(0, (uint8_t)(18 + i * 10), g_oledstring, 8, 1);
    }

    OLED_ShowString(0, 56, (uint8_t *)"K0 NEXT K1 RUN", 8, 1);
    OLED_Refresh();
}

uint8_t menu_start_select(void)
{
    uint8_t group = MENU_GROUP_BASIC;
    uint8_t task = 0;
    uint8_t in_task_page = 0;
    uint8_t key_num = 0;
    uint8_t mode = 0;

    while (1)
    {
        key_num = key_scan(0);
        if (key_num != 0U)
        {
            switch (key_num)
            {
                case KEY0_PRES:
                    menu_beep(40);
                    if (in_task_page == 0U)
                    {
                        group = (uint8_t)((group + 1U) % MENU_GROUP_COUNT);
                    }
                    else
                    {
                        task = (uint8_t)((task + 1U) % menu_group_task_count(group));
                    }
                    break;

                case KEY1_PRES:
                    menu_beep(80);
                    if (in_task_page == 0U)
                    {
                        in_task_page = 1U;
                        task = 0;
                    }
                    else
                    {
                        mode = menu_group_to_mode(group, task);
                        OLED_Clear();
                        OLED_ShowString(0, 0, (uint8_t *)"Task Locked", 16, 1);
                        OLED_ShowString(0, 24, (uint8_t *)menu_get_mode_name(mode), 8, 1);
                        OLED_ShowString(0, 44, (uint8_t *)"Release keys...", 8, 1);
                        OLED_Refresh();
                        HAL_Delay(800);
                        return mode;
                    }
                    break;

                default:
                    break;
            }
        }

        if (in_task_page == 0U)
        {
            menu_draw_group(group);
        }
        else
        {
            menu_draw_task(group, task);
        }

        HAL_Delay(150);
    }
}

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
                      int right_pwm)
{
    uint8_t lap_target = 4U;

    if (mode == MENU_MODE_ADV_DOUBLE_LOOP)
    {
        lap_target = 9U;
    }
    else if (mode == MENU_MODE_ADV_DIAGONAL_AC)
    {
        lap_target = 3U;
    }
    else if (mode == MENU_MODE_ADV_RANDOM_POINTS)
    {
        lap_target = manual_target_count;
    }

    OLED_ShowString(0, 0, (uint8_t *)menu_get_mode_name(mode), 8, 1);

    sprintf((char *)g_oledstring, "Run:%s", (drive_enabled == 0U) ? "SAFE" : "ON  ");
    OLED_ShowString(0, 8, g_oledstring, 8, 1);

    sprintf((char *)g_oledstring, "Yaw:%7.2f", yaw);
    OLED_ShowString(0, 16, g_oledstring, 8, 1);

    sprintf((char *)g_oledstring, "Tar:%7.2f", target_yaw);
    OLED_ShowString(0, 24, g_oledstring, 8, 1);

    sprintf((char *)g_oledstring, "Line:%4ld %02X", line_err, line_state);
    OLED_ShowString(0, 32, g_oledstring, 8, 1);

    sprintf((char *)g_oledstring, "L:%4d R:%4d", left_pwm, right_pwm);
    OLED_ShowString(0, 40, g_oledstring, 8, 1);

    sprintf((char *)g_oledstring, "Lap:%u/%u T:%u",
            circle_corner_count,
            lap_target,
            circle_turn_active);
    OLED_ShowString(0, 48, g_oledstring, 8, 1);

    OLED_ShowString(0, 56, (uint8_t *)"K0 RUN K1 MENU", 8, 1);
}
