#include "menu.h"
#include "key.h"
#include "oled.h"
#include "stdio.h"

extern uint8_t g_oledstring[];

typedef enum
{
    MENU_GROUP_LINE = 0,
    MENU_GROUP_BT,
    MENU_GROUP_COUNT
} menu_group_t;

typedef struct
{
    const char *name;
    uint8_t mode;
} menu_item_t;

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
    "Line CW",
    "Line CCW",
    "Line Target",
    "Phone BT",
    "Voice BT"
};

static const char *s_group_names[MENU_GROUP_COUNT] =
{
    "Line Mode",
    "BT Mode"
};

static const menu_item_t s_line_menu_items[] =
{
    {"Line CW", MENU_MODE_LINE_CW},
    {"Line CCW", MENU_MODE_LINE_CCW},
    {"Line Target", MENU_MODE_LINE_TARGET_CORNERS}
};

static const menu_item_t s_bt_menu_items[] =
{
    {"Phone BT", MENU_MODE_BT_REMOTE},
    {"Voice BT", MENU_MODE_VOICE_REMOTE}
};

const char *menu_get_mode_name(uint8_t mode)
{
    if (mode >= MENU_MODE_COUNT)
    {
        return "UNKNOWN";
    }

    return s_mode_names[mode];
}

static void menu_draw_group(uint8_t selected)
{
    uint8_t i;

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Main Menu", 16, 1);

    for (i = 0; i < MENU_GROUP_COUNT; i++)
    {
        sprintf((char *)g_oledstring, "%c%s", (i == selected) ? '>' : ' ', s_group_names[i]);
        OLED_ShowString(0, (uint8_t)(18 + i * 14), g_oledstring, 8, 1);
    }

    OLED_ShowString(0, 52, (uint8_t *)"K0:Next K1:Enter", 8, 1);
    OLED_Refresh();
}

static void menu_draw_submenu(uint8_t group, uint8_t selected)
{
    const menu_item_t *items = s_line_menu_items;
    uint8_t item_count = (uint8_t)(sizeof(s_line_menu_items) / sizeof(s_line_menu_items[0]));
    uint8_t i;

    if (group == MENU_GROUP_BT)
    {
        items = s_bt_menu_items;
        item_count = (uint8_t)(sizeof(s_bt_menu_items) / sizeof(s_bt_menu_items[0]));
    }

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)s_group_names[group], 16, 1);

    for (i = 0; i < item_count; i++)
    {
        sprintf((char *)g_oledstring, "%c%s", (i == selected) ? '>' : ' ', items[i].name);
        OLED_ShowString(0, (uint8_t)(18 + i * 11), g_oledstring, 8, 1);
    }

    OLED_ShowString(0, 52, (uint8_t *)"K0:Next K1:OK", 8, 1);
    OLED_Refresh();
}

static uint8_t menu_select_submenu(uint8_t group)
{
    const menu_item_t *items = s_line_menu_items;
    uint8_t item_count = (uint8_t)(sizeof(s_line_menu_items) / sizeof(s_line_menu_items[0]));
    uint8_t selected = 0U;
    uint8_t key_num;

    if (group == MENU_GROUP_BT)
    {
        items = s_bt_menu_items;
        item_count = (uint8_t)(sizeof(s_bt_menu_items) / sizeof(s_bt_menu_items[0]));
    }

    while (1)
    {
        key_num = key_scan(0);
        if (key_num != 0U)
        {
            switch (key_num)
            {
                case KEY0_PRES:
                    menu_beep(40);
                    selected = (uint8_t)((selected + 1U) % item_count);
                    break;

                case KEY1_PRES:
                    menu_beep(80);
                    OLED_Clear();
                    OLED_ShowString(0, 0, (uint8_t *)"Mode Locked", 16, 1);
                    OLED_ShowString(0, 24, (uint8_t *)items[selected].name, 8, 1);
                    OLED_ShowString(0, 44, (uint8_t *)"Release keys...", 8, 1);
                    OLED_Refresh();
                    HAL_Delay(800);
                    return items[selected].mode;

                default:
                    break;
            }
        }

        menu_draw_submenu(group, selected);

        HAL_Delay(150);
    }
}

uint8_t menu_start_select(void)
{
    uint8_t selected_group = MENU_GROUP_LINE;
    uint8_t key_num = 0;

    while (1)
    {
        key_num = key_scan(0);
        if (key_num != 0U)
        {
            switch (key_num)
            {
                case KEY0_PRES:
                    menu_beep(40);
                    selected_group = (uint8_t)((selected_group + 1U) % MENU_GROUP_COUNT);
                    break;

                case KEY1_PRES:
                    menu_beep(80);
                    HAL_Delay(200);
                    return menu_select_submenu(selected_group);

                default:
                    break;
            }
        }

        menu_draw_group(selected_group);

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

    if (mode == MENU_MODE_LINE_TARGET_CORNERS)
    {
        lap_target = manual_target_count;
    }

    (void)target_yaw;

    OLED_ShowString(0, 0, (uint8_t *)menu_get_mode_name(mode), 8, 1);

    sprintf((char *)g_oledstring, "R:%s C:%u/%u",
            (drive_enabled == 0U) ? "SAFE" : "ON",
            circle_corner_count,
            lap_target);
    OLED_ShowString(0, 10, g_oledstring, 8, 1);

    sprintf((char *)g_oledstring, "Yaw:%6.1f", yaw);
    OLED_ShowString(0, 20, g_oledstring, 8, 1);

    sprintf((char *)g_oledstring, "Line:%5d", (int)line_err);
    OLED_ShowString(0, 30, g_oledstring, 8, 1);

    sprintf((char *)g_oledstring, "S:%02X T:%u", line_state, circle_turn_active);
    OLED_ShowString(0, 40, g_oledstring, 8, 1);

    sprintf((char *)g_oledstring, "L:%4d R:%4d", left_pwm, right_pwm);
    OLED_ShowString(0, 50, g_oledstring, 8, 1);
}
