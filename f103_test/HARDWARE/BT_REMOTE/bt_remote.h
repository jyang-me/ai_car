#ifndef __BT_REMOTE_H
#define __BT_REMOTE_H

#include "main.h"

#define BT_MODE_REMOTE   0U
#define BT_MODE_OBSTACLE 1U
#define BT_MODE_FOLLOW   2U

#define BT_SPEED_LEVEL_MIN 1U
#define BT_SPEED_LEVEL_MAX 5U
#define BT_SPEED_LEVEL_DEFAULT 3U
#define BT_REMOTE_MOVE_MS 2000U

typedef struct
{
  uint8_t motion;
  uint8_t command_motion;
  uint8_t command_level;
  uint8_t last_move_motion;
  uint8_t last_move_level;
  uint8_t last_raw_cmd;
  uint8_t car_mode;
  uint8_t speed_level;
  uint8_t yaw_lock_ready;
  uint8_t turn_active;
  uint8_t turn_stable_count;
  uint32_t turn_start_tick;
  uint32_t last_angle_count;
  uint32_t rx_count;
  uint32_t last_rx_tick;
  float yaw_target;
  int yaw_corr;
  int speed;
  int left_pwm;
  int right_pwm;
} bt_remote_state_t;

extern volatile uint8_t Fore;
extern volatile uint8_t Back;
extern volatile uint8_t Left;
extern volatile uint8_t Right;
extern volatile uint8_t Car_Mode;
extern volatile uint8_t Speed_Level;
extern volatile uint8_t Bluetooth_Last_Data;
extern volatile uint16_t Bluetooth_Rx_Count;
extern volatile uint8_t g_bt_receivedata;
extern volatile uint32_t g_bt_error_count;

void bt_remote_start_rx(void);
void bt_remote_parse_data(uint8_t data);
void bt_remote_on_rx_byte(uint8_t data);
void bt_remote_on_uart_error(void);
void bt_remote_send_data(char data);
void bt_remote_send_string(char *string);

void bt_remote_reset(bt_remote_state_t *state);
void bt_remote_update(bt_remote_state_t *state, uint8_t drive_enabled, float current_yaw, uint32_t angle_count);
void bt_remote_show_status(uint8_t mode, uint8_t drive_enabled, const bt_remote_state_t *state);

#endif
