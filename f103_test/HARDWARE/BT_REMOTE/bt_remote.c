#include "bt_remote.h"
#include "menu.h"
#include "motor.h"
#include "oled.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"

#define BT_REMOTE_BASE_PWM 600
#define BT_REMOTE_SPEED_STEP 200
#define BT_REMOTE_YAW_KP 22
#define BT_REMOTE_YAW_MAX_CORR 600
#define BT_REMOTE_TURN_MIN_PWM 650
#define BT_REMOTE_TURN_MAX_PWM 1200
#define BT_REMOTE_YAW_DONE_ERR 3.0f
#define BT_REMOTE_TURN_STABLE_COUNT 3U
#define BT_REMOTE_TURN_TIMEOUT_MS 3500U

extern uint8_t g_oledstring[];

volatile uint8_t Fore = 0;
volatile uint8_t Back = 0;
volatile uint8_t Left = 0;
volatile uint8_t Right = 0;
volatile uint8_t Car_Mode = BT_MODE_FOLLOW;
volatile uint8_t Speed_Level = BT_SPEED_LEVEL_DEFAULT;
volatile uint8_t Bluetooth_Last_Data = 0;
volatile uint8_t Bluetooth_Last_Motion = 'S';
volatile uint8_t Bluetooth_Last_Level = 0;
volatile uint8_t Bluetooth_Last_Move_Motion = 'S';
volatile uint8_t Bluetooth_Last_Move_Level = 0;
static volatile uint8_t s_wait_command_level = 0;
volatile uint16_t Bluetooth_Rx_Count = 0;
volatile uint8_t g_bt_receivedata = 0;
volatile uint32_t g_bt_error_count = 0;

static int bt_clamp_motor_pwm(int pwm)
{
  if (pwm > 2400)
  {
    pwm = 2400;
  }
  else if (pwm < -2400)
  {
    pwm = -2400;
  }

  return pwm;
}

static int bt_speed_from_level(uint8_t level)
{
  return BT_REMOTE_BASE_PWM + ((int)level * BT_REMOTE_SPEED_STEP);
}

static float bt_yaw_add(float yaw, float add_deg)
{
  yaw += add_deg;

  while (yaw > 180.0f)
  {
    yaw -= 360.0f;
  }
  while (yaw < -180.0f)
  {
    yaw += 360.0f;
  }

  return yaw;
}

static float bt_yaw_error(float target_yaw, float current_yaw)
{
  float err = target_yaw - current_yaw;

  while (err > 180.0f)
  {
    err -= 360.0f;
  }
  while (err < -180.0f)
  {
    err += 360.0f;
  }

  return err;
}

static float bt_abs_float(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static int bt_limit_yaw_corr(float corr)
{
  if (corr > (float)BT_REMOTE_YAW_MAX_CORR)
  {
    corr = (float)BT_REMOTE_YAW_MAX_CORR;
  }
  else if (corr < -(float)BT_REMOTE_YAW_MAX_CORR)
  {
    corr = -(float)BT_REMOTE_YAW_MAX_CORR;
  }

  return (int)corr;
}

static int bt_turn_pwm_from_error(float yaw_err)
{
  int pwm = bt_limit_yaw_corr(yaw_err * (float)BT_REMOTE_YAW_KP);

  if ((pwm > 0) && (pwm < BT_REMOTE_TURN_MIN_PWM))
  {
    pwm = BT_REMOTE_TURN_MIN_PWM;
  }
  else if ((pwm < 0) && (pwm > -BT_REMOTE_TURN_MIN_PWM))
  {
    pwm = -BT_REMOTE_TURN_MIN_PWM;
  }

  if (pwm > BT_REMOTE_TURN_MAX_PWM)
  {
    pwm = BT_REMOTE_TURN_MAX_PWM;
  }
  else if (pwm < -BT_REMOTE_TURN_MAX_PWM)
  {
    pwm = -BT_REMOTE_TURN_MAX_PWM;
  }

  return pwm;
}

static void bt_stop(void)
{
  Fore = 0;
  Back = 0;
  Left = 0;
  Right = 0;
}

static void bt_direction(uint8_t fore, uint8_t back, uint8_t left, uint8_t right)
{
  Car_Mode = BT_MODE_REMOTE;
  Fore = fore;
  Back = back;
  Left = left;
  Right = right;
}

static void bt_speed_up(void)
{
  if (Speed_Level < BT_SPEED_LEVEL_MAX)
  {
    Speed_Level++;
  }
}

static void bt_speed_down(void)
{
  if (Speed_Level > BT_SPEED_LEVEL_MIN)
  {
    Speed_Level--;
  }
}

static uint8_t bt_digit_to_level(uint8_t data)
{
  if ((data >= '0') && (data <= '9'))
  {
    uint8_t level = (uint8_t)(data - '0');
    if (level < BT_SPEED_LEVEL_MIN)
    {
      return BT_SPEED_LEVEL_MIN;
    }
    if (level > BT_SPEED_LEVEL_MAX)
    {
      return BT_SPEED_LEVEL_MAX;
    }
    return level;
  }

  return Speed_Level;
}

static void bt_record_motion(uint8_t motion)
{
  Bluetooth_Last_Motion = motion;
  if (motion == 'S')
  {
    Bluetooth_Last_Level = 0U;
  }
  else
  {
    Bluetooth_Last_Level = Speed_Level;
    Bluetooth_Last_Move_Motion = motion;
    Bluetooth_Last_Move_Level = Speed_Level;
  }
  s_wait_command_level = 1U;
}

static uint8_t bt_motion_from_flags(void)
{
  if (Fore != 0U)
  {
    return 'F';
  }
  if (Back != 0U)
  {
    return 'B';
  }
  if (Left != 0U)
  {
    return 'L';
  }
  if (Right != 0U)
  {
    return 'R';
  }

  return 'S';
}

void bt_remote_start_rx(void)
{
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_bt_receivedata, 1);
}

void bt_remote_parse_data(uint8_t data)
{
  Bluetooth_Last_Data = data;
  Bluetooth_Rx_Count++;

  if ((s_wait_command_level != 0U) && (data >= '0') && (data <= '9'))
  {
    if (Bluetooth_Last_Motion == 'S')
    {
      Bluetooth_Last_Level = 0U;
      bt_stop();
    }
    else
    {
      Speed_Level = bt_digit_to_level(data);
      Bluetooth_Last_Level = Speed_Level;
      Bluetooth_Last_Move_Motion = Bluetooth_Last_Motion;
      Bluetooth_Last_Move_Level = Speed_Level;
    }
    s_wait_command_level = 0U;
    return;
  }

  s_wait_command_level = 0U;

  switch (data)
  {
  case 0x00:
  case 'S':
  case 's':
  case '0':
    bt_stop();
    bt_record_motion('S');
    break;

  case 0x01:
  case 'F':
  case 'f':
  case '1':
  case 'W':
  case 'w':
    bt_direction(1U, 0U, 0U, 0U);
    bt_record_motion('F');
    break;

  case 0x05:
  case 'B':
  case 'b':
  case '2':
  case 'X':
  case 'x':
    bt_direction(0U, 1U, 0U, 0U);
    bt_record_motion('B');
    break;

  case 0x03:
  case 'L':
  case 'l':
  case '3':
  case 'A':
  case 'a':
    bt_direction(0U, 0U, 1U, 0U);
    bt_record_motion('L');
    break;

  case 0x07:
  case 'R':
  case 'r':
  case '4':
  case 'D':
  case 'd':
    bt_direction(0U, 0U, 0U, 1U);
    bt_record_motion('R');
    break;

  case 0x09:
  case '+':
  case '6':
    bt_speed_up();
    break;

  case 0x0A:
  case '-':
  case '7':
    bt_speed_down();
    break;

  case 0x10:
  case 'M':
  case 'm':
  case '8':
    Car_Mode = BT_MODE_REMOTE;
    bt_stop();
    break;

  case 0x11:
  case 'O':
  case 'o':
  case '9':
    Car_Mode = BT_MODE_OBSTACLE;
    bt_stop();
    break;

  case 0x12:
  case 'T':
  case 't':
    Car_Mode = BT_MODE_FOLLOW;
    bt_stop();
    break;

  case '\r':
    break;

  default:
    break;
  }
}

void bt_remote_on_rx_byte(uint8_t data)
{
  bt_remote_parse_data(data);
  bt_remote_start_rx();
}

void bt_remote_on_uart_error(void)
{
  g_bt_error_count++;
  HAL_UART_AbortReceive(&huart1);
  __HAL_UART_CLEAR_PEFLAG(&huart1);
  huart1.ErrorCode = HAL_UART_ERROR_NONE;
  bt_remote_start_rx();
}

void bt_remote_send_data(char data)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&data, 1U, 0xFFFFU);
}

void bt_remote_send_string(char *string)
{
  uint16_t len = (uint16_t)strlen(string);
  uint16_t i;

  for (i = 0; i < len; i++)
  {
    bt_remote_send_data(*string++);
  }
}

void bt_remote_reset(bt_remote_state_t *state)
{
  __disable_irq();
  bt_stop();
  Car_Mode = BT_MODE_REMOTE;
  Bluetooth_Last_Data = 0U;
  Bluetooth_Last_Motion = 'S';
  Bluetooth_Last_Level = 0U;
  Bluetooth_Last_Move_Motion = 'S';
  Bluetooth_Last_Move_Level = 0U;
  s_wait_command_level = 0U;
  Bluetooth_Rx_Count = 0U;
  Speed_Level = BT_SPEED_LEVEL_DEFAULT;
  __enable_irq();

  state->motion = 'S';
  state->command_motion = 'S';
  state->command_level = 0U;
  state->last_move_motion = 'S';
  state->last_move_level = 0U;
  state->last_raw_cmd = 0U;
  state->car_mode = Car_Mode;
  state->speed_level = Speed_Level;
  state->yaw_lock_ready = 0U;
  state->turn_active = 0U;
  state->turn_stable_count = 0U;
  state->turn_start_tick = 0U;
  state->last_angle_count = 0U;
  state->rx_count = 0U;
  state->last_rx_tick = HAL_GetTick();
  state->yaw_target = 0.0f;
  state->yaw_corr = 0;
  state->speed = bt_speed_from_level(Speed_Level);
  state->left_pwm = 0;
  state->right_pwm = 0;
  load_motor_pwm(0, 0);
}

void bt_remote_update(bt_remote_state_t *state, uint8_t drive_enabled, float current_yaw, uint32_t angle_count)
{
  uint8_t fore = Fore;
  uint8_t back = Back;
  uint8_t left = Left;
  uint8_t right = Right;
  uint8_t car_mode = Car_Mode;
  uint8_t yaw_ready = (angle_count != 0U) ? 1U : 0U;
  float yaw_err = 0.0f;

  if (state->rx_count != Bluetooth_Rx_Count)
  {
    state->rx_count = Bluetooth_Rx_Count;
    state->last_rx_tick = HAL_GetTick();
  }

  state->motion = bt_motion_from_flags();
  state->command_motion = Bluetooth_Last_Motion;
  state->command_level = Bluetooth_Last_Level;
  state->last_move_motion = Bluetooth_Last_Move_Motion;
  state->last_move_level = Bluetooth_Last_Move_Level;
  state->last_raw_cmd = Bluetooth_Last_Data;
  state->car_mode = car_mode;
  state->speed_level = Speed_Level;
  state->speed = bt_speed_from_level(Speed_Level);

  if ((drive_enabled == 0U) || (car_mode != BT_MODE_REMOTE))
  {
    state->motion = 'S';
    state->left_pwm = 0;
    state->right_pwm = 0;
    state->yaw_lock_ready = 0U;
    state->turn_active = 0U;
    state->turn_stable_count = 0U;
    state->yaw_corr = 0;
    load_motor_pwm(0, 0);
    return;
  }

  if ((state->last_rx_tick != 0U) &&
      ((int32_t)(HAL_GetTick() - state->last_rx_tick) >= (int32_t)BT_REMOTE_MOVE_MS))
  {
    bt_stop();
    state->motion = 'S';
    state->left_pwm = 0;
    state->right_pwm = 0;
    state->yaw_lock_ready = 0U;
    state->turn_active = 0U;
    state->turn_stable_count = 0U;
    state->yaw_corr = 0;
    load_motor_pwm(0, 0);
    return;
  }

  if (fore != 0U)
  {
    state->turn_active = 0U;
    state->turn_stable_count = 0U;

    if ((yaw_ready != 0U) && (state->yaw_lock_ready == 0U))
    {
      state->yaw_target = current_yaw;
      state->yaw_lock_ready = 1U;
    }

    if ((yaw_ready != 0U) && (state->yaw_lock_ready != 0U))
    {
      yaw_err = bt_yaw_error(state->yaw_target, current_yaw);
      state->yaw_corr = bt_limit_yaw_corr(yaw_err * (float)BT_REMOTE_YAW_KP);
      state->left_pwm = bt_clamp_motor_pwm(state->speed - state->yaw_corr);
      state->right_pwm = bt_clamp_motor_pwm(state->speed + state->yaw_corr);
    }
    else
    {
      state->yaw_corr = 0;
      state->left_pwm = bt_clamp_motor_pwm(state->speed);
      state->right_pwm = bt_clamp_motor_pwm(state->speed);
    }
  }
  else if (back != 0U)
  {
    state->yaw_lock_ready = 0U;
    state->turn_active = 0U;
    state->turn_stable_count = 0U;
    state->yaw_corr = 0;
    state->left_pwm = bt_clamp_motor_pwm(-state->speed);
    state->right_pwm = bt_clamp_motor_pwm(-state->speed);
  }
  else if (left != 0U)
  {
    if ((yaw_ready != 0U) && (state->turn_active == 0U))
    {
      state->yaw_target = bt_yaw_add(current_yaw, 90.0f);
      state->turn_active = 1U;
      state->turn_stable_count = 0U;
      state->turn_start_tick = HAL_GetTick();
      state->yaw_lock_ready = 0U;
    }
  }
  else if (right != 0U)
  {
    if ((yaw_ready != 0U) && (state->turn_active == 0U))
    {
      state->yaw_target = bt_yaw_add(current_yaw, -90.0f);
      state->turn_active = 1U;
      state->turn_stable_count = 0U;
      state->turn_start_tick = HAL_GetTick();
      state->yaw_lock_ready = 0U;
    }
  }
  else
  {
    state->yaw_lock_ready = 0U;
    state->turn_active = 0U;
    state->turn_stable_count = 0U;
    state->yaw_corr = 0;
  }

  if (state->turn_active != 0U)
  {
    if (yaw_ready != 0U)
    {
      yaw_err = bt_yaw_error(state->yaw_target, current_yaw);
      state->yaw_corr = bt_limit_yaw_corr(yaw_err * (float)BT_REMOTE_YAW_KP);

      if (bt_abs_float(yaw_err) <= BT_REMOTE_YAW_DONE_ERR)
      {
        state->left_pwm = 0;
        state->right_pwm = 0;
        state->yaw_corr = 0;
        if (++state->turn_stable_count >= BT_REMOTE_TURN_STABLE_COUNT)
        {
          bt_stop();
          state->turn_active = 0U;
          state->turn_stable_count = 0U;
          state->motion = 'S';
        }
      }
      else
      {
        state->turn_stable_count = 0U;
        state->yaw_corr = bt_turn_pwm_from_error(yaw_err);
        state->left_pwm = bt_clamp_motor_pwm(-state->yaw_corr);
        state->right_pwm = bt_clamp_motor_pwm(state->yaw_corr);
      }
    }

    if ((state->turn_active != 0U) &&
        ((HAL_GetTick() - state->turn_start_tick) >= BT_REMOTE_TURN_TIMEOUT_MS))
    {
      bt_stop();
      state->turn_active = 0U;
      state->turn_stable_count = 0U;
      state->motion = 'S';
      state->left_pwm = 0;
      state->right_pwm = 0;
      state->yaw_corr = 0;
    }
  }
  else if ((fore == 0U) && (back == 0U))
  {
    state->left_pwm = 0;
    state->right_pwm = 0;
  }

  state->last_angle_count = angle_count;
  load_motor_pwm(state->left_pwm, state->right_pwm);
}

void bt_remote_show_status(uint8_t mode, uint8_t drive_enabled, const bt_remote_state_t *state)
{
  char raw_char = '-';

  (void)drive_enabled;

  if ((state->last_raw_cmd >= 32U) && (state->last_raw_cmd <= 126U))
  {
    raw_char = (char)state->last_raw_cmd;
  }

  OLED_ShowString(0, 0, (uint8_t *)menu_get_mode_name(mode), 16, 1);

  sprintf((char *)g_oledstring, "Cmd:%c%u Raw:%c",
          state->command_motion,
          state->command_level,
          raw_char);
  OLED_ShowString(0, 20, g_oledstring, 8, 1);

  sprintf((char *)g_oledstring, "Left :%4d", state->left_pwm);
  OLED_ShowString(0, 34, g_oledstring, 8, 1);

  sprintf((char *)g_oledstring, "Right:%4d", state->right_pwm);
  OLED_ShowString(0, 48, g_oledstring, 8, 1);
}
