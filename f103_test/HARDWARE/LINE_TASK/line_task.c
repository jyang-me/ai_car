#include "line_task.h"
#include "line.h"
#include "menu.h"
#include "motor.h"

#define LINE_BASE_PWM 1200
#define LINE_KP 22
#define LINE_MAX_CORR 700
#define LINE_TURN_SEARCH_PWM 1050
#define LINE_TURN_FORWARD_PWM 450
#define LINE_CIRCLE_TIMEOUT_MS 18000U
#define LINE_CIRCLE_DONE_COUNT 4U
#define LINE_LOST_STOP_COUNT 3U
#define LINE_CORNER_FORWARD_MS 200U
#define LINE_CORNER_STOP_MS 150U
#define LINE_CORNER_WAIT_NONE 0U
#define LINE_CORNER_WAIT_FORWARD 1U
#define LINE_CORNER_WAIT_STOP 2U

static int line_clamp_motor_pwm(int pwm)
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

static uint8_t line_task_is_circle_mode(uint8_t mode)
{
  return ((mode == MENU_MODE_LINE_CW) || (mode == MENU_MODE_LINE_CCW)) ? 1U : 0U;
}

static uint8_t line_task_is_manual_mode(uint8_t mode)
{
  return (mode == MENU_MODE_LINE_TARGET_CORNERS) ? 1U : 0U;
}

static int8_t line_task_turn_dir(uint8_t mode)
{
  return (mode == MENU_MODE_LINE_CW) ? -1 : 1;
}

static void line_task_count_beep(void)
{
  HAL_GPIO_WritePin(TASK_LED_GPIO_Port, TASK_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TASK_LED_GPIO_Port, TASK_LED_Pin, GPIO_PIN_SET);
}

void line_task_reset(line_task_t *task)
{
  task->active = 0U;
  task->finished = 0U;
  task->start_tick = 0U;
  task->corner_count = 0U;
  task->turn_active = 0U;
  task->lost_count = 0U;
  task->corner_wait_phase = LINE_CORNER_WAIT_NONE;
  task->corner_wait_start_tick = 0U;
  task->manual_target_count = LINE_TASK_DEFAULT_TARGET;
  task->turn_dir = 0;
  task->line_offset = 0;
  task->line_state = 0U;
}

uint8_t line_task_is_mode(uint8_t mode)
{
  return ((line_task_is_circle_mode(mode) != 0U) ||
          (line_task_is_manual_mode(mode) != 0U)) ? 1U : 0U;
}

void line_task_start(line_task_t *task, uint8_t mode)
{
  task->active = line_task_is_mode(mode);
  task->finished = 0U;
  task->start_tick = HAL_GetTick();
  task->corner_count = 0U;
  task->turn_active = 0U;
  task->lost_count = 0U;
  task->corner_wait_phase = LINE_CORNER_WAIT_NONE;
  task->corner_wait_start_tick = 0U;
  task->turn_dir = (mode == MENU_MODE_LINE_TARGET_CORNERS) ?
                   line_task_turn_dir(MENU_MODE_LINE_CW) : line_task_turn_dir(mode);
}

void line_task_stop(line_task_t *task)
{
  task->active = 0U;
  task->finished = 0U;
  task->turn_active = 0U;
  task->lost_count = 0U;
  task->corner_wait_phase = LINE_CORNER_WAIT_NONE;
  task->corner_wait_start_tick = 0U;
}

void line_task_increment_target(line_task_t *task, uint8_t mode, uint8_t drive_enabled)
{
  if ((mode == MENU_MODE_LINE_TARGET_CORNERS) && (drive_enabled == 0U))
  {
    task->manual_target_count++;
  }
}

uint8_t line_task_update(line_task_t *task, uint8_t mode, int *left_pwm, int *right_pwm)
{
  int line_corr;
  int turn_pwm_search;

  if (task->active == 0U)
  {
    return 0U;
  }

  task->line_offset = line_err();
  task->line_state = g_line_sensor_state;

  if (line_task_is_manual_mode(mode) != 0U)
  {
    if (task->turn_active != 0U)
    {
      if (g_line_sensor_count != 0U)
      {
        task->turn_active = 0U;
        task->lost_count = 0U;
      }
    }
    else if ((task->finished == 0U) && (g_line_sensor_count == 0U))
    {
      if (++task->lost_count >= LINE_LOST_STOP_COUNT)
      {
        task->lost_count = 0U;
        task->corner_count++;
        line_task_count_beep();

        if (task->corner_count >= task->manual_target_count)
        {
          task->finished = 1U;
        }
        else
        {
          task->turn_dir = line_task_turn_dir(MENU_MODE_LINE_CW);
          task->turn_active = 1U;
        }
      }
    }
    else
    {
      task->lost_count = 0U;
    }
  }
  else if (line_task_is_circle_mode(mode) != 0U)
  {
    if (task->corner_wait_phase != LINE_CORNER_WAIT_NONE)
    {
      if (task->corner_wait_phase == LINE_CORNER_WAIT_FORWARD)
      {
        *left_pwm = LINE_BASE_PWM;
        *right_pwm = LINE_BASE_PWM;
        load_motor_pwm(*left_pwm, *right_pwm);
        if ((HAL_GetTick() - task->corner_wait_start_tick) >= LINE_CORNER_FORWARD_MS)
        {
          task->corner_wait_phase = LINE_CORNER_WAIT_STOP;
          task->corner_wait_start_tick = HAL_GetTick();
          *left_pwm = 0;
          *right_pwm = 0;
          load_motor_pwm(0, 0);
        }
      }
      else
      {
        *left_pwm = 0;
        *right_pwm = 0;
        load_motor_pwm(0, 0);
        if ((HAL_GetTick() - task->corner_wait_start_tick) >= LINE_CORNER_STOP_MS)
        {
          task->corner_wait_phase = LINE_CORNER_WAIT_NONE;
          task->finished = 1U;
        }
      }
    }
    else if (task->turn_active != 0U)
    {
      if (g_line_sensor_count != 0U)
      {
        task->turn_active = 0U;
        task->lost_count = 0U;
      }
    }
    else if (g_line_sensor_count == 0U)
    {
      if (++task->lost_count >= LINE_LOST_STOP_COUNT)
      {
        task->lost_count = 0U;
        task->corner_count++;
        line_task_count_beep();

        if (task->corner_count >= LINE_CIRCLE_DONE_COUNT)
        {
          task->corner_wait_phase = LINE_CORNER_WAIT_FORWARD;
          task->corner_wait_start_tick = HAL_GetTick();
          *left_pwm = LINE_BASE_PWM;
          *right_pwm = LINE_BASE_PWM;
          load_motor_pwm(*left_pwm, *right_pwm);
        }
        else
        {
          task->turn_active = 1U;
        }
      }
    }
    else
    {
      task->lost_count = 0U;
    }
  }

  if ((task->turn_active == 0U) &&
      !((line_task_is_circle_mode(mode) != 0U) && (task->corner_wait_phase != LINE_CORNER_WAIT_NONE)) &&
      (task->finished == 0U))
  {
    line_corr = (int)(task->line_offset * LINE_KP);
    if (line_corr > LINE_MAX_CORR)
    {
      line_corr = LINE_MAX_CORR;
    }
    else if (line_corr < -LINE_MAX_CORR)
    {
      line_corr = -LINE_MAX_CORR;
    }

    *left_pwm = line_clamp_motor_pwm(LINE_BASE_PWM + line_corr);
    *right_pwm = line_clamp_motor_pwm(LINE_BASE_PWM - line_corr);
    load_motor_pwm(*left_pwm, *right_pwm);
  }
  else if ((task->turn_active != 0U) && (task->finished == 0U))
  {
    turn_pwm_search = LINE_TURN_SEARCH_PWM;
    *left_pwm = line_clamp_motor_pwm(LINE_TURN_FORWARD_PWM - task->turn_dir * turn_pwm_search);
    *right_pwm = line_clamp_motor_pwm(LINE_TURN_FORWARD_PWM + task->turn_dir * turn_pwm_search);
    load_motor_pwm(*left_pwm, *right_pwm);
  }

  if ((line_task_is_circle_mode(mode) != 0U) &&
      (task->corner_wait_phase == LINE_CORNER_WAIT_NONE) &&
      ((HAL_GetTick() - task->start_tick) >= LINE_CIRCLE_TIMEOUT_MS))
  {
    task->finished = 1U;
  }

  if (task->finished != 0U)
  {
    task->active = 0U;
    *left_pwm = 0;
    *right_pwm = 0;
    load_motor_pwm(0, 0);
    return 1U;
  }

  return 0U;
}
