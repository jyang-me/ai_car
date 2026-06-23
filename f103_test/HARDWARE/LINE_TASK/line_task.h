#ifndef __LINE_TASK_H
#define __LINE_TASK_H

#include "main.h"

#define LINE_TASK_DEFAULT_TARGET 4U

typedef struct
{
  uint8_t active;
  uint8_t finished;
  uint32_t start_tick;
  uint8_t corner_count;
  uint8_t turn_active;
  uint8_t lost_count;
  uint8_t corner_wait_phase;
  uint32_t corner_wait_start_tick;
  uint8_t manual_target_count;
  int8_t turn_dir;
  int32_t line_offset;
  uint8_t line_state;
} line_task_t;

void line_task_reset(line_task_t *task);
uint8_t line_task_is_mode(uint8_t mode);
void line_task_start(line_task_t *task, uint8_t mode);
void line_task_stop(line_task_t *task);
void line_task_increment_target(line_task_t *task, uint8_t mode, uint8_t drive_enabled);
uint8_t line_task_update(line_task_t *task, uint8_t mode, int *left_pwm, int *right_pwm);

#endif
