#include "scheduler.h"
#include "fw_config.h"
#include "stm32f4xx_hal.h"

typedef struct {
  sched_fn_t fn;
  uint32_t period_ms;
  uint32_t last_run_ms;
  uint8_t used;
} sched_task_t;

static sched_task_t s_tasks[FW_SCHED_MAX_TASKS];

void Scheduler_Init(void)
{
  uint32_t i;
  for (i = 0U; i < FW_SCHED_MAX_TASKS; i++) {
    s_tasks[i].fn = 0;
    s_tasks[i].period_ms = 0U;
    s_tasks[i].last_run_ms = 0U;
    s_tasks[i].used = 0U;
  }
}

int Scheduler_AddTask(sched_fn_t fn, uint32_t period_ms)
{
  uint32_t i;

  if (fn == 0 || period_ms == 0U) {
    return -1;
  }

  for (i = 0U; i < FW_SCHED_MAX_TASKS; i++) {
    if (!s_tasks[i].used) {
      s_tasks[i].fn = fn;
      s_tasks[i].period_ms = period_ms;
      s_tasks[i].last_run_ms = HAL_GetTick();
      s_tasks[i].used = 1U;
      return 0;
    }
  }

  return -1;
}

void Scheduler_RunPending(void)
{
  uint32_t i;
  uint32_t now;

  now = HAL_GetTick();

  for (i = 0U; i < FW_SCHED_MAX_TASKS; i++) {
    if (!s_tasks[i].used || s_tasks[i].fn == 0) {
      continue;
    }

    if ((uint32_t)(now - s_tasks[i].last_run_ms) >= s_tasks[i].period_ms) {
      s_tasks[i].last_run_ms = now;
      s_tasks[i].fn();
    }
  }
}
