#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>

typedef void (*sched_fn_t)(void);

void Scheduler_Init(void);
int Scheduler_AddTask(sched_fn_t fn, uint32_t period_ms);
void Scheduler_RunPending(void);

#endif
