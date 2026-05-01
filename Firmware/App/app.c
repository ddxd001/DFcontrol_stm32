#include "app.h"
#include "bsp.h"
#include "scheduler.h"
#include "fw_fault.h"
#include "app_tasks.h"

void App_Init(void)
{
  Fault_Init();
  Bsp_Init();
}

void App_StartScheduling(void)
{
  Scheduler_Init();
  App_RegisterTasks();
}
