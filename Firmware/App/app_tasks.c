#include "app_tasks.h"
#include "scheduler.h"

#include "buzzer_drv.h"
#include "debug_uart.h"

static void App_Task_1ms(void)
{
  BuzzerDrv_Process();
}

static void App_Task_10ms(void)
{
  while (DebugUart_RxAvail() > 0U) {
    uint32_t n = DebugUart_EchoRxToTx(160U, 200U);
    if (n == 0U) {
      break;
    }
  }
}

static void App_Task_50ms(void)
{
}

static void App_Task_100ms(void)
{
}

/* 自检：开机后约每隔 1s 短鸣一次，共 5 次；测完删掉本任务及相关注册 */
static void App_Task_BeepSelftest_1Hz(void)
{
  static uint8_t s_count;

  if (s_count >= 5U) {
    return;
  }

  BuzzerDrv_Beep(100U);
  s_count++;
}

void App_RegisterTasks(void)
{
  (void)Scheduler_AddTask(App_Task_1ms,   1U);
  (void)Scheduler_AddTask(App_Task_10ms, 10U);
  (void)Scheduler_AddTask(App_Task_50ms, 50U);
  (void)Scheduler_AddTask(App_Task_100ms, 100U);
  (void)Scheduler_AddTask(App_Task_BeepSelftest_1Hz, 1000U);
}
