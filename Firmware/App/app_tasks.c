#include "app_tasks.h"
#include "scheduler.h"

#include "buzzer_drv.h"
#include "chassis_uart_bridge.h"
#include "debug_uart.h"

static void App_Task_1ms(void)
{
  BuzzerDrv_Process();
  /* 460800 UART5：透传在 10ms 才跑易 ORE，改 1ms 轮询 DR */
  if (ChassisUartBridge_IsPassthrough()) {
    ChassisUartBridge_Process();
  }
}

static void App_Task_10ms(void)
{
  if (ChassisUartBridge_IsPassthrough()) {
    return;
  }

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

void App_RegisterTasks(void)
{
  (void)Scheduler_AddTask(App_Task_1ms,   1U);
  (void)Scheduler_AddTask(App_Task_10ms, 10U);
  (void)Scheduler_AddTask(App_Task_50ms, 50U);
  (void)Scheduler_AddTask(App_Task_100ms, 100U);
}
