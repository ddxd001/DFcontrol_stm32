#include "app_tasks.h"
#include "scheduler.h"

#include "buzzer_drv.h"
#include "button_drv.h"
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
  static uint8_t s_beep_remain;
  static uint8_t s_beep_stage;
  static uint8_t s_pending_beeps;
  static uint32_t s_beep_deadline_ms;
  uint32_t now_ms;

  ButtonDrv_Process();

  if (ButtonDrv_WasPressed(BUTTON_DRV_PD3)) {
    s_pending_beeps = 1U;
  } else if (ButtonDrv_WasPressed(BUTTON_DRV_PD4)) {
    s_pending_beeps = 2U;
  } else if (ButtonDrv_WasPressed(BUTTON_DRV_PD5)) {
    s_pending_beeps = 3U;
  }

  if (s_beep_stage == 0U && s_pending_beeps > 0U) {
    s_beep_remain = s_pending_beeps;
    s_pending_beeps = 0U;
    s_beep_stage = 1U;
    s_beep_deadline_ms = 0U;
  }

  now_ms = HAL_GetTick();
  if (s_beep_stage == 1U && (int32_t)(now_ms - s_beep_deadline_ms) >= 0) {
    if (s_beep_remain > 0U) {
      BuzzerDrv_Beep(80U);
      s_beep_remain--;
      s_beep_deadline_ms = now_ms + 180U;
      s_beep_stage = (s_beep_remain > 0U) ? 1U : 2U;
    }
  } else if (s_beep_stage == 2U && (int32_t)(now_ms - s_beep_deadline_ms) >= 0) {
    s_beep_stage = 0U;
  }

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
  ButtonDrv_Init();

  (void)Scheduler_AddTask(App_Task_1ms,   1U);
  (void)Scheduler_AddTask(App_Task_10ms, 10U);
  (void)Scheduler_AddTask(App_Task_50ms, 50U);
  (void)Scheduler_AddTask(App_Task_100ms, 100U);
}
