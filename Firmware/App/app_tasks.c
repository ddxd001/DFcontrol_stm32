#include "app_tasks.h"
#include "scheduler.h"

#include "buzzer_drv.h"
#include "button_drv.h"
#include "chassis_uart_bridge.h"
#include "debug_uart.h"
#include "dflink_chassis_motion.h"
#include "fw_config.h"

static uint8_t s_q1_start_req;

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
    s_q1_start_req = 1U;
    (void)DebugUart_Send((const uint8_t *)"Q1 start by PD3\r\n", 17U, 100U);
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
#if FW_Q1_ENABLE != 0
  static uint8_t s_state;
  static uint8_t s_wait_ticks;
  static uint8_t s_round;
  HAL_StatusTypeDef st;

  const int32_t pre_move_y_m21739 = 1739; /* 0.08m * 21739 */
  const int32_t move_px_m21739 = 0;
  const int32_t move_py_m21739 = 21739; /* 1.0m * 21739 */
  const int32_t move_py_last_m21739 = 20000; /* 0.92m * 21739 */
  const int32_t move_pz_m21739 = 0;
  const int16_t move_speed_mps100 = 3000; /* 30 m/s * 100 */
  const int32_t rot_left_90 = -900000;  /* -90deg * 10000 */
  const int16_t rot_vmax = 1500;        /* 15 deg/s * 100 */

  if (ChassisUartBridge_IsPassthrough()) {
    return;
  }

  if (s_q1_start_req != 0U) {
    s_q1_start_req = 0U;
    s_state = 1U;
    s_wait_ticks = 0U;
    s_round = 0U;
    (void)DebugUart_Send((const uint8_t *)"Q1 restart\r\n", 12U, 100U);
  }

  switch (s_state) {
    case 0U: /* 等待 PD3 开始按钮 */
      return;

    case 1U: /* 启动后先前进 0.05m */
      st = DflinkChassis_SendAdaptConstPMove(0, pre_move_y_m21739, 0,
                                             move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      (void)DebugUart_Send((const uint8_t *)"Q1 pre-move ok\r\n", 16U, 100U);
      s_wait_ticks = 0U;
      s_state = 2U;
      break;

    case 2U: /* 前置前进稳定等待约 0.5s */
      if (s_wait_ticks++ < 5U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 3U;
      break;

    case 3U: /* 左转90度 */
      st = DflinkChassis_SendRotation(0, 0, rot_left_90, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
      }
      (void)DebugUart_Send((const uint8_t *)"Q1 turn ok\r\n", 12U, 100U);
      s_wait_ticks = 0U;
      s_state = 4U;
      break;

    case 4U: /* 转向稳定等待约 0.5s */
      if (s_wait_ticks++ < 5U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 5U;
      break;

    case 5U: /* 直走：前3轮1m，最后1轮0.92m */
      st = DflinkChassis_SendAdaptConstPMove(
          move_px_m21739, (s_round >= 3U) ? move_py_last_m21739 : move_py_m21739,
          move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      (void)DebugUart_Send((const uint8_t *)"Q1 move ok\r\n", 12U, 100U);
      s_wait_ticks = 0U;
      s_state = 6U;
      break;

    case 6U: /* 行进稳定等待约 2s */
      if (s_wait_ticks++ < 20U) {
        return;
      }
      s_wait_ticks = 0U;
      s_round++;
      if (s_round >= 4U) {
        (void)DebugUart_Send((const uint8_t *)"Q1 done\r\n", 9U, 100U);
        s_state = 7U;
      } else {
        s_state = 3U;
      }
      break;

    default:
      break;
  }
#endif
}

void App_RegisterTasks(void)
{
  ButtonDrv_Init();

  (void)Scheduler_AddTask(App_Task_1ms,   1U);
  (void)Scheduler_AddTask(App_Task_10ms, 10U);
  (void)Scheduler_AddTask(App_Task_50ms, 50U);
  (void)Scheduler_AddTask(App_Task_100ms, 100U);
}
