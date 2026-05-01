#include "app_tasks.h"
#include "scheduler.h"

#include "buzzer_drv.h"
#include "button_drv.h"
#include "chassis_uart_bridge.h"
#include "debug_uart.h"
#include "dflink_chassis_motion.h"
#include "fw_config.h"

static uint8_t s_q1_start_req;
static uint8_t s_q2_case1_start_req;
static uint8_t s_q2_case2_start_req;

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
  uint8_t rx;
  uint32_t now_ms;

  ButtonDrv_Process();

  if (ButtonDrv_WasPressed(BUTTON_DRV_PD3)) {
    s_q1_start_req = 1U;
    (void)DebugUart_Send((const uint8_t *)"Q1 start by PD3\r\n", 17U, 100U);
  } else if (ButtonDrv_WasPressed(BUTTON_DRV_PD4)) {
    s_q2_case1_start_req = 1U;
    (void)DebugUart_Send((const uint8_t *)"Q2-1 start by PD4\r\n", 19U, 100U);
  } else if (ButtonDrv_WasPressed(BUTTON_DRV_PD5)) {
    s_q2_case2_start_req = 1U;
    (void)DebugUart_Send((const uint8_t *)"Q2-2 start by PD5\r\n", 19U, 100U);
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
    if (!DebugUart_ReadByte(&rx)) {
      break;
    }

    if (rx == 0x08U) {
      static const uint8_t ack_q1[] = "ACK 0x08 -> Q1\r\n";
      s_q1_start_req = 1U;
      (void)DebugUart_Send(ack_q1, (uint16_t)(sizeof(ack_q1) - 1U), 100U);
      continue;
    }
    if (rx == 0x00U) {
      static const uint8_t ack_q21[] = "ACK 0x00 -> Q2-1\r\n";
      s_q2_case1_start_req = 1U;
      (void)DebugUart_Send(ack_q21, (uint16_t)(sizeof(ack_q21) - 1U), 100U);
      continue;
    }
    if (rx == 0x01U) {
      static const uint8_t ack_q22[] = "ACK 0x01 -> Q2-2\r\n";
      s_q2_case2_start_req = 1U;
      (void)DebugUart_Send(ack_q22, (uint16_t)(sizeof(ack_q22) - 1U), 100U);
      continue;
    }

    /* 对非控制字节保持单字节回显，便于串口调试 */
    (void)DebugUart_Send(&rx, 1U, 50U);
  }
}

static void App_Task_50ms(void)
{
}

static void App_Task_100ms(void)
{
#if FW_Q1_ENABLE != 0
  static uint8_t s_mode; /* 1=Q1, 2=Q2-1, 3=Q2-2 */
  static uint8_t s_state;
  static uint8_t s_wait_ticks;
  static uint8_t s_round;
  static uint8_t s_ret_seg;
  HAL_StatusTypeDef st;

  const int32_t pre_move_y_m21739 = 1739; /* 0.08m * 21739 */
  const int32_t move_px_m21739 = 0;
  const int32_t move_py_m21739 = 21739; /* 1.0m * 21739 */
  const int32_t move_py_ret_last_m21739 = 19565; /* 0.9m * 21739 */
  const int32_t move_py_q1_last_m21739 = 20000; /* 0.92m * 21739 */
  const int32_t move_pz_m21739 = 0;
  const int16_t move_speed_mps100 = 3000; /* 30 m/s * 100 */
  const int32_t rot_left_90 = -900000;  /* -90deg * 10000 */
  const int32_t rot_right_90 = 900000;  /* +90deg * 10000 */
  const int32_t rot_u_turn = -1800000;  /* -180deg * 10000 */
  const int16_t rot_vmax = 1500;        /* 15 deg/s * 100 */

  if (ChassisUartBridge_IsPassthrough()) {
    return;
  }

  if (s_q1_start_req != 0U) {
    s_q1_start_req = 0U;
    s_mode = 1U;
    s_state = 1U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
    (void)DebugUart_Send((const uint8_t *)"Q1 restart\r\n", 12U, 100U);
  } else if (s_q2_case1_start_req != 0U) {
    s_q2_case1_start_req = 0U;
    s_mode = 2U;
    s_state = 1U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
    (void)DebugUart_Send((const uint8_t *)"Q2-1 restart\r\n", 14U, 100U);
  } else if (s_q2_case2_start_req != 0U) {
    s_q2_case2_start_req = 0U;
    s_mode = 3U;
    s_state = 1U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
    (void)DebugUart_Send((const uint8_t *)"Q2-2 restart\r\n", 14U, 100U);
  }

  switch (s_state) {
    case 0U: /* 等待 PD3(Q1) / PD4(Q2-1) / PD5(Q2-2) */
      return;

    case 1U: /* 启动后先前进 0.08m */
      st = DflinkChassis_SendAdaptConstPMove(0, pre_move_y_m21739, 0,
                                             move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      if (s_mode == 1U) {
        (void)DebugUart_Send((const uint8_t *)"Q1 pre-move ok\r\n", 16U, 100U);
      } else if (s_mode == 2U) {
        (void)DebugUart_Send((const uint8_t *)"Q2-1 pre-move ok\r\n", 18U, 100U);
      } else {
        (void)DebugUart_Send((const uint8_t *)"Q2-2 pre-move ok\r\n", 18U, 100U);
      }
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

    case 3U: /* Q1/Q2-1左转90；Q2-2右转90 */
      st = DflinkChassis_SendRotation(
          0, 0, (s_mode == 3U) ? rot_right_90 : rot_left_90, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
      }
      if (s_mode == 1U) {
        (void)DebugUart_Send((const uint8_t *)"Q1 turn ok\r\n", 12U, 100U);
      } else if (s_mode == 2U) {
        (void)DebugUart_Send((const uint8_t *)"Q2-1 turn ok\r\n", 14U, 100U);
      } else {
        (void)DebugUart_Send((const uint8_t *)"Q2-2 turn ok\r\n", 14U, 100U);
      }
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

    case 5U: /* 直走: Q1最后一段0.92m，Q2固定1m */
      st = DflinkChassis_SendAdaptConstPMove(
          move_px_m21739,
          (s_mode == 1U && s_round >= 3U) ? move_py_q1_last_m21739 : move_py_m21739,
          move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      if (s_mode == 1U) {
        (void)DebugUart_Send((const uint8_t *)"Q1 move ok\r\n", 12U, 100U);
      } else if (s_mode == 2U) {
        (void)DebugUart_Send((const uint8_t *)"Q2-1 move ok\r\n", 14U, 100U);
      } else {
        (void)DebugUart_Send((const uint8_t *)"Q2-2 move ok\r\n", 14U, 100U);
      }
      s_wait_ticks = 0U;
      s_state = 6U;
      break;

    case 6U: /* 行进稳定等待约 2s */
      if (s_wait_ticks++ < 20U) {
        return;
      }
      s_wait_ticks = 0U;
      s_round++;
      if (s_round >= ((s_mode == 1U) ? 4U : 3U)) {
        if (s_mode == 1U) {
          (void)DebugUart_Send((const uint8_t *)"Q1 done\r\n", 9U, 100U);
          s_state = 7U;
        } else if (s_mode == 2U) {
          (void)DebugUart_Send((const uint8_t *)"Q2-1 outbound done\r\n", 20U, 100U);
          s_ret_seg = 0U;
          s_state = 7U;
        } else {
          (void)DebugUart_Send((const uint8_t *)"Q2-2 outbound done\r\n", 20U, 100U);
          s_ret_seg = 0U;
          s_state = 7U;
        }
      } else {
        s_state = 3U;
      }
      break;

    case 7U: /* Q2 调头；Q1 已完成停机 */
      if (s_mode < 2U || s_mode > 3U) {
        return;
      }
      st = DflinkChassis_SendRotation(0, 0, rot_u_turn, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
      }
      if (s_mode == 2U) {
        (void)DebugUart_Send((const uint8_t *)"Q2-1 return uturn ok\r\n", 23U, 100U);
      } else {
        (void)DebugUart_Send((const uint8_t *)"Q2-2 return uturn ok\r\n", 23U, 100U);
      }
      s_wait_ticks = 0U;
      s_state = 8U;
      break;

    case 8U: /* 调头稳定等待约 0.5s */
      if (s_wait_ticks++ < 5U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 9U;
      break;

    case 9U: /* Q2 返程：先两段1m，最后0.9m */
      st = DflinkChassis_SendAdaptConstPMove(
          move_px_m21739, (s_ret_seg < 2U) ? move_py_m21739 : move_py_ret_last_m21739,
          move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      if (s_ret_seg < 2U) {
        if (s_mode == 2U) {
          (void)DebugUart_Send((const uint8_t *)"Q2-1 return move 1m ok\r\n", 24U, 100U);
        } else {
          (void)DebugUart_Send((const uint8_t *)"Q2-2 return move 1m ok\r\n", 24U, 100U);
        }
      } else {
        if (s_mode == 2U) {
          (void)DebugUart_Send((const uint8_t *)"Q2-1 return move 0.9m ok\r\n", 25U, 100U);
        } else {
          (void)DebugUart_Send((const uint8_t *)"Q2-2 return move 0.9m ok\r\n", 25U, 100U);
        }
      }
      s_wait_ticks = 0U;
      s_state = 10U;
      break;

    case 10U: /* 返程位移稳定等待约 2s */
      if (s_wait_ticks++ < 20U) {
        return;
      }
      s_wait_ticks = 0U;
      if (s_ret_seg < 2U) {
        s_state = 11U;
      } else {
        s_state = 12U;
      }
      break;

    case 11U: /* Q2返程拐点: Q2-1右转90, Q2-2左转90 */
      st = DflinkChassis_SendRotation(
          0, 0, (s_mode == 2U) ? rot_right_90 : rot_left_90, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
      }
      if (s_mode == 2U) {
        (void)DebugUart_Send((const uint8_t *)"Q2-1 return turn ok\r\n", 21U, 100U);
      } else {
        (void)DebugUart_Send((const uint8_t *)"Q2-2 return turn ok\r\n", 21U, 100U);
      }
      s_wait_ticks = 0U;
      s_state = 14U;
      break;

    case 14U: /* 右转稳定等待约 0.5s */
      if (s_wait_ticks++ < 5U) {
        return;
      }
      s_wait_ticks = 0U;
      s_ret_seg++;
      s_state = 9U;
      break;

    case 12U: /* Q2 完成（含原路返回） */
      if (s_mode == 2U) {
        (void)DebugUart_Send((const uint8_t *)"Q2-1 done\r\n", 11U, 100U);
      } else {
        (void)DebugUart_Send((const uint8_t *)"Q2-2 done\r\n", 11U, 100U);
      }
      s_state = 13U;
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
