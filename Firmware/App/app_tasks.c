#include "app_tasks.h"
#include "scheduler.h"

#include "buzzer_drv.h"
#include "button_drv.h"
#include "chassis_uart_bridge.h"
#include "debug_uart.h"
#include "dflink_chassis_motion.h"
#include "fw_config.h"
#include "gw_gray_sensor.h"
#include "stepper_motor_drv.h"
#include "usart.h"

static uint8_t s_q1_start_req;
static uint8_t s_q2_case1_start_req;
static uint8_t s_q2_case2_start_req;
static uint8_t s_q3_case1_start_req;
static uint8_t s_q3_case2_start_req;
static uint8_t s_q4_start_req;
static uint8_t s_test1_start_req;
static uint8_t s_test2_start_req;
static uint8_t s_pending_beeps;
static StepperMotorDrv s_stepper_motor;
static uint8_t s_stepper_motor_inited;
static GwGraySensor s_gray_sensor;
static uint8_t s_gray_sensor_inited;
static uint8_t s_q3_case1_cross_detect_active;
static uint8_t s_q3_case1_cross_count;
static uint8_t s_q3_case1_cross_latched;
static uint8_t s_test1_dist_measure_active;
static uint8_t s_test1_last_cross_valid;
static uint32_t s_test1_last_cross_tick_ms;
#define APP_GRAY_CROSS_MIN_ACTIVE_BITS 3U

static void App_Motion_StopAndBeep(uint8_t *mode, uint8_t *state, uint8_t *wait_ticks,
                                   uint8_t *round, uint8_t *ret_seg)
{
  BuzzerDrv_Beep(80U);
  s_q3_case1_cross_detect_active = 0U;
  s_q3_case1_cross_count = 0U;
  s_q3_case1_cross_latched = 0U;
  s_test1_dist_measure_active = 0U;
  s_test1_last_cross_valid = 0U;
  s_test1_last_cross_tick_ms = 0U;
  *mode = 0U;
  *state = 0U;
  *wait_ticks = 0U;
  *round = 0U;
  *ret_seg = 0U;
}

static void App_SendCrossIndexed(uint8_t idx)
{
  uint8_t msg[12];
  uint8_t n;
  uint8_t d2;
  uint8_t d1;
  uint8_t d0;

  msg[0] = 'C';
  msg[1] = 'R';
  msg[2] = 'O';
  msg[3] = 'S';
  msg[4] = 'S';
  msg[5] = ':';
  n = 6U;

  if (idx >= 100U) {
    d2 = (uint8_t)(idx / 100U);
    d1 = (uint8_t)((idx % 100U) / 10U);
    d0 = (uint8_t)(idx % 10U);
    msg[n++] = (uint8_t)('0' + d2);
    msg[n++] = (uint8_t)('0' + d1);
    msg[n++] = (uint8_t)('0' + d0);
  } else if (idx >= 10U) {
    d1 = (uint8_t)(idx / 10U);
    d0 = (uint8_t)(idx % 10U);
    msg[n++] = (uint8_t)('0' + d1);
    msg[n++] = (uint8_t)('0' + d0);
  } else {
    msg[n++] = (uint8_t)('0' + idx);
  }

  msg[n++] = '\r';
  msg[n++] = '\n';
  (void)DebugUart_Send(msg, n, 50U);
}

static void App_SendDistCm1(uint32_t dist_cm_x10)
{
  uint8_t msg[22];
  uint8_t digits[10];
  uint8_t n;
  uint8_t dlen;
  uint32_t v;
  uint8_t frac;
  uint8_t i;

  msg[0] = 'D';
  msg[1] = 'I';
  msg[2] = 'S';
  msg[3] = 'T';
  msg[4] = ':';
  n = 5U;

  v = dist_cm_x10 / 10U;
  frac = (uint8_t)(dist_cm_x10 % 10U);
  if (v == 0U) {
    msg[n++] = '0';
  } else {
    dlen = 0U;
    while (v > 0U && dlen < (uint8_t)sizeof(digits)) {
      digits[dlen++] = (uint8_t)(v % 10U);
      v /= 10U;
    }
    for (i = 0U; i < dlen; i++) {
      msg[n++] = (uint8_t)('0' + digits[dlen - 1U - i]);
    }
  }

  msg[n++] = '.';
  msg[n++] = (uint8_t)('0' + frac);
  msg[n++] = 'c';
  msg[n++] = 'm';
  msg[n++] = '\r';
  msg[n++] = '\n';
  (void)DebugUart_Send(msg, n, 50U);
}

static void App_ProcessCrossDetect10ms(void)
{
  bool is_cross;

  if (s_q3_case1_cross_detect_active == 0U || s_gray_sensor_inited == 0U) {
    return;
  }

  if (GwGray_ReadAndDetectCross(&s_gray_sensor, APP_GRAY_CROSS_MIN_ACTIVE_BITS, &is_cross) &&
      is_cross != 0U) {
    if (s_q3_case1_cross_latched == 0U) {
      uint32_t now_ms;
      BuzzerDrv_Beep(80U);
      s_q3_case1_cross_count++;
      App_SendCrossIndexed(s_q3_case1_cross_count);
      if (s_test1_dist_measure_active != 0U) {
        now_ms = HAL_GetTick();
        if (s_test1_last_cross_valid != 0U) {
          uint32_t dt_ms;
          uint32_t dist_cm_x10;
          dt_ms = now_ms - s_test1_last_cross_tick_ms;
          /* 距离(cm,保留1位) = 时间(ms) * 0.33(m/s) * 0.71 * 100 * 10 / 1000
           * = 时间(ms) * 234.3 / 1000
           */
          dist_cm_x10 = (dt_ms * 2343U + 5000U) / 10000U;
          App_SendDistCm1(dist_cm_x10);
        }
        s_test1_last_cross_tick_ms = now_ms;
        s_test1_last_cross_valid = 1U;
      }
      s_q3_case1_cross_latched = 1U;
    }
  } else {
    s_q3_case1_cross_latched = 0U;
  }
}

static void App_Task_1ms(void)
{
  BuzzerDrv_Process();

  if (ChassisUartBridge_IsPassthrough()) {
    ChassisUartBridge_Process();
  }
}

static void App_Task_10ms(void)
{
  static uint8_t s_beep_remain;
  static uint8_t s_beep_stage;
  static uint8_t s_uart_seq_state;
  static uint32_t s_beep_deadline_ms;
  static uint8_t s_start_guard_init;
  static uint32_t s_start_guard_deadline_ms;
  const int32_t stepper_pulse_90deg = 800; /* 默认按 3200 脉冲/圈换算，需按实机标定 */
  const int32_t stepper_speed_turn = 100;
  uint32_t now_ms;

  now_ms = HAL_GetTick();
  ButtonDrv_Process();

  if (s_start_guard_init == 0U) {
    s_start_guard_init = 1U;
    s_start_guard_deadline_ms = now_ms + 1200U;
  }

  if ((int32_t)(now_ms - s_start_guard_deadline_ms) < 0) {
    /* 上电保护期：按键事件与请求全部忽略，防止误启动。 */
    (void)ButtonDrv_WasPressed(BUTTON_DRV_PD3);
    (void)ButtonDrv_WasPressed(BUTTON_DRV_PD4);
    (void)ButtonDrv_WasPressed(BUTTON_DRV_PD5);
    s_q1_start_req = 0U;
    s_q2_case1_start_req = 0U;
    s_q2_case2_start_req = 0U;
    s_q3_case1_start_req = 0U;
    s_q3_case2_start_req = 0U;
    s_q4_start_req = 0U;
    s_test1_start_req = 0U;
    s_test2_start_req = 0U;
  } else if (ButtonDrv_WasPressed(BUTTON_DRV_PD3)) {
    s_q1_start_req = 1U;
  } else if (ButtonDrv_WasPressed(BUTTON_DRV_PD4)) {
    s_q2_case1_start_req = 1U;
  } else if (ButtonDrv_WasPressed(BUTTON_DRV_PD5)) {
    s_q2_case2_start_req = 1U;
  }

  if (s_beep_stage == 0U && s_pending_beeps > 0U) {
    s_beep_remain = s_pending_beeps;
    s_pending_beeps = 0U;
    s_beep_stage = 1U;
    s_beep_deadline_ms = 0U;
  }

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

  App_ProcessCrossDetect10ms();

  if (ChassisUartBridge_IsPassthrough()) {
    return;
  }

  /* UART1 验证：收到什么就回显什么。 */
  {
    uint8_t rx;
    while (DebugUart_ReadByte(&rx)) {
      if (s_uart_seq_state == 0U) {
        s_uart_seq_state = (rx == 0xAAU) ? 1U : 0U;
      } else if (s_uart_seq_state == 1U) {
        if (rx == 0xBBU) {
          s_uart_seq_state = 2U;
        } else {
          s_uart_seq_state = (rx == 0xAAU) ? 1U : 0U;
        }
      } else {
        if (rx == 0x01U) {
          BuzzerDrv_Beep(80U);
        } else if (rx == 0x02U) {
          s_q1_start_req = 1U;
        } else if (rx == 0x03U) {
          s_q2_case1_start_req = 1U;
        } else if (rx == 0x04U) {
          s_q2_case2_start_req = 1U;
        } else if (rx == 0x05U) {
          s_q3_case1_start_req = 1U;
        } else if (rx == 0x06U) {
          s_q3_case2_start_req = 1U;
        } else if (rx == 0x07U) {
          s_q4_start_req = 1U;
        } else if (rx == 0xD0U) {
          s_test1_start_req = 1U;
        } else if (rx == 0xD1U) {
          s_test2_start_req = 1U;
        } else if (rx == 0x08U) {
          if (s_stepper_motor_inited != 0U) {
            (void)StepperMotorDrv_Enable(&s_stepper_motor);
          }
        } else if (rx == 0x09U) {
          if (s_stepper_motor_inited != 0U) {
            (void)StepperMotorDrv_Disable(&s_stepper_motor);
          }
        } else if (rx == 0x0AU) {
          if (s_stepper_motor_inited != 0U) {
            s_stepper_motor.pos_mode = STEPPER_POS_RELATIVE;
            s_stepper_motor.speed = stepper_speed_turn;
            s_stepper_motor.pulse = -stepper_pulse_90deg; /* 左转90度 */
            (void)StepperMotorDrv_RunPosition(&s_stepper_motor);
          }
        } else if (rx == 0x0BU) {
          if (s_stepper_motor_inited != 0U) {
            s_stepper_motor.pos_mode = STEPPER_POS_RELATIVE;
            s_stepper_motor.speed = stepper_speed_turn;
            s_stepper_motor.pulse = stepper_pulse_90deg; /* 右转90度 */
            (void)StepperMotorDrv_RunPosition(&s_stepper_motor);
          }
        }
        s_uart_seq_state = (rx == 0xAAU) ? 1U : 0U;
      }
    }
  }

}

static void App_Task_50ms(void)
{
}

static void App_Task_100ms(void)
{
#if FW_Q1_ENABLE != 0
  static uint8_t s_boot_guard_init;
  static uint32_t s_boot_guard_deadline_ms;
  static uint8_t s_mode; /* 1=Q1, 2=Q2-1, 3=Q2-2, 4=Q3-1, 5=Q3-2, 6=Q4, 7=TEST1, 8=TEST2 */
  static uint8_t s_state;
  static uint8_t s_wait_ticks;
  static uint8_t s_round;
  static uint8_t s_ret_seg;
  uint32_t now_ms;
  HAL_StatusTypeDef st;

  const int32_t pre_move_y_m21739 = 1739; /* 0.08m * 21739 */
  const int32_t move_px_m21739 = 0;
  const int32_t move_py_m21739 = 21739; /* 1.0m * 21739 */
  const int32_t move_py_test1_02_m21739 = 4348; /* 0.2m * 21739 */
  const int32_t move_py_test1_06_m21739 = 13043; /* 0.6m * 21739 */
  const int32_t move_py_half_m21739 = 10870; /* 0.5m * 21739 */
  const int32_t move_py_q4_first_m21739 = 11957; /* 0.55m * 21739 */
  const int32_t move_py_q4_second_m21739 = 9783; /* 0.45m * 21739 */
  const int32_t move_py_ret_last_m21739 = 19565; /* 0.9m * 21739 */
  const int32_t move_py_q1_last_m21739 = 20000; /* 0.92m * 21739 */
  const int32_t move_pz_m21739 = 0;
  const int16_t move_speed_mps100 = 3000; /* 30 m/s * 100 */
  const int16_t move_speed_q3_case1_uniform_mps100 = 1000; /* 10 m/s * 100 */
  const int16_t move_speed_q3_case2_uniform_mps100 = 1000; /* 10 m/s * 100 */
  const int32_t stepper_pulse_90deg = 800;
  const int32_t stepper_speed_turn = 100;
  const int32_t rot_left_90 = -900000;  /* -90deg * 10000 */
  const int32_t rot_right_90 = 900000;  /* +90deg * 10000 */
  const int32_t rot_u_turn = -1800000;  /* -180deg * 10000 */
  const int16_t rot_vmax = 1500;        /* 15 deg/s * 100 */
  const uint8_t wait_ticks_default = 14U;          /* 1.4s */
  const uint8_t wait_ticks_q3_case1_uniform = 28U; /* 2.8s */

  now_ms = HAL_GetTick();
  if (s_boot_guard_init == 0U) {
    s_boot_guard_init = 1U;
    s_boot_guard_deadline_ms = now_ms + 1200U;
  }
  if ((int32_t)(now_ms - s_boot_guard_deadline_ms) < 0) {
    /* 100ms 状态机兜底：保护期内强制空闲，防止任何早期误触发落到 UART5。 */
    s_q1_start_req = 0U;
    s_q2_case1_start_req = 0U;
    s_q2_case2_start_req = 0U;
    s_q3_case1_start_req = 0U;
    s_q3_case2_start_req = 0U;
    s_q4_start_req = 0U;
    s_test1_start_req = 0U;
    s_test2_start_req = 0U;
    s_mode = 0U;
    s_state = 0U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
    return;
  }

  if (s_q1_start_req != 0U) {
    s_q1_start_req = 0U;
    BuzzerDrv_Beep(80U);
    s_q3_case1_cross_detect_active = 0U;
    s_q3_case1_cross_count = 0U;
    s_q3_case1_cross_latched = 0U;
    s_test1_dist_measure_active = 0U;
    s_test1_last_cross_valid = 0U;
    s_test1_last_cross_tick_ms = 0U;
    s_mode = 1U;
    s_state = 1U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
  } else if (s_q2_case1_start_req != 0U) {
    s_q2_case1_start_req = 0U;
    BuzzerDrv_Beep(80U);
    s_q3_case1_cross_detect_active = 0U;
    s_q3_case1_cross_count = 0U;
    s_q3_case1_cross_latched = 0U;
    s_test1_dist_measure_active = 0U;
    s_test1_last_cross_valid = 0U;
    s_test1_last_cross_tick_ms = 0U;
    s_mode = 2U;
    s_state = 1U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
  } else if (s_q2_case2_start_req != 0U) {
    s_q2_case2_start_req = 0U;
    BuzzerDrv_Beep(80U);
    s_q3_case1_cross_detect_active = 0U;
    s_q3_case1_cross_count = 0U;
    s_q3_case1_cross_latched = 0U;
    s_test1_dist_measure_active = 0U;
    s_test1_last_cross_valid = 0U;
    s_test1_last_cross_tick_ms = 0U;
    s_mode = 3U;
    s_state = 1U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
  } else if (s_q3_case1_start_req != 0U) {
    s_q3_case1_start_req = 0U;
    BuzzerDrv_Beep(80U);
    s_q3_case1_cross_detect_active = 0U;
    s_q3_case1_cross_count = 0U;
    s_q3_case1_cross_latched = 0U;
    s_test1_dist_measure_active = 0U;
    s_test1_last_cross_valid = 0U;
    s_test1_last_cross_tick_ms = 0U;
    s_mode = 4U;
    s_state = 1U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
  } else if (s_q3_case2_start_req != 0U) {
    s_q3_case2_start_req = 0U;
    BuzzerDrv_Beep(80U);
    s_q3_case1_cross_detect_active = 0U;
    s_q3_case1_cross_count = 0U;
    s_q3_case1_cross_latched = 0U;
    s_test1_dist_measure_active = 0U;
    s_test1_last_cross_valid = 0U;
    s_test1_last_cross_tick_ms = 0U;
    s_mode = 5U;
    s_state = 1U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
  } else if (s_q4_start_req != 0U) {
    s_q4_start_req = 0U;
    BuzzerDrv_Beep(80U);
    s_q3_case1_cross_detect_active = 0U;
    s_q3_case1_cross_count = 0U;
    s_q3_case1_cross_latched = 0U;
    s_test1_dist_measure_active = 0U;
    s_test1_last_cross_valid = 0U;
    s_test1_last_cross_tick_ms = 0U;
    s_mode = 6U;
    s_state = 1U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
  } else if (s_test1_start_req != 0U) {
    s_test1_start_req = 0U;
    BuzzerDrv_Beep(80U);
    s_q3_case1_cross_detect_active = 0U;
    s_q3_case1_cross_count = 0U;
    s_q3_case1_cross_latched = 0U;
    s_test1_dist_measure_active = 0U;
    s_test1_last_cross_valid = 0U;
    s_test1_last_cross_tick_ms = 0U;
    s_mode = 7U;
    s_state = 30U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
  } else if (s_test2_start_req != 0U) {
    s_test2_start_req = 0U;
    BuzzerDrv_Beep(80U);
    s_q3_case1_cross_detect_active = 0U;
    s_q3_case1_cross_count = 0U;
    s_q3_case1_cross_latched = 0U;
    s_mode = 8U;
    s_state = 40U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
  }

  switch (s_state) {
    case 0U: /* 等待 PD3(Q1) / PD4(Q2-1) / PD5(Q2-2) */
      return;

    case 1U: /* 启动动作：Q3-2 匀速前进1m，其余先前进0.08m */
      if (s_mode == 5U) {
        st = DflinkChassis_SendVelDisplacement(
            move_px_m21739, move_py_m21739, move_pz_m21739,
            move_speed_q3_case2_uniform_mps100, 200U);
        if (st != HAL_OK) {
          return;
        }
        s_wait_ticks = 0U;
        s_state = 15U;
        break;
      } else if (s_mode == 6U) {
        st = DflinkChassis_SendAdaptConstPMove(0, pre_move_y_m21739, 0,
                                               move_speed_mps100, 200U);
        if (st != HAL_OK) {
          return;
        }
        s_wait_ticks = 0U;
        s_state = 16U;
        break;
      }
      st = DflinkChassis_SendAdaptConstPMove(0, pre_move_y_m21739, 0,
                                             move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 2U;
      break;

    case 2U: /* 前置前进稳定等待约 0.3s */
      if (s_wait_ticks++ < 3U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 3U;
      break;

    case 3U: /* Q1/Q2-1/Q3-1左转90；Q2-2右转90 */
      st = DflinkChassis_SendRotation(
          0, 0, (s_mode == 3U) ? rot_right_90 : rot_left_90, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
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

    case 5U: /* 直走: Q3-1 第2段替换为 TEST1 整段直走流程 */
      if (s_mode == 4U && s_round == 1U) {
        s_wait_ticks = 0U;
        s_state = 50U;
        break;
      }
      s_q3_case1_cross_detect_active = 0U;
      s_q3_case1_cross_count = 0U;
      s_q3_case1_cross_latched = 0U;
      s_test1_dist_measure_active = 0U;
      s_test1_last_cross_valid = 0U;
      s_test1_last_cross_tick_ms = 0U;
      st = DflinkChassis_SendAdaptConstPMove(
          move_px_m21739,
          ((s_mode == 1U || s_mode == 4U) && s_round >= 3U) ? move_py_q1_last_m21739
                                                             : move_py_m21739,
          move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 6U;
      break;

    case 6U: /* 行进稳定等待：默认1.4s，Q3-1第2段匀速后延长到2.5s */
      if (s_wait_ticks++ <
          ((s_mode == 4U && s_round == 1U) ? wait_ticks_q3_case1_uniform
                                           : wait_ticks_default)) {
        return;
      }
      s_wait_ticks = 0U;
      s_round++;
      if (s_round >= ((s_mode == 1U || s_mode == 4U) ? 4U : 3U)) {
        if (s_mode == 1U || s_mode == 4U) {
          s_state = 7U;
        } else if (s_mode == 2U) {
          s_ret_seg = 0U;
          s_state = 7U;
        } else {
          s_ret_seg = 0U;
          s_state = 7U;
        }
      } else {
        s_state = 3U;
      }
      break;

    case 7U: /* Q2 调头；Q1/Q3-1 已完成停机 */
      if (s_mode == 1U || s_mode == 4U) {
        App_Motion_StopAndBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
        return;
      }
      if (s_mode != 2U && s_mode != 3U) {
        return;
      }
      st = DflinkChassis_SendRotation(0, 0, rot_u_turn, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 8U;
      break;

    case 8U: /* 调头稳定等待约 0.7s */
      if (s_wait_ticks++ < 7U) {
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
      s_wait_ticks = 0U;
      s_state = 10U;
      break;

    case 10U: /* 返程位移稳定等待约 1.4s */
      if (s_wait_ticks++ < wait_ticks_default) {
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
      s_wait_ticks = 0U;
      s_state = 14U;
      break;

    case 14U: /* 右转稳定等待约 0.3s */
      if (s_wait_ticks++ < 3U) {
        return;
      }
      s_wait_ticks = 0U;
      s_ret_seg++;
      s_state = 9U;
      break;

    case 12U: /* Q2 完成（含原路返回） */
      App_Motion_StopAndBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
      break;

    case 15U: /* Q3-2 匀速前进1m后等待约1.2s结束 */
      if (s_wait_ticks++ < 12U) {
        return;
      }
      App_Motion_StopAndBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
      break;

    case 16U: /* Q4 前置前进稳定等待约 0.5s */
      if (s_wait_ticks++ < 5U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 17U;
      break;

    case 17U: /* Q4 左转 90 度 */
      st = DflinkChassis_SendRotation(0, 0, rot_left_90, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 18U;
      break;

    case 18U: /* Q4 转向稳定等待约 0.5s */
      if (s_wait_ticks++ < 5U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 19U;
      break;

    case 19U: /* Q4 前进 0.55m */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_q4_first_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 20U;
      break;

    case 20U: /* Q4 等待 4.0s */
      if (s_wait_ticks++ < 40U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 21U;
      break;

    case 21U: /* Q4 再前进 0.45m */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_q4_second_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 22U;
      break;

    case 22U: /* Q4 第二段 0.5m 稳定等待约 2.2s */
      if (s_wait_ticks++ < 22U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 23U;
      break;

    case 23U: /* Q4 再左转 90 度 */
      st = DflinkChassis_SendRotation(0, 0, rot_left_90, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
      }
      if (s_stepper_motor_inited != 0U) {
        s_stepper_motor.pos_mode = STEPPER_POS_RELATIVE;
        s_stepper_motor.speed = stepper_speed_turn;
        s_stepper_motor.pulse = stepper_pulse_90deg; /* 步进电机右转90度 */
        (void)StepperMotorDrv_RunPosition(&s_stepper_motor);
      }
      s_wait_ticks = 0U;
      s_state = 24U;
      break;

    case 24U: /* Q4 再次转向稳定等待约 0.5s */
      if (s_wait_ticks++ < 5U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 25U;
      break;

    case 25U: /* Q4 最后前进 0.92m */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_q1_last_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 26U;
      break;

    case 26U: /* Q4 最后一段稳定等待约 1.8s 后停车 */
      if (s_wait_ticks++ < 18U) {
        return;
      }
      App_Motion_StopAndBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
      break;

    case 30U: /* TEST1: 匀速前进 0.2m */
      st = DflinkChassis_SendVelDisplacement(move_px_m21739, move_py_test1_02_m21739,
                                             move_pz_m21739, move_speed_q3_case2_uniform_mps100,
                                             200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 31U;
      break;

    case 31U: /* TEST1: 第一段匀速稳定等待 */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 32U;
      break;

    case 32U: /* TEST1: 蜂鸣器触发一次 */
      BuzzerDrv_Beep(80U);
      s_state = 33U;
      break;

    case 33U: /* TEST1: 匀速前进 0.6m，开启路口检测 */
      s_q3_case1_cross_detect_active = 1U;
      s_q3_case1_cross_count = 0U;
      s_q3_case1_cross_latched = 0U;
      s_test1_dist_measure_active = 1U;
      s_test1_last_cross_valid = 0U;
      s_test1_last_cross_tick_ms = 0U;
      st = DflinkChassis_SendVelDisplacement(move_px_m21739, move_py_test1_06_m21739,
                                             move_pz_m21739, move_speed_q3_case2_uniform_mps100,
                                             200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 34U;
      break;

    case 34U: /* TEST1: 第二段匀速稳定等待（含路口检测） */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 35U;
      break;

    case 35U: /* TEST1: 蜂鸣器连续触发 - 第1次 */
      BuzzerDrv_Beep(80U);
      s_wait_ticks = 0U;
      s_state = 36U;
      break;

    case 36U: /* TEST1: 蜂鸣器连续触发 - 第2次 */
      if (s_wait_ticks++ < 2U) {
        return;
      }
      BuzzerDrv_Beep(80U);
      s_wait_ticks = 0U;
      s_state = 37U;
      break;

    case 37U: /* TEST1: 自适应前进 0.2m */
      s_q3_case1_cross_detect_active = 0U;
      s_q3_case1_cross_count = 0U;
      s_q3_case1_cross_latched = 0U;
      s_test1_dist_measure_active = 0U;
      s_test1_last_cross_valid = 0U;
      s_test1_last_cross_tick_ms = 0U;
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_test1_02_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 38U;
      break;

    case 38U: /* TEST1: 末段稳定等待后停车 */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      App_Motion_StopAndBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
      break;

    case 40U: /* TEST2: 匀速向前 1.0m，速度10 */
      st = DflinkChassis_SendVelDisplacement(move_px_m21739, move_py_m21739, move_pz_m21739,
                                             move_speed_q3_case2_uniform_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 41U;
      break;

    case 41U: /* TEST2: 稳定等待后停车 */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      App_Motion_StopAndBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
      break;

    case 50U: /* Q3-1 第2段替换流程: 匀速前进 0.2m */
      st = DflinkChassis_SendVelDisplacement(move_px_m21739, move_py_test1_02_m21739,
                                             move_pz_m21739, move_speed_q3_case2_uniform_mps100,
                                             200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 51U;
      break;

    case 51U: /* Q3-1 第2段替换流程: 第一段匀速稳定等待 */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 52U;
      break;

    case 52U: /* Q3-1 第2段替换流程: 蜂鸣一次 */
      BuzzerDrv_Beep(80U);
      s_state = 53U;
      break;

    case 53U: /* Q3-1 第2段替换流程: 匀速前进 0.6m，开启路口检测 */
      s_q3_case1_cross_detect_active = 1U;
      s_q3_case1_cross_count = 0U;
      s_q3_case1_cross_latched = 0U;
      s_test1_dist_measure_active = 1U;
      s_test1_last_cross_valid = 0U;
      s_test1_last_cross_tick_ms = 0U;
      st = DflinkChassis_SendVelDisplacement(move_px_m21739, move_py_test1_06_m21739,
                                             move_pz_m21739, move_speed_q3_case2_uniform_mps100,
                                             200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 54U;
      break;

    case 54U: /* Q3-1 第2段替换流程: 第二段匀速稳定等待（含路口检测） */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 55U;
      break;

    case 55U: /* Q3-1 第2段替换流程: 双蜂鸣第1次 */
      BuzzerDrv_Beep(80U);
      s_wait_ticks = 0U;
      s_state = 56U;
      break;

    case 56U: /* Q3-1 第2段替换流程: 双蜂鸣第2次 */
      if (s_wait_ticks++ < 2U) {
        return;
      }
      BuzzerDrv_Beep(80U);
      s_wait_ticks = 0U;
      s_state = 57U;
      break;

    case 57U: /* Q3-1 第2段替换流程: 自适应前进 0.2m */
      s_q3_case1_cross_detect_active = 0U;
      s_q3_case1_cross_count = 0U;
      s_q3_case1_cross_latched = 0U;
      s_test1_dist_measure_active = 0U;
      s_test1_last_cross_valid = 0U;
      s_test1_last_cross_tick_ms = 0U;
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_test1_02_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 58U;
      break;

    case 58U: /* Q3-1 第2段替换流程结束，按原Q3逻辑推进轮次 */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      s_wait_ticks = 0U;
      s_round++;
      if (s_round >= 4U) {
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
  StepperMotorDrv_Init(&s_stepper_motor, &huart2, 0x01U);
  s_stepper_motor_inited = 1U;
  GwGraySensor_InitDefaults(&s_gray_sensor, &hi2c1);
  s_gray_sensor_inited = GwGraySensor_InitPingWait(&s_gray_sensor, 300U) ? 1U : 0U;
  s_q3_case1_cross_detect_active = 0U;
  s_q3_case1_cross_count = 0U;
  s_q3_case1_cross_latched = 0U;
  s_test1_dist_measure_active = 0U;
  s_test1_last_cross_valid = 0U;
  s_test1_last_cross_tick_ms = 0U;

  (void)Scheduler_AddTask(App_Task_1ms,   1U);
  (void)Scheduler_AddTask(App_Task_10ms, 10U);
  (void)Scheduler_AddTask(App_Task_50ms, 50U);
  (void)Scheduler_AddTask(App_Task_100ms, 100U);
}
