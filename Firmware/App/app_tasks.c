#include "app_tasks.h"
#include "scheduler.h"

#include "buzzer_drv.h"
#include "button_drv.h"
#include "chassis_uart_bridge.h"
#include "debug_uart.h"
#include "dflink_chassis_motion.h"
#include "fw_config.h"
#include "gw_gray_sensor.h"
#include "i2c.h"
#include "stepper_motor_drv.h"
#include "usart.h"

static uint8_t s_q1_start_req;
static uint8_t s_q2_case1_start_req;
static uint8_t s_q2_case2_start_req;
static uint8_t s_q3_case1_start_req;
static uint8_t s_q3_case2_start_req;
static uint8_t s_q4_start_req;
static uint8_t s_q4_case1_start_req;
static uint8_t s_test1_start_req;
static uint8_t s_test1_rec_start_req;
static uint8_t s_test2_start_req;
static uint8_t s_test3_case1_start_req;
static uint8_t s_test3_case2_start_req;
static uint8_t s_test3_case3_start_req;
static uint8_t s_test3_case4_start_req;
static uint8_t s_test4_start_req;
static uint8_t s_test5_start_req;
static uint8_t s_test6_start_req;
static uint8_t s_test7_active;
static uint8_t s_test7_prev_raw_cross;
static uint8_t s_q4v_uart6_wait_resp;
static uint8_t s_q4v_uart6_resp_ready;
static uint8_t s_q4v_uart6_resp_code;
static uint8_t s_pending_beeps;
static uint8_t s_pd3_q4v_phase;   /* 0=未发 handshake，1=已发 AA CC DD，待第二次按键启动 Q4-V */
static uint8_t s_pd3_bb_beep_done; /* 本周期内已对 AA CC BB 蜂鸣过一次 */
static StepperMotorDrv s_stepper_motor;
static uint8_t s_stepper_motor_inited;
static uint8_t s_stepper_motor_enabled;
static uint8_t s_heading_lock_enabled;
static uint8_t s_heading_lock_init_done;
static GwGraySensor s_gray_sensor;
static uint8_t s_gray_sensor_inited;
static uint8_t s_q3_case1_cross_detect_active;
static uint8_t s_q3_case1_cross_count;
static uint8_t s_q3_case1_cross_latched;
static uint8_t s_test1_dist_measure_active;
static uint8_t s_test1_last_cross_valid;
static uint32_t s_test1_last_cross_tick_ms;
static uint8_t s_cross_raw_on_cnt;
static uint8_t s_cross_raw_off_cnt;
static uint8_t s_cross_confirmed;
static int32_t s_measure_corr_rz10000;
#define APP_MEAS_LOG_MAX 16U
#define APP_MEAS_EXPECT_CROSS_COUNT 3U
#define APP_CROSS_ON_CONFIRM_FRAMES   4U /* 原为 7；越低越多误触发风险 */
#define APP_CROSS_OFF_CONFIRM_FRAMES  3U /* 原为 4 */
#define APP_DIST_CM_X10_MIN 10U  /* 1.0cm */
#define APP_DIST_CM_X10_MAX 50U  /* 5.0cm */
static uint8_t s_meas_log_cross_idx[APP_MEAS_LOG_MAX];
static uint32_t s_meas_log_dist_x10[APP_MEAS_LOG_MAX];
static uint8_t s_meas_log_has_dist[APP_MEAS_LOG_MAX];
static uint8_t s_meas_log_count;
/* TEST1-REC：检测窗内仅采样 digital_inv + tick，段结束后离线debounce 与 TEST1 相同再算 l1/l2 */
#define APP_TEST1_REC_MAX 320U
static uint8_t s_test1_rec_buf_inv[APP_TEST1_REC_MAX];
static uint32_t s_test1_rec_buf_tick[APP_TEST1_REC_MAX];
static uint16_t s_test1_rec_count;
static uint8_t s_test1_rec_active;
#define APP_GRAY_CROSS_MIN_ACTIVE_BITS 3U
#define APP_GRAY_WATCHDOG_FAIL_TICKS 5U /* 连续 Ping 失败次数后认为灰度离线 */

static void App_SendBtGrayOffline(void)
{
  static const uint8_t k_gray_off[] =
      {'\r', '\n', 'G', 'R', 'A', 'Y', ':', 'O', 'F', 'F', '\r', '\n'};
  (void)HAL_UART_Transmit(&huart4, (uint8_t *)k_gray_off,
                          (uint16_t)(sizeof(k_gray_off) / sizeof(k_gray_off[0])), 50U);
}

static void App_GraySensorWatchdog10Hz(void)
{
  static uint8_t s_gray_ping_fail_cnt;
  uint8_t was_online;

  was_online = s_gray_sensor_inited;

  if (GwGray_Ping(&s_gray_sensor)) {
    s_gray_ping_fail_cnt = 0U;
    s_gray_sensor_inited = 1U;
  } else {
    if (s_gray_ping_fail_cnt < 255U) {
      s_gray_ping_fail_cnt++;
    }
    if (s_gray_ping_fail_cnt >= APP_GRAY_WATCHDOG_FAIL_TICKS) {
      s_gray_sensor_inited = 0U;
      if (was_online != 0U) {
        App_SendBtGrayOffline();
      }
    }
  }
}

#define APP_TEST6_REPORT_INTERVAL_TICKS 10U /* 1 Hz @ App_Task_100ms */
#define APP_TEST6_REPORT_SAMPLES 120U        /* 共 120 秒采样便于拔插灰度观测 */

static uint8_t s_test6_active;
static uint8_t s_test6_tick_div;
static uint16_t s_test6_samples_remain;
static uint8_t s_test6_boot_guard_init;
static uint32_t s_test6_boot_deadline_ms;

static uint8_t App_Test6_AppendU32Dec(uint8_t *msg, uint8_t n, uint32_t v)
{
  uint8_t digits[10];
  uint8_t dlen;
  uint8_t i;

  if (v == 0U) {
    msg[n++] = '0';
    return n;
  }
  dlen = 0U;
  while (v > 0U && dlen < (uint8_t)sizeof(digits)) {
    digits[dlen++] = (uint8_t)(v % 10U);
    v /= 10U;
  }
  for (i = 0U; i < dlen; i++) {
    msg[n++] = (uint8_t)('0' + digits[dlen - 1U - i]);
  }
  return n;
}

static uint8_t App_Test6_AppendHexU8(uint8_t *msg, uint8_t n, uint8_t v)
{
  static const char k_hex[] = "0123456789ABCDEF";

  msg[n++] = (uint8_t)k_hex[(v >> 4) & 0x0FU];
  msg[n++] = (uint8_t)k_hex[v & 0x0FU];
  return n;
}

static void App_Test6EmitSample(void)
{
  uint8_t msg[96];
  uint8_t n;
  uint32_t tick_ms;
  uint32_t i2c_err0;
  uint32_t i2c_state_u;
  bool ping_ok;
  bool dig_ok;
  uint8_t inited_shadow;
  uint32_t i2c_err1;
  uint8_t di;

  tick_ms = HAL_GetTick();
  i2c_err0 = HAL_I2C_GetError(&hi2c1);
  i2c_state_u = (uint32_t)hi2c1.State;
  inited_shadow = s_gray_sensor_inited;
  ping_ok = GwGray_Ping(&s_gray_sensor);
  dig_ok = GwGray_ReadDigitalUpdate(&s_gray_sensor);
  i2c_err1 = HAL_I2C_GetError(&hi2c1);
  di = s_gray_sensor.digital_inv;

  n = 0U;
  msg[n++] = '\r';
  msg[n++] = '\n';
  msg[n++] = 'T';
  msg[n++] = '6';
  msg[n++] = ' ';
  msg[n++] = 'm';
  msg[n++] = 's';
  msg[n++] = '=';
  n = App_Test6_AppendU32Dec(msg, n, tick_ms);
  msg[n++] = ' ';
  msg[n++] = 'i';
  msg[n++] = 'n';
  msg[n++] = 'i';
  msg[n++] = 't';
  msg[n++] = '=';
  msg[n++] = (uint8_t)('0' + (uint8_t)(inited_shadow != 0U ? 1 : 0));
  msg[n++] = ' ';
  msg[n++] = 'p';
  msg[n++] = 'n';
  msg[n++] = 'g';
  msg[n++] = '=';
  msg[n++] = (uint8_t)('0' + (uint8_t)(ping_ok ? 1 : 0));
  msg[n++] = ' ';
  msg[n++] = 'd';
  msg[n++] = 'g';
  msg[n++] = '=';
  msg[n++] = (uint8_t)('0' + (uint8_t)(dig_ok ? 1 : 0));
  msg[n++] = ' ';
  msg[n++] = 's';
  msg[n++] = 't';
  msg[n++] = '=';
  n = App_Test6_AppendHexU8(msg, n, (uint8_t)(i2c_state_u & 0xFFU));
  msg[n++] = ' ';
  msg[n++] = 'e';
  msg[n++] = '0';
  msg[n++] = '=';
  n = App_Test6_AppendHexU8(msg, n, (uint8_t)(i2c_err0 & 0xFFU));
  msg[n++] = ' ';
  msg[n++] = 'e';
  msg[n++] = '1';
  msg[n++] = '=';
  n = App_Test6_AppendHexU8(msg, n, (uint8_t)(i2c_err1 & 0xFFU));
  msg[n++] = ' ';
  msg[n++] = 'd';
  msg[n++] = 'i';
  msg[n++] = '=';
  n = App_Test6_AppendHexU8(msg, n, di);
  msg[n++] = '\r';
  msg[n++] = '\n';
  (void)HAL_UART_Transmit(&huart4, msg, n, 50U);
}

static void App_Test6_Process100ms(uint32_t now_ms)
{
  static const char k_banner[] =
      "\r\nTEST6:start 120s@1Hz UART4 init=inited-flag png dg "
      "st=i2cs e0=HALerr0 e1=HALerr1 di=digital_inv\r\n";

  if (s_test6_boot_guard_init == 0U) {
    s_test6_boot_guard_init = 1U;
    s_test6_boot_deadline_ms = now_ms + 1200U;
  }
  if ((int32_t)(now_ms - s_test6_boot_deadline_ms) < 0) {
    s_test6_start_req = 0U;
    s_test6_active = 0U;
    return;
  }

  if (s_test6_start_req != 0U) {
    s_test6_start_req = 0U;
    s_test6_active = 1U;
    s_test6_tick_div = 0U;
    s_test6_samples_remain = APP_TEST6_REPORT_SAMPLES;
    BuzzerDrv_Beep(80U);
    (void)HAL_UART_Transmit(&huart4, (uint8_t *)k_banner,
                            (uint16_t)(sizeof(k_banner) - 1U), 50U);
    App_Test6EmitSample();
    if (s_test6_samples_remain > 0U) {
      s_test6_samples_remain--;
    }
    if (s_test6_samples_remain == 0U) {
      s_test6_active = 0U;
    }
    return;
  }

  if (s_test6_active == 0U) {
    return;
  }

  if (s_test6_tick_div + 1U < APP_TEST6_REPORT_INTERVAL_TICKS) {
    s_test6_tick_div++;
    return;
  }
  s_test6_tick_div = 0U;
  App_Test6EmitSample();
  if (s_test6_samples_remain > 0U) {
    s_test6_samples_remain--;
  }
  if (s_test6_samples_remain == 0U) {
    s_test6_active = 0U;
  }
}

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
  s_cross_raw_on_cnt = 0U;
  s_cross_raw_off_cnt = 0U;
  s_cross_confirmed = 0U;
  s_measure_corr_rz10000 = 0;
  s_meas_log_count = 0U;
  s_test1_rec_active = 0U;
  s_q4v_uart6_wait_resp = 0U;
  s_q4v_uart6_resp_ready = 0U;
  s_q4v_uart6_resp_code = 0U;
  *mode = 0U;
  *state = 0U;
  *wait_ticks = 0U;
  *round = 0U;
  *ret_seg = 0U;
}

static void App_Motion_StopNoBeep(uint8_t *mode, uint8_t *state, uint8_t *wait_ticks,
                                  uint8_t *round, uint8_t *ret_seg)
{
  s_q3_case1_cross_detect_active = 0U;
  s_q3_case1_cross_count = 0U;
  s_q3_case1_cross_latched = 0U;
  s_test1_dist_measure_active = 0U;
  s_test1_last_cross_valid = 0U;
  s_test1_last_cross_tick_ms = 0U;
  s_cross_raw_on_cnt = 0U;
  s_cross_raw_off_cnt = 0U;
  s_cross_confirmed = 0U;
  s_measure_corr_rz10000 = 0;
  s_meas_log_count = 0U;
  s_test1_rec_active = 0U;
  s_q4v_uart6_wait_resp = 0U;
  s_q4v_uart6_resp_ready = 0U;
  s_q4v_uart6_resp_code = 0U;
  *mode = 0U;
  *state = 0U;
  *wait_ticks = 0U;
  *round = 0U;
  *ret_seg = 0U;
}

static uint8_t App_AppendCm1ToMsg(uint8_t *msg, uint8_t n, uint32_t dist_cm_x10)
{
  uint8_t digits[10];
  uint8_t dlen;
  uint32_t v;
  uint8_t frac;
  uint8_t i;

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
  return n;
}

static void App_SendBtMeasureL12(uint32_t l1_cm_x10, uint32_t l2_cm_x10)
{
  uint8_t msg[48];
  uint8_t n;

  /* 蓝牙发送格式：
   *
   * \r\n
   * l1 = 数据1cm\r\n
   * l2 = 数据2cm\r\n
   */
  n = 0U;
  msg[n++] = '\r';
  msg[n++] = '\n';
  msg[n++] = 'l';
  msg[n++] = '1';
  msg[n++] = ' ';
  msg[n++] = '=';
  msg[n++] = ' ';
  n = App_AppendCm1ToMsg(msg, n, l1_cm_x10);
  msg[n++] = '\r';
  msg[n++] = '\n';
  msg[n++] = 'l';
  msg[n++] = '2';
  msg[n++] = ' ';
  msg[n++] = '=';
  msg[n++] = ' ';
  n = App_AppendCm1ToMsg(msg, n, l2_cm_x10);
  msg[n++] = '\r';
  msg[n++] = '\n';

  (void)HAL_UART_Transmit(&huart4, msg, n, 50U);
}

static void App_FlushMeasureLogs(void)
{
  uint8_t i;
  uint32_t l1_cm_x10;
  uint32_t l2_cm_x10;
  uint8_t dist_idx;

  l1_cm_x10 = 25U; /* 缺失项默认发送 2.5cm */
  l2_cm_x10 = 25U; /* 缺失项默认发送 2.5cm */
  dist_idx = 0U;

  for (i = 0U; i < s_meas_log_count; i++) {
    if (s_meas_log_has_dist[i] != 0U) {
      if (dist_idx == 0U) {
        l1_cm_x10 = s_meas_log_dist_x10[i];
        dist_idx = 1U;
      } else if (dist_idx == 1U) {
        l2_cm_x10 = s_meas_log_dist_x10[i];
        dist_idx = 2U;
      }
    }
  }

  App_SendBtMeasureL12(l1_cm_x10, l2_cm_x10);
  s_meas_log_count = 0U;
}

static void App_ClearMeasureLogs(void)
{
  s_meas_log_count = 0U;
}

static void App_ProcessTest1RecSample10ms(void)
{
  if (s_test1_rec_active == 0U || s_gray_sensor_inited == 0U) {
    return;
  }
  if (s_test1_rec_count >= APP_TEST1_REC_MAX) {
    return;
  }
  if (!GwGray_ReadDigitalUpdate(&s_gray_sensor)) {
    return;
  }
  s_test1_rec_buf_inv[s_test1_rec_count] = s_gray_sensor.digital_inv;
  s_test1_rec_buf_tick[s_test1_rec_count] = HAL_GetTick();
  s_test1_rec_count++;
}

/* 段结束后对缓冲跑一次与 App_ProcessCrossDetect10ms 等价的 debounce，再填 s_meas_log_* */
static void App_AnalyzeTest1RecBuffer(void)
{
  uint16_t i;
  uint8_t on_cnt;
  uint8_t off_cnt;
  uint8_t confirmed;
  uint8_t latched;
  uint8_t cross_n;
  uint32_t ticks[APP_MEAS_EXPECT_CROSS_COUNT];
  bool raw_cross;

  on_cnt = 0U;
  off_cnt = 0U;
  confirmed = 0U;
  latched = 0U;
  cross_n = 0U;

  for (i = 0U; i < s_test1_rec_count; i++) {
    raw_cross = GwGray_IsCrossByDigitalInv(s_test1_rec_buf_inv[i], APP_GRAY_CROSS_MIN_ACTIVE_BITS);
    if (raw_cross) {
      if (on_cnt < 255U) {
        on_cnt++;
      }
      off_cnt = 0U;
      if (confirmed == 0U && on_cnt >= APP_CROSS_ON_CONFIRM_FRAMES) {
        confirmed = 1U;
      }
    } else {
      if (off_cnt < 255U) {
        off_cnt++;
      }
      on_cnt = 0U;
      if (confirmed != 0U && off_cnt >= APP_CROSS_OFF_CONFIRM_FRAMES) {
        confirmed = 0U;
      }
    }

    if (confirmed != 0U) {
      if (latched == 0U) {
        if (cross_n < APP_MEAS_EXPECT_CROSS_COUNT) {
          ticks[cross_n] = s_test1_rec_buf_tick[i];
          cross_n++;
        }
        latched = 1U;
        if (cross_n >= APP_MEAS_EXPECT_CROSS_COUNT) {
          break;
        }
      }
    } else {
      latched = 0U;
    }
  }

  App_ClearMeasureLogs();
  for (i = 0U; i < cross_n; i++) {
    uint8_t slot;
    uint32_t dt_ms;
    uint32_t dist_cm_x10;

    slot = s_meas_log_count;
    if (slot >= APP_MEAS_LOG_MAX) {
      break;
    }
    s_meas_log_cross_idx[slot] = (uint8_t)(i + 1U);
    s_meas_log_has_dist[slot] = 0U;
    if (i >= 1U) {
      dt_ms = ticks[i] - ticks[i - 1U];
      dist_cm_x10 = (dt_ms * 1197U + 5000U) / 10000U;
      if (dist_cm_x10 < APP_DIST_CM_X10_MIN) {
        dist_cm_x10 = APP_DIST_CM_X10_MIN;
      } else if (dist_cm_x10 > APP_DIST_CM_X10_MAX) {
        dist_cm_x10 = APP_DIST_CM_X10_MAX;
      }
      s_meas_log_dist_x10[slot] = dist_cm_x10;
      s_meas_log_has_dist[slot] = 1U;
    }
    s_meas_log_count++;
  }
}

/* VOFA+ FireWater：prefix:ch0,ch1,... 每行 \r\n 结尾（数值通道便于上位机绘图） */
static uint8_t App_MsgAppendDecU32(uint8_t *msg, uint8_t n, uint32_t v)
{
  uint8_t d[10];
  uint8_t k;
  uint8_t j;

  if (v == 0U) {
    msg[n++] = '0';
    return n;
  }
  k = 0U;
  while (v > 0U && k < (uint8_t)sizeof(d)) {
    d[k++] = (uint8_t)('0' + (v % 10U));
    v /= 10U;
  }
  for (j = k; j > 0U; j--) {
    msg[n++] = d[j - 1U];
  }
  return n;
}

static void App_Test1Rec_SendFireWaterUsart1(void)
{
  uint8_t line[56];
  uint8_t n;
  uint16_t i;
  uint16_t j;
  static const uint8_t k_meta_hdr[] = {'t','1','r','_','m',':'};
  static const uint8_t k_row_hdr[] = {'t','1','r',':'};

  n = 0U;
  for (j = 0U; j < (uint16_t)(sizeof(k_meta_hdr) / sizeof(k_meta_hdr[0])); j++) {
    line[n++] = k_meta_hdr[j];
  }
  n = App_MsgAppendDecU32(line, n, (uint32_t)s_test1_rec_count);
  line[n++] = '\r';
  line[n++] = '\n';
  (void)DebugUart_Send(line, n, 50U);

  for (i = 0U; i < s_test1_rec_count; i++) {
    uint32_t tick;
    uint8_t inv;

    n = 0U;
    for (j = 0U; j < (uint16_t)(sizeof(k_row_hdr) / sizeof(k_row_hdr[0])); j++) {
      line[n++] = k_row_hdr[j];
    }
    n = App_MsgAppendDecU32(line, n, (uint32_t)i);
    line[n++] = ',';
    tick = s_test1_rec_buf_tick[i];
    n = App_MsgAppendDecU32(line, n, tick);
    line[n++] = ',';
    inv = s_test1_rec_buf_inv[i];
    n = App_MsgAppendDecU32(line, n, (uint32_t)inv);
    line[n++] = '\r';
    line[n++] = '\n';
    (void)DebugUart_Send(line, n, 50U);
  }
}

/* TEST1-GYCK：Ping + 读数字量；成功蜂鸣 1 次，失败队列蜂鸣 2 次（PD4 或 AA BB DB） */
static void App_Test1Gyck_CheckAndBeep(void)
{
  bool ok;

  ok = GwGray_Ping(&s_gray_sensor);
  if (ok != false) {
    ok = GwGray_ReadDigitalUpdate(&s_gray_sensor);
  }
  if (ok != false) {
    BuzzerDrv_Beep(80U);
  } else {
    s_pending_beeps = 2U;
  }
}

static void App_Test7_UserToggle(void)
{
  s_test7_active ^= 1U;
  s_test7_prev_raw_cross = 0U;
  if (s_test7_active != 0U) {
    BuzzerDrv_Beep(80U);
  }
}

static void App_Test7_Process10ms(void)
{
  bool is_cross;
  bool read_ok;

  if (s_test7_active == 0U) {
    return;
  }

  if (s_gray_sensor_inited == 0U) {
    return;
  }

  read_ok = GwGray_ReadAndDetectCross(&s_gray_sensor, APP_GRAY_CROSS_MIN_ACTIVE_BITS, &is_cross);
  if (!read_ok) {
    return;
  }

  if (is_cross && s_test7_prev_raw_cross == 0U) {
    BuzzerDrv_Beep(80U);
  }
  s_test7_prev_raw_cross = (is_cross != false) ? 1U : 0U;
}

static void App_ProcessCrossDetect10ms(void)
{
  bool is_cross;
  bool raw_cross;
  bool read_ok;

  if (s_q3_case1_cross_detect_active == 0U || s_gray_sensor_inited == 0U) {
    return;
  }

  read_ok = GwGray_ReadAndDetectCross(&s_gray_sensor, APP_GRAY_CROSS_MIN_ACTIVE_BITS, &is_cross);
  if (!read_ok) {
    return;
  }

  raw_cross = (is_cross != 0U);
  if (raw_cross) {
    if (s_cross_raw_on_cnt < 255U) {
      s_cross_raw_on_cnt++;
    }
    s_cross_raw_off_cnt = 0U;
    if (s_cross_confirmed == 0U && s_cross_raw_on_cnt >= APP_CROSS_ON_CONFIRM_FRAMES) {
      s_cross_confirmed = 1U;
    }
  } else {
    if (s_cross_raw_off_cnt < 255U) {
      s_cross_raw_off_cnt++;
    }
    s_cross_raw_on_cnt = 0U;
    if (s_cross_confirmed != 0U && s_cross_raw_off_cnt >= APP_CROSS_OFF_CONFIRM_FRAMES) {
      s_cross_confirmed = 0U;
    }
  }

  if (s_cross_confirmed != 0U) {
    if (s_q3_case1_cross_latched == 0U) {
      uint32_t now_ms;
      uint8_t slot;
      BuzzerDrv_Beep(80U);
      s_q3_case1_cross_count++;
      slot = s_meas_log_count;
      if (slot < APP_MEAS_LOG_MAX) {
        s_meas_log_cross_idx[slot] = s_q3_case1_cross_count;
        s_meas_log_has_dist[slot] = 0U;
      }
      if (s_test1_dist_measure_active != 0U) {
        now_ms = HAL_GetTick();
        if (s_test1_last_cross_valid != 0U) {
          uint32_t dt_ms;
          uint32_t dist_cm_x10;
          dt_ms = now_ms - s_test1_last_cross_tick_ms;
          /* 距离(cm,保留1位) = 时间(ms) * 0.15(m/s) * 0.798 * 100 * 10 / 1000
           * = 时间(ms) * 119.7 / 1000
           */
          dist_cm_x10 = (dt_ms * 1197U + 5000U) / 10000U;
          if (dist_cm_x10 < APP_DIST_CM_X10_MIN) {
            dist_cm_x10 = APP_DIST_CM_X10_MIN;
          } else if (dist_cm_x10 > APP_DIST_CM_X10_MAX) {
            dist_cm_x10 = APP_DIST_CM_X10_MAX;
          }
          if (slot < APP_MEAS_LOG_MAX) {
            s_meas_log_dist_x10[slot] = dist_cm_x10;
            s_meas_log_has_dist[slot] = 1U;
          }
        }
        s_test1_last_cross_tick_ms = now_ms;
        s_test1_last_cross_valid = 1U;
      }
      if (slot < APP_MEAS_LOG_MAX) {
        s_meas_log_count++;
      }
      if (s_q3_case1_cross_count >= APP_MEAS_EXPECT_CROSS_COUNT) {
        /* 每次测距固定 3 个路口：达到后停止后续检测，避免多余触发干扰。 */
        s_q3_case1_cross_detect_active = 0U;
        s_test1_dist_measure_active = 0U;
      }
      s_q3_case1_cross_latched = 1U;
    }
  } else {
    s_q3_case1_cross_latched = 0U;
  }
}

static int32_t App_CalcGrayCorrRz10000(void)
{
  uint8_t bits;
  uint8_t i;
  uint8_t cnt;
  uint16_t sum_idx10;
  int16_t center_idx10;
  int16_t err_idx10;
  uint16_t mag;
  int32_t deg10000;
  const int16_t idx_center10 = 35;      /* 8路中心(3.5*10) */
  const uint16_t deadband_idx10 = 5U;   /* 小偏差不修正 */
  const int32_t deg_min10000 = 20000;   /* 2.0deg */
  const int32_t deg_max10000 = 100000;  /* 10.0deg */

  if (s_gray_sensor_inited == 0U) {
    return 0;
  }
  if (!GwGray_ReadDigitalUpdate(&s_gray_sensor)) {
    return 0;
  }

  bits = s_gray_sensor.digital_inv; /* bit=1 表示该路检测到黑线 */
  if (bits == 0U) {
    return 0;
  }

  cnt = 0U;
  sum_idx10 = 0U;
  for (i = 0U; i < 8U; i++) {
    if (((bits >> i) & 0x01U) != 0U) {
      cnt++;
      sum_idx10 = (uint16_t)(sum_idx10 + (uint16_t)(i * 10U));
    }
  }
  if (cnt == 0U) {
    return 0;
  }

  center_idx10 = (int16_t)(sum_idx10 / cnt);
  err_idx10 = (int16_t)(center_idx10 - idx_center10); /* >0 黑线在右侧，<0 在左侧 */
  mag = (uint16_t)((err_idx10 >= 0) ? err_idx10 : -err_idx10);
  if (mag <= deadband_idx10) {
    return 0;
  }

  /* 按偏移量线性映射到 2~10 度 */
  deg10000 = deg_min10000 + ((int32_t)mag * (deg_max10000 - deg_min10000)) / 35;
  if (deg10000 > deg_max10000) {
    deg10000 = deg_max10000;
  }
  if (err_idx10 < 0) {
    deg10000 = -deg10000; /* 黑线在左侧 -> 向左小角度 */
  } /* 黑线在右侧 -> 向右小角度（正） */

  return deg10000;
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
  static uint8_t s_uart6_rx_prev1;
  static uint8_t s_uart6_rx_prev2;
  static uint8_t s_uart6_link_wait_resp;
  static uint8_t s_uart6_test5_active;
  static uint32_t s_uart6_link_deadline_ms;
  static uint32_t s_uart6_test5_deadline_ms;
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
    s_test1_rec_start_req = 0U;
    s_test2_start_req = 0U;
    s_test5_start_req = 0U;
    s_test6_start_req = 0U;
    s_test7_active = 0U;
    s_test7_prev_raw_cross = 0U;
    s_pd3_q4v_phase = 0U;
    s_pd3_bb_beep_done = 0U;
    s_heading_lock_init_done = 0U;
  } else if (ButtonDrv_WasPressed(BUTTON_DRV_PD3)) {
    static const uint8_t k_uart6_aa_cc_dd[3] = {0xAAU, 0xCCU, 0xDDU};
    if (s_pd3_q4v_phase == 0U) {
      if (DebugUart6_Send(k_uart6_aa_cc_dd, 3U, 50U) == HAL_OK) {
        s_pd3_q4v_phase = 1U;
        s_pd3_bb_beep_done = 0U;
      }
    } else {
      s_q4_start_req = 1U; /* Q4-V，与同任务串口 AA BB 0B 等价 */
      s_pd3_q4v_phase = 0U;
    }
  } else if (ButtonDrv_WasPressed(BUTTON_DRV_PD4)) {
    App_Test1Gyck_CheckAndBeep();
  } else if (ButtonDrv_WasPressed(BUTTON_DRV_PD5)) {
    HAL_StatusTypeDef st_lock;
    uint8_t next_lock;
    next_lock = (s_heading_lock_enabled == 0U) ? 1U : 0U;
    st_lock = DflinkChassis_SetHeadingLock(next_lock, 200U);
    if (st_lock == HAL_OK) {
      s_heading_lock_enabled = next_lock;
      BuzzerDrv_Beep(80U);
    }
  }

  if (s_heading_lock_init_done == 0U &&
      (int32_t)(now_ms - s_start_guard_deadline_ms) >= 0) {
    if (DflinkChassis_SetHeadingLock(1U, 200U) == HAL_OK) {
      s_heading_lock_enabled = 1U;
      s_heading_lock_init_done = 1U;
    }
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

  App_ProcessTest1RecSample10ms();
  App_ProcessCrossDetect10ms();

  App_Test7_Process10ms();

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
          s_q4_case1_start_req = 1U; /* Q4-1 */
        } else if (rx == 0xD0U) {
          s_test1_start_req = 1U;
        } else if (rx == 0xDAU) {
          s_test1_rec_start_req = 1U; /* TEST1-REC：先采样后离线算距 */
        } else if (rx == 0xDBU) {
          App_Test1Gyck_CheckAndBeep(); /* TEST1-GYCK：灰度可读性，同 PD4 */
        } else if (rx == 0xD1U) {
          s_test2_start_req = 1U;
        } else if (rx == 0xD2U) {
          s_test3_case1_start_req = 1U;
        } else if (rx == 0xD3U) {
          s_test3_case2_start_req = 1U;
        } else if (rx == 0xD4U) {
          s_test3_case3_start_req = 1U;
        } else if (rx == 0xD5U) {
          s_test3_case4_start_req = 1U;
        } else if (rx == 0xD6U) {
          s_test4_start_req = 1U;
        } else if (rx == 0xD7U) {
          s_test5_start_req = 1U; /* TEST5: UART6连接检测 */
        } else if (rx == 0xD8U) {
          s_test6_start_req = 1U; /* TEST6: 灰度 I2C 诊断 */
        } else if (rx == 0xD9U) {
          App_Test7_UserToggle();
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
          s_q4_start_req = 1U; /* Q4-V */
        }
        s_uart_seq_state = (rx == 0xAAU) ? 1U : 0U;
      }
    }
  }

  /* UART6：
   * 1) 收到 AA CC 06 后回发 AA CC 06；
   * 2) 仅 TEST5 触发发送 AA CC 05，1000ms 内需收到 AA CC 05，否则连接失败。
   */
  {
    static const uint8_t k_probe_aa_cc_05[3] = {0xAAU, 0xCCU, 0x05U};
    static const uint8_t k_ack_aa_cc_06[3] = {0xAAU, 0xCCU, 0x06U};
    uint8_t rx6;
    HAL_StatusTypeDef st_u6;

    if (s_uart6_link_wait_resp != 0U &&
        (int32_t)(now_ms - s_uart6_link_deadline_ms) >= 0) {
      s_uart6_link_wait_resp = 0U;
      if (s_uart6_test5_active != 0U) {
        s_uart6_test5_active = 0U;
        s_pending_beeps = 2U; /* TEST5 失败：蜂鸣2次 */
      }
    }

    if (s_test5_start_req != 0U) {
      s_test5_start_req = 0U;
      st_u6 = DebugUart6_Send(k_probe_aa_cc_05, 3U, 20U);
      if (st_u6 == HAL_OK) {
        s_uart6_link_wait_resp = 1U;
        s_uart6_link_deadline_ms = now_ms + 1000U;
        s_uart6_test5_active = 1U;
        s_uart6_test5_deadline_ms = s_uart6_link_deadline_ms;
      } else {
        s_uart6_test5_active = 0U;
        s_pending_beeps = 2U; /* TEST5 发送失败：按连接失败处理 */
      }
    }

    while (DebugUart6_ReadByte(&rx6)) {
      if (s_uart6_rx_prev2 == 0xAAU && s_uart6_rx_prev1 == 0xCCU && rx6 == 0x06U) {
        (void)DebugUart6_Send(k_ack_aa_cc_06, 3U, 20U);
      }

      if (s_uart6_rx_prev2 == 0xAAU && s_uart6_rx_prev1 == 0xCCU && rx6 == 0x05U) {
        s_uart6_link_wait_resp = 0U;
        if (s_uart6_test5_active != 0U &&
            (int32_t)(now_ms - s_uart6_test5_deadline_ms) <= 0) {
          s_uart6_test5_active = 0U;
          s_pending_beeps = 1U; /* TEST5 成功：蜂鸣1次 */
        }
      } else if (s_uart6_rx_prev2 == 0xAAU && s_uart6_rx_prev1 == 0xCCU &&
                 (rx6 == 0xEEU || rx6 == 0x01U || rx6 == 0x02U || rx6 == 0x03U ||
                  rx6 == 0x04U)) {
        if (s_q4v_uart6_wait_resp != 0U) {
          s_q4v_uart6_resp_code = rx6;
          s_q4v_uart6_resp_ready = 1U;
        }
      } else if (s_uart6_rx_prev2 == 0xAAU && s_uart6_rx_prev1 == 0xCCU && rx6 == 0xBBU) {
        if (s_pd3_q4v_phase != 0U && s_pd3_bb_beep_done == 0U) {
          BuzzerDrv_Beep(80U);
          s_pd3_bb_beep_done = 1U;
        }
      }

      s_uart6_rx_prev2 = s_uart6_rx_prev1;
      s_uart6_rx_prev1 = rx6;
    }
  }

}

static void App_Task_50ms(void)
{
}

static void App_Task_100ms(void)
{
  uint32_t now_ms;

  /* 10 Hz：与调度 100ms 节拍一致，巡检 I2C Ping 判断灰度是否在线 */
  App_GraySensorWatchdog10Hz();

  now_ms = HAL_GetTick();
  App_Test6_Process100ms(now_ms);

#if FW_Q1_ENABLE != 0
  static uint8_t s_boot_guard_init;
  static uint32_t s_boot_guard_deadline_ms;
  static uint8_t s_mode; /* 1=Q1,2=Q2-1,3=Q2-2,4=Q3-1,5=Q3-2,6=Q4-V,7=TEST1,8=TEST2,9=TEST3-1,10=TEST3-2,11=TEST3-3,12=TEST3-4,13=TEST4,14=Q4-1,15=TEST1-REC */
  static uint8_t s_state;
  static uint8_t s_wait_ticks;
  static uint8_t s_round;
  static uint8_t s_ret_seg;
  HAL_StatusTypeDef st;

  const int32_t pre_move_y_m21739 = 1739; /* 0.08m * 21739 */
  const int32_t move_px_m21739 = 0;
  const int32_t move_py_m21739 = 21739; /* 1.0m * 21739 */
  const int32_t move_py_test1_02_m21739 = 4348; /* 0.2m * 21739 */
  const int32_t move_py_test1_04_m21739 = 8696; /* 0.4m * 21739 */
  const int32_t move_py_q3_case2_start_m21739 = 2174; /* 0.1m * 21739 */
  const int32_t move_py_test3_case1_m21739 = 16196; /* 0.745m * 21739 */
  const int32_t move_py_test3_case2_m21739 = 16196; /* 0.745m * 21739 */
  const int32_t move_py_test3_case3_m21739 = 10217; /* 0.47m * 21739 */
  const int32_t move_py_test3_case4_m21739 = 20435; /* 0.94m * 21739 */
  const int32_t move_py_half_m21739 = 10870; /* 0.5m * 21739 */
  const int32_t move_py_q4_first_m21739 = 11957; /* 0.55m * 21739 */
  const int32_t move_py_q4_second_m21739 = 9783; /* 0.45m * 21739 */
  const int32_t move_py_ret_last_m21739 = 19565; /* 0.9m * 21739 */
  const int32_t move_py_q1_last_m21739 = 20000; /* 0.92m * 21739 */
  const int32_t move_pz_m21739 = 0;
  const int16_t move_speed_mps100 = 3000; /* 30 m/s * 100 */
  const int16_t move_speed_q4v_last_seg_mps100 = 1500; /* 速度15档，Q4-V末段0.92m直走 */
  const int16_t move_speed_q3_case1_uniform_mps100 = 1000; /* 10 m/s * 100 */
  const int16_t move_speed_q3_case2_uniform_mps100 = 1000; /* 10 m/s * 100 */
  const int16_t move_speed_detect_mps100 = 500; /* 5 m/s * 100，仅0.6m检测段 */
  const int32_t stepper_pulse_90deg = 800;
  const int32_t stepper_speed_turn = 100;
  const int32_t rot_left_90 = -900000;  /* -90deg * 10000 */
  const int32_t rot_left_1535 = -1535000; /* -153.5deg * 10000 */
  const int32_t rot_left_1165 = -1165000; /* -116.5deg * 10000 */
  const int32_t rot_left_135 = -1350000; /* -135deg * 10000 */
  const int32_t rot_right_90 = 900000;  /* +90deg * 10000 */
  const int32_t rot_u_turn = -1800000;  /* -180deg * 10000 */
  const int16_t rot_vmax = 1500;        /* 15 deg/s * 100 */
  const int16_t rot_vmax_q3_case2_slow = 1200; /* 12 deg/s * 100，Q3-2后3次右转慢一点 */
  const uint8_t wait_ticks_q3_case2_first_turn = 2U; /* 第1次右转后等待减少200ms */
  const uint8_t wait_ticks_q3_case2_other_turn = 4U; /* 后3次右转后等待减少200ms */
  const uint8_t wait_ticks_default = 14U;          /* 1.4s */
  const uint8_t wait_ticks_detect_seg = 22U;       /* 约2.2s，0.4m检测段等待 */
  const uint8_t wait_ticks_pre_detect = 6U;        /* 0.6s，校准直行后进入测量段前等待 */
  const uint8_t wait_ticks_q3_case1_uniform = 28U; /* 2.8s */

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
    s_q4_case1_start_req = 0U;
    s_test1_start_req = 0U;
    s_test1_rec_start_req = 0U;
    s_test2_start_req = 0U;
    s_test3_case1_start_req = 0U;
    s_test3_case2_start_req = 0U;
    s_test3_case3_start_req = 0U;
    s_test3_case4_start_req = 0U;
    s_test4_start_req = 0U;
    s_test6_start_req = 0U;
    s_test7_active = 0U;
    s_test7_prev_raw_cross = 0U;
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
  } else if (s_q4_case1_start_req != 0U) {
    s_q4_case1_start_req = 0U;
    BuzzerDrv_Beep(80U);
    s_q3_case1_cross_detect_active = 0U;
    s_q3_case1_cross_count = 0U;
    s_q3_case1_cross_latched = 0U;
    s_test1_dist_measure_active = 0U;
    s_test1_last_cross_valid = 0U;
    s_test1_last_cross_tick_ms = 0U;
    s_mode = 14U;
    s_state = 1U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
  } else if (s_test1_rec_start_req != 0U) {
    s_test1_rec_start_req = 0U;
    BuzzerDrv_Beep(80U);
    s_q3_case1_cross_detect_active = 0U;
    s_q3_case1_cross_count = 0U;
    s_q3_case1_cross_latched = 0U;
    s_test1_dist_measure_active = 0U;
    s_test1_last_cross_valid = 0U;
    s_test1_last_cross_tick_ms = 0U;
    s_test1_rec_active = 0U;
    s_test1_rec_count = 0U;
    s_mode = 15U;
    s_state = 30U;
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
  } else if (s_test3_case1_start_req != 0U) {
    s_test3_case1_start_req = 0U;
    BuzzerDrv_Beep(80U);
    s_q3_case1_cross_detect_active = 0U;
    s_q3_case1_cross_count = 0U;
    s_q3_case1_cross_latched = 0U;
    s_test1_dist_measure_active = 0U;
    s_test1_last_cross_valid = 0U;
    s_test1_last_cross_tick_ms = 0U;
    s_mode = 9U;
    s_state = 70U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
  } else if (s_test3_case2_start_req != 0U) {
    s_test3_case2_start_req = 0U;
    BuzzerDrv_Beep(80U);
    s_q3_case1_cross_detect_active = 0U;
    s_q3_case1_cross_count = 0U;
    s_q3_case1_cross_latched = 0U;
    s_test1_dist_measure_active = 0U;
    s_test1_last_cross_valid = 0U;
    s_test1_last_cross_tick_ms = 0U;
    s_mode = 10U;
    s_state = 74U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
  } else if (s_test3_case3_start_req != 0U) {
    s_test3_case3_start_req = 0U;
    BuzzerDrv_Beep(80U);
    s_q3_case1_cross_detect_active = 0U;
    s_q3_case1_cross_count = 0U;
    s_q3_case1_cross_latched = 0U;
    s_test1_dist_measure_active = 0U;
    s_test1_last_cross_valid = 0U;
    s_test1_last_cross_tick_ms = 0U;
    s_mode = 11U;
    s_state = 78U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
  } else if (s_test3_case4_start_req != 0U) {
    s_test3_case4_start_req = 0U;
    BuzzerDrv_Beep(80U);
    s_q3_case1_cross_detect_active = 0U;
    s_q3_case1_cross_count = 0U;
    s_q3_case1_cross_latched = 0U;
    s_test1_dist_measure_active = 0U;
    s_test1_last_cross_valid = 0U;
    s_test1_last_cross_tick_ms = 0U;
    s_mode = 12U;
    s_state = 82U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
  } else if (s_test4_start_req != 0U) {
    s_test4_start_req = 0U;
    BuzzerDrv_Beep(80U);
    s_q3_case1_cross_detect_active = 0U;
    s_q3_case1_cross_count = 0U;
    s_q3_case1_cross_latched = 0U;
    s_test1_dist_measure_active = 0U;
    s_test1_last_cross_valid = 0U;
    s_test1_last_cross_tick_ms = 0U;
    s_mode = 13U;
    s_state = 86U;
    s_wait_ticks = 0U;
    s_round = 0U;
    s_ret_seg = 0U;
  }

  switch (s_state) {
    case 0U: /* 等待 PD3(Q1) / PD4(Q2-1) / PD5(Q2-2) */
      return;

    case 1U: /* 启动动作：Q3-2按专用流程，其余先前进0.08m */
      if (s_mode == 5U) {
        s_wait_ticks = 0U;
        s_round = 0U;
        s_state = 92U;
        break;
      } else if (s_mode == 6U || s_mode == 14U) {
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
      if (s_wait_ticks++ < ((s_mode == 4U && s_round == 1U) ? 3U : 5U)) {
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

    case 15U: /* 兼容保留：旧Q3-2流程（当前不再进入） */
      if (s_wait_ticks++ < 12U) {
        return;
      }
      App_Motion_StopAndBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
      break;

    case 16U: /* Q4-V 前置前进稳定等待约 0.5s */
      if (s_wait_ticks++ < 5U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 17U;
      break;

    case 17U: /* Q4-V 左转 90 度 */
      st = DflinkChassis_SendRotation(0, 0, rot_left_90, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 18U;
      break;

    case 18U: /* Q4-V 转向稳定等待约 0.5s */
      if (s_wait_ticks++ < 5U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 19U;
      break;

    case 19U: /* Q4-V 前进 0.55m */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_q4_first_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 126U;
      break;

    case 126U: { /* Q4-V 第4步完成后：UART6 发送 AA CC 08 */
      static const uint8_t k_q4v_uart6_aa_cc_08[3] = {0xAAU, 0xCCU, 0x08U};
      st = DebugUart6_Send(k_q4v_uart6_aa_cc_08, 3U, 50U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 20U;
      break;
    }

    case 20U: /* Q4-V 等待 4.0s */
      if (s_wait_ticks++ < 40U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 21U;
      break;

    case 21U: /* Q4-V 再前进 0.5m（6s 等待后） */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_half_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 22U;
      break;

    case 22U: /* Q4-V 第6步0.5m后稳定等待约 1.4s */
      if (s_wait_ticks++ < 14U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 23U;
      break;

    case 23U: /* Q4-V 再左转 90 度 */
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

    case 24U: /* Q4-V 再次转向稳定等待约 0.5s */
      if (s_wait_ticks++ < 5U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 29U;
      break;

    case 29U: { /* Q4-V 第7步后：UART6 发送 AA CC 07 */
      static const uint8_t k_q4v_uart6_aa_cc_07[3] = {0xAAU, 0xCCU, 0x07U};
      st = DebugUart6_Send(k_q4v_uart6_aa_cc_07, 3U, 50U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 25U;
      break;
    }

    case 25U: /* Q4-V 最后前进 0.92m（直走速度15档） */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_q1_last_m21739,
                                             move_pz_m21739, move_speed_q4v_last_seg_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 26U;
      break;

    case 26U: /* Q4-V 第8步直走后稳定等待约 3.0s */
      if (s_wait_ticks++ < 30U) {
        return;
      }
      if (s_mode == 14U) {
        s_wait_ticks = 0U;
        s_state = 122U; /* Q4-1: Q4-V后接TEST3-2 */
        break;
      }
      if (s_mode == 6U) {
        s_wait_ticks = 0U;
        s_state = 27U;
        break;
      }
      App_Motion_StopAndBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
      break;

    case 27U: { /* Q4-V 发送 UART6 查询帧 AA CC 00 */
      static const uint8_t k_q4v_query_aa_cc_00[3] = {0xAAU, 0xCCU, 0x00U};
      st = DebugUart6_Send(k_q4v_query_aa_cc_00, 3U, 50U);
      if (st != HAL_OK) {
        return;
      }
      s_q4v_uart6_wait_resp = 1U;
      s_q4v_uart6_resp_ready = 0U;
      s_q4v_uart6_resp_code = 0U;
      s_wait_ticks = 0U;
      s_state = 28U;
      break;
    }

    case 28U: /* Q4-V 等待 UART6 决策帧 AA CC XX */
      if (s_q4v_uart6_resp_ready == 0U) {
        return;
      }

      s_q4v_uart6_wait_resp = 0U;
      s_q4v_uart6_resp_ready = 0U;

      if (s_q4v_uart6_resp_code == 0xEEU) {
        App_Motion_StopNoBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
        s_pending_beeps = 3U; /* 连续触发3次 */
        break;
      }

      s_q3_case1_cross_detect_active = 0U;
      s_q3_case1_cross_count = 0U;
      s_q3_case1_cross_latched = 0U;
      s_test1_dist_measure_active = 0U;
      s_test1_last_cross_valid = 0U;
      s_test1_last_cross_tick_ms = 0U;
      s_wait_ticks = 0U;
      s_round = 0U;
      s_ret_seg = 0U;

      if (s_q4v_uart6_resp_code == 0x01U) {      /* AA CC 01 -> TEST3-4 */
        s_mode = 12U;
        s_state = 82U;
      } else if (s_q4v_uart6_resp_code == 0x02U) { /* AA CC 02 -> TEST3-1 */
        s_mode = 9U;
        s_state = 70U;
      } else if (s_q4v_uart6_resp_code == 0x03U) { /* AA CC 03 -> TEST3-3 */
        s_mode = 11U;
        s_state = 78U;
      } else if (s_q4v_uart6_resp_code == 0x04U) { /* AA CC 04 -> TEST3-2 */
        s_mode = 10U;
        s_state = 74U;
      } else {
        App_Motion_StopNoBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
      }
      break;

    case 30U: /* TEST1: 0.2m前先做灰度偏移修正 */
      s_measure_corr_rz10000 = App_CalcGrayCorrRz10000();
      if (s_measure_corr_rz10000 != 0) {
        st = DflinkChassis_SendRotation(0, 0, s_measure_corr_rz10000, rot_vmax, 200U);
        if (st != HAL_OK) {
          return;
        }
        s_wait_ticks = 0U;
        s_state = 42U;
        break;
      }
      s_state = 43U;
      break;

    case 42U: /* TEST1: 修正转向稳定等待（与 Q3-1 替换段 case 59 一致：0.3s） */
      if (s_wait_ticks++ < 3U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 43U;
      break;

    case 43U: /* TEST1: 匀速前进 0.2m */
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
      if (s_wait_ticks++ < wait_ticks_pre_detect) {
        return;
      }
      s_wait_ticks = 0U;
      if (s_measure_corr_rz10000 != 0) {
        s_state = 44U;
      } else {
        s_state = 33U;
      }
      break;

    case 44U: /* TEST1: 0.2m后反向转回修正角度 */
      st = DflinkChassis_SendRotation(0, 0, -s_measure_corr_rz10000, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 45U;
      break;

    case 45U: /* TEST1: 反向转回稳定等待（与 Q3-1 替换段 case 62 一致：0.2s） */
      if (s_wait_ticks++ < 2U) {
        return;
      }
      s_wait_ticks = 0U;
      s_measure_corr_rz10000 = 0;
      s_state = 33U;
      break;

    case 32U: /* TEST1: 蜂鸣器触发一次 */
      BuzzerDrv_Beep(80U);
      s_state = 33U;
      break;

    case 33U: /* TEST1 / TEST1-REC: 匀速前进 0.4m；REC 仅采样，段末离线算距 */
      App_ClearMeasureLogs();
      if (s_mode == 15U) {
        s_q3_case1_cross_detect_active = 0U;
        s_test1_dist_measure_active = 0U;
      } else {
        s_q3_case1_cross_detect_active = 1U;
        s_test1_dist_measure_active = 1U;
      }
      s_q3_case1_cross_count = 0U;
      s_q3_case1_cross_latched = 0U;
      s_cross_raw_on_cnt = 0U;
      s_cross_raw_off_cnt = 0U;
      s_cross_confirmed = 0U;
      s_test1_last_cross_valid = 0U;
      s_test1_last_cross_tick_ms = 0U;
      st = DflinkChassis_SendVelDisplacement(move_px_m21739, move_py_test1_04_m21739,
                                             move_pz_m21739, move_speed_detect_mps100,
                                             200U);
      if (st != HAL_OK) {
        return;
      }
      if (s_mode == 15U) {
        s_test1_rec_active = 1U;
        s_test1_rec_count = 0U;
      }
      s_wait_ticks = 0U;
      s_state = 34U;
      break;

    case 34U: /* TEST1: 第二段匀速稳定等待（含路口检测） */
      if (s_wait_ticks++ < wait_ticks_detect_seg) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 37U;
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

    case 37U: /* TEST1 / TEST1-REC: 自适应前进 0.4m */
      if (s_mode == 15U) {
        s_test1_rec_active = 0U;
        App_Test1Rec_SendFireWaterUsart1();
        App_AnalyzeTest1RecBuffer();
      }
      App_FlushMeasureLogs();
      s_q3_case1_cross_detect_active = 0U;
      s_q3_case1_cross_count = 0U;
      s_q3_case1_cross_latched = 0U;
      s_test1_dist_measure_active = 0U;
      s_test1_last_cross_valid = 0U;
      s_test1_last_cross_tick_ms = 0U;
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_test1_04_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 38U;
      break;

    case 38U: /* TEST1: 末段稳定等待后停车 */
      if (s_wait_ticks++ < ((s_mode == 5U) ? 12U : wait_ticks_default)) {
        return;
      }
      if (s_mode == 5U) {
        s_wait_ticks = 0U;
        s_state = 101U;
        break;
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

    case 50U: /* Q3-1 第2段替换流程: 0.2m前先做灰度偏移修正 */
      s_measure_corr_rz10000 = App_CalcGrayCorrRz10000();
      if (s_measure_corr_rz10000 != 0) {
        st = DflinkChassis_SendRotation(0, 0, s_measure_corr_rz10000, rot_vmax, 200U);
        if (st != HAL_OK) {
          return;
        }
        s_wait_ticks = 0U;
        s_state = 59U;
        break;
      }
      s_state = 60U;
      break;

    case 59U: /* Q3-1 第2段替换流程: 修正转向稳定等待 */
      if (s_wait_ticks++ < 3U) { /* 缩短到0.3s */
        return;
      }
      s_wait_ticks = 0U;
      s_state = 60U;
      break;

    case 60U: /* Q3-1 第2段替换流程: 匀速前进 0.2m */
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
      if (s_wait_ticks++ < wait_ticks_pre_detect) {
        return;
      }
      s_wait_ticks = 0U;
      if (s_measure_corr_rz10000 != 0) {
        s_state = 61U;
      } else {
        s_state = 53U;
      }
      break;

    case 61U: /* Q3-1 第2段替换流程: 0.2m后反向转回修正角度 */
      st = DflinkChassis_SendRotation(0, 0, -s_measure_corr_rz10000, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 62U;
      break;

    case 62U: /* Q3-1 第2段替换流程: 反向转回稳定等待 */
      if (s_wait_ticks++ < 2U) { /* 缩短到0.2s */
        return;
      }
      s_wait_ticks = 0U;
      s_measure_corr_rz10000 = 0;
      s_state = 53U;
      break;

    case 52U: /* Q3-1 第2段替换流程: 蜂鸣一次 */
      BuzzerDrv_Beep(80U);
      s_state = 53U;
      break;

    case 53U: /* Q3-1 第2段替换流程: 匀速前进 0.4m，开启路口检测 */
      App_ClearMeasureLogs();
      s_q3_case1_cross_detect_active = 1U;
      s_q3_case1_cross_count = 0U;
      s_q3_case1_cross_latched = 0U;
      s_cross_raw_on_cnt = 0U;
      s_cross_raw_off_cnt = 0U;
      s_cross_confirmed = 0U;
      s_test1_dist_measure_active = 1U;
      s_test1_last_cross_valid = 0U;
      s_test1_last_cross_tick_ms = 0U;
      st = DflinkChassis_SendVelDisplacement(move_px_m21739, move_py_test1_04_m21739,
                                             move_pz_m21739, move_speed_detect_mps100,
                                             200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 54U;
      break;

    case 54U: /* Q3-1 第2段替换流程: 第二段匀速稳定等待（含路口检测） */
      if (s_wait_ticks++ < wait_ticks_detect_seg) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 57U;
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

    case 57U: /* Q3-1 第2段替换流程: 自适应前进 0.4m */
      App_FlushMeasureLogs();
      s_q3_case1_cross_detect_active = 0U;
      s_q3_case1_cross_count = 0U;
      s_q3_case1_cross_latched = 0U;
      s_test1_dist_measure_active = 0U;
      s_test1_last_cross_valid = 0U;
      s_test1_last_cross_tick_ms = 0U;
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_test1_04_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 58U;
      break;

    case 58U: /* Q3-1 第2段替换流程结束，缩短末段稳定等待后推进轮次 */
      if (s_wait_ticks++ < 10U) { /* 1.0s */
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

    case 70U: /* TEST3-1: 左转 153.5 度 */
      st = DflinkChassis_SendRotation(0, 0, rot_left_1535, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 71U;
      break;

    case 71U: /* TEST3-1: 转向稳定等待 */
      if (s_wait_ticks++ < 5U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 72U;
      break;

    case 72U: /* TEST3-1: 前进 0.745m */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_test3_case1_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 73U;
      break;

    case 73U: /* TEST3-1: 稳定等待后停车 */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      App_Motion_StopAndBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
      break;

    case 74U: /* TEST3-2: 左转 116.5 度 */
      st = DflinkChassis_SendRotation(0, 0, rot_left_1165, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 75U;
      break;

    case 75U: /* TEST3-2: 转向稳定等待 */
      if (s_wait_ticks++ < 5U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 76U;
      break;

    case 76U: /* TEST3-2: 前进 0.745m */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_test3_case2_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 77U;
      break;

    case 77U: /* TEST3-2: 稳定等待后停车 */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      App_Motion_StopAndBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
      break;

    case 78U: /* TEST3-3: 左转 135 度 */
      st = DflinkChassis_SendRotation(0, 0, rot_left_135, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 79U;
      break;

    case 79U: /* TEST3-3: 转向稳定等待 */
      if (s_wait_ticks++ < 5U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 80U;
      break;

    case 80U: /* TEST3-3: 前进 0.47m */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_test3_case3_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 81U;
      break;

    case 81U: /* TEST3-3: 稳定等待后停车 */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      App_Motion_StopAndBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
      break;

    case 82U: /* TEST3-4: 左转 135 度 */
      st = DflinkChassis_SendRotation(0, 0, rot_left_135, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 83U;
      break;

    case 83U: /* TEST3-4: 转向稳定等待 */
      if (s_wait_ticks++ < 5U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 84U;
      break;

    case 84U: /* TEST3-4: 前进 0.94m */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_test3_case4_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 85U;
      break;

    case 85U: /* TEST3-4: 稳定等待后停车 */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      App_Motion_StopAndBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
      break;

    case 86U: /* TEST4: 自适应前进 0.55m */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_q4_first_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 87U;
      break;

    case 87U: /* TEST4: 等待 4.0s */
      if (s_wait_ticks++ < 40U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 88U;
      break;

    case 88U: /* TEST4: 自适应前进 0.45m */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_q4_second_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 89U;
      break;

    case 89U: /* TEST4: 第二段前进后稳定等待 */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 90U;
      break;

    case 90U: /* TEST4: 向后走 1.0m，速度10 */
      st = DflinkChassis_SendVelDisplacement(move_px_m21739, -move_py_m21739, move_pz_m21739,
                                             move_speed_q3_case2_uniform_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 91U;
      break;

    case 91U: /* TEST4: 后退稳定等待后停车 */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      App_Motion_StopAndBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
      break;

    case 122U: /* Q4-1: 接续 TEST3-2 左转 116.5 度 */
      st = DflinkChassis_SendRotation(0, 0, rot_left_1165, rot_vmax, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 123U;
      break;

    case 123U: /* Q4-1: 转向稳定等待 */
      if (s_wait_ticks++ < 5U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 124U;
      break;

    case 124U: /* Q4-1: 接续 TEST3-2 前进 0.745m */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_test3_case2_m21739,
                                             move_pz_m21739, move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 125U;
      break;

    case 125U: /* Q4-1: 稳定等待后停车 */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      App_Motion_StopAndBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
      break;

    case 92U: /* Q3-2: 先自适应前进 0.1m */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_q3_case2_start_m21739, move_pz_m21739,
                                             move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 93U;
      break;

    case 93U: /* Q3-2: 0.1m 稳定等待 */
      if (s_wait_ticks++ < 4U) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 94U;
      break;

    case 94U: /* Q3-2: （右转90 + 自适应1m）循环2次 */
      if (s_round >= 2U) {
        s_state = 98U; /* 循环完成，进入测距前右转 */
      } else {
        s_state = 95U;
      }
      break;

    case 95U: /* Q3-2: 右转 90 度 */
      st = DflinkChassis_SendRotation(0, 0, rot_right_90,
                                      (s_round == 0U) ? rot_vmax : rot_vmax_q3_case2_slow, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 96U;
      break;

    case 96U: /* Q3-2: 右转后稳定等待 */
      if (s_wait_ticks++ < ((s_round == 0U) ? wait_ticks_q3_case2_first_turn
                                            : wait_ticks_q3_case2_other_turn)) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 97U;
      break;

    case 97U: /* Q3-2: 自适应直走 1m */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_m21739, move_pz_m21739,
                                             move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 107U;
      break;

    case 107U: /* Q3-2: 1m 稳定等待后进入下一轮 */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      s_wait_ticks = 0U;
      s_round++;
      s_state = 94U;
      break;

    case 98U: /* Q3-2: 进入测距流程前再右转 90 度 */
      st = DflinkChassis_SendRotation(0, 0, rot_right_90, rot_vmax_q3_case2_slow, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 99U;
      break;

    case 99U: /* Q3-2: 转向稳定等待 */
      if (s_wait_ticks++ < wait_ticks_q3_case2_other_turn) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 30U; /* 进入与TEST1同构的测距流程 */
      break;

    case 101U: /* Q3-2: 测距流程后右转 90 度 */
      st = DflinkChassis_SendRotation(0, 0, rot_right_90, rot_vmax_q3_case2_slow, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 102U;
      break;

    case 102U: /* Q3-2: 末次右转后稳定等待 */
      if (s_wait_ticks++ < wait_ticks_q3_case2_other_turn) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 103U;
      break;

    case 103U: /* Q3-2: 末次自适应直走 0.9m */
      st = DflinkChassis_SendAdaptConstPMove(move_px_m21739, move_py_ret_last_m21739, move_pz_m21739,
                                             move_speed_mps100, 200U);
      if (st != HAL_OK) {
        return;
      }
      s_wait_ticks = 0U;
      s_state = 104U;
      break;

    case 104U: /* Q3-2: 收尾稳定等待后停车 */
      if (s_wait_ticks++ < wait_ticks_default) {
        return;
      }
      App_Motion_StopAndBeep(&s_mode, &s_state, &s_wait_ticks, &s_round, &s_ret_seg);
      break;

    default:
      break;
  }
#endif
}

void App_RegisterTasks(void)
{
  ButtonDrv_Init();
  s_heading_lock_enabled = 1U;
  s_heading_lock_init_done = 0U;
  StepperMotorDrv_Init(&s_stepper_motor, &huart2, 0x01U);
  s_stepper_motor_inited = 1U;
  s_stepper_motor_enabled = 0U;
  GwGraySensor_InitDefaults(&s_gray_sensor, &hi2c1);
  s_gray_sensor_inited = GwGraySensor_InitPingWait(&s_gray_sensor, 300U) ? 1U : 0U;
  s_test7_active = 0U;
  s_test7_prev_raw_cross = 0U;
  s_q3_case1_cross_detect_active = 0U;
  s_q3_case1_cross_count = 0U;
  s_q3_case1_cross_latched = 0U;
  s_test1_dist_measure_active = 0U;
  s_test1_last_cross_valid = 0U;
  s_test1_last_cross_tick_ms = 0U;
  s_cross_raw_on_cnt = 0U;
  s_cross_raw_off_cnt = 0U;
  s_cross_confirmed = 0U;
  s_meas_log_count = 0U;

  (void)Scheduler_AddTask(App_Task_1ms,   1U);
  (void)Scheduler_AddTask(App_Task_10ms, 10U);
  (void)Scheduler_AddTask(App_Task_50ms, 50U);
  (void)Scheduler_AddTask(App_Task_100ms, 100U);
}
