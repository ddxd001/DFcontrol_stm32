#include "app_tasks.h"
#include "scheduler.h"

#include <stdio.h>

#include "buzzer_drv.h"
#include "debug_uart.h"
#include "gw_gray_sensor.h"

static GwGraySensor s_gray;

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
  uint8_t a[8];
  uint16_t off = 0U;
  char line[168];
  int n;
  bool have_off;

  if (!GwGray_ReadDigitalUpdate(&s_gray) || !GwGray_ReadAnalog8(&s_gray, a)) {
    (void)DebugUart_Send((const uint8_t *)"GRAY read fail\r\n", 18U, 50U);
    return;
  }

  have_off = GwGray_ReadLineOffsetU16(&s_gray, &off);
  if (have_off) {
    n = snprintf(line, sizeof(line),
                 "GRAY dig=0x%02X inv=0x%02X A=%u,%u,%u,%u,%u,%u,%u,%u off=%u\r\n",
                 (unsigned)s_gray.digital_raw, (unsigned)s_gray.digital_inv,
                 (unsigned)a[0], (unsigned)a[1], (unsigned)a[2], (unsigned)a[3],
                 (unsigned)a[4], (unsigned)a[5], (unsigned)a[6], (unsigned)a[7],
                 (unsigned)off);
  } else {
    n = snprintf(line, sizeof(line),
                 "GRAY dig=0x%02X inv=0x%02X A=%u,%u,%u,%u,%u,%u,%u,%u off=NA\r\n",
                 (unsigned)s_gray.digital_raw, (unsigned)s_gray.digital_inv,
                 (unsigned)a[0], (unsigned)a[1], (unsigned)a[2], (unsigned)a[3],
                 (unsigned)a[4], (unsigned)a[5], (unsigned)a[6], (unsigned)a[7]);
  }

  if (n > 0 && n < (int)sizeof(line)) {
    (void)DebugUart_Send((const uint8_t *)line, (uint16_t)n, 100U);
  }
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
  GwGraySensor_InitDefaults(&s_gray, &hi2c1);

  (void)Scheduler_AddTask(App_Task_1ms,   1U);
  (void)Scheduler_AddTask(App_Task_10ms, 10U);
  (void)Scheduler_AddTask(App_Task_50ms, 50U);
  (void)Scheduler_AddTask(App_Task_100ms, 100U);
  (void)Scheduler_AddTask(App_Task_BeepSelftest_1Hz, 1000U);
}
