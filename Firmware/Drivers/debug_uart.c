/*
 * USART1 调试收发：字节级 RX 中断 + 环形缓冲；TX 为 HAL_UART_Transmit（阻塞式，带超时）。
 * 若在其它文件中再实现 HAL_UART_RxCpltCallback，请将 USART1 分支合并进同一回调。
 */
#include "debug_uart.h"

#include <stdarg.h>
#include <stdio.h>

#include "fw_config.h"
#include "stm32f407xx.h"
#include "usart.h"

#define DBG_RX_CAP 512U

static UART_HandleTypeDef *s_uart;
static uint8_t s_rx_ring[DBG_RX_CAP];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t s_rx_byte;

static void dbg_emit_text(char *text, int n_chars, uint32_t tout_ms)
{
  uint16_t len16;

  if (text == 0 || n_chars <= 0) {
    return;
  }

  len16 = ((unsigned int)n_chars > 0xFFFFU) ? 0xFFFFU : (uint16_t)n_chars;
  (void)DebugUart_Send((uint8_t *)text, len16, tout_ms);
}

static void dbg_emit_fmt(uint32_t tout_ms, const char *fmt, ...)
{
  char stack_buf[112];
  va_list ap;
  int n;

  va_start(ap, fmt);
  n = vsnprintf(stack_buf, sizeof(stack_buf), fmt, ap);
  va_end(ap);

  if (n <= 0) {
    return;
  }
  if (n >= (int)sizeof(stack_buf)) {
    n = (int)sizeof(stack_buf) - 1;
  }

  dbg_emit_text(stack_buf, n, tout_ms);
}

static bool dbg_rx_try_push_isr(uint8_t b)
{
  uint16_t h;
  uint16_t next_h;

  h = s_rx_head;
  next_h = (uint16_t)((h + 1U) % DBG_RX_CAP);
  if (next_h == s_rx_tail) {
    return false;
  }
  s_rx_ring[h] = b;
  s_rx_head = next_h;
  return true;
}

void DebugUart_Init(void)
{
  s_uart = &huart1;

  s_rx_head = 0U;
  s_rx_tail = 0U;

  HAL_NVIC_SetPriority(USART1_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  if (HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U) != HAL_OK) {
    /* 调用方可在运行时通过 Read 无数据推断 */
  }
}

void DebugUart_PrintDeviceInfo(void)
{
  /* 须在 MX_USART1 + SystemClock_Config 完成之后调用 */

  dbg_emit_fmt(100U, "\r\n=== %s ===\r\n", FW_FIRMWARE_NAME);
  dbg_emit_fmt(50U, "MCU: STM32F407VETx\r\n");

  dbg_emit_fmt(100U, "SYSCLK_Hz: %lu\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());
  dbg_emit_fmt(100U, "USART1_baud: %lu 8N1\r\n", (unsigned long)huart1.Init.BaudRate);

  dbg_emit_fmt(100U, "Rx echo: on (verbatim)\r\n---\r\n");
}

uint32_t DebugUart_EchoRxToTx(uint32_t max_chunk, uint32_t tout_ms)
{
  static uint8_t tx_tmp[160];
  uint32_t rd;
  uint16_t snd;

  if (max_chunk == 0U || max_chunk > sizeof(tx_tmp)) {
    max_chunk = sizeof(tx_tmp);
  }

  rd = DebugUart_ReadBytes(tx_tmp, max_chunk);
  if (rd == 0U) {
    return 0U;
  }

  snd = (uint16_t)((rd > 0xFFFFU) ? 0xFFFFU : rd);
  if (DebugUart_Send(tx_tmp, snd, tout_ms) != HAL_OK) {
    return 0U;
  }

  return rd;
}

HAL_StatusTypeDef DebugUart_Send(const uint8_t *buf, uint16_t len, uint32_t tout_ms)
{
  if (buf == 0 || len == 0U || s_uart == 0) {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(s_uart, (uint8_t *)buf, len, tout_ms);
}

uint32_t DebugUart_RxAvail(void)
{
  uint16_t h;
  uint16_t t;

  h = s_rx_head;
  t = s_rx_tail;

  return (uint32_t)((DBG_RX_CAP + h - t) % DBG_RX_CAP);
}

bool DebugUart_ReadByte(uint8_t *out)
{
  uint8_t v;

  if (out == 0) {
    return false;
  }

  if (s_rx_head == s_rx_tail) {
    return false;
  }

  v = s_rx_ring[s_rx_tail];
  s_rx_tail = (uint16_t)((s_rx_tail + 1U) % DBG_RX_CAP);
  *out = v;
  return true;
}

uint32_t DebugUart_ReadBytes(uint8_t *buf, uint32_t maxlen)
{
  uint32_t n;

  if (buf == 0 || maxlen == 0U) {
    return 0U;
  }

  n = 0U;
  while (n < maxlen && DebugUart_ReadByte(&buf[n])) {
    n++;
  }

  return n;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (s_uart == 0 || huart != s_uart) {
    return;
  }

  (void)dbg_rx_try_push_isr(s_rx_byte);
  (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (s_uart == 0 || huart != s_uart) {
    return;
  }

  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);

  (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
}
