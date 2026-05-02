/*
 * USART1 调试口：中断接收 + 环形缓冲。
 * 同时保留 UART5 回调分发，避免影响 dflink_uart5。
 */
#include "debug_uart.h"

#include "dflink_uart5.h"
#include "usart.h"

#define DBG_RX_CAP 512U

static UART_HandleTypeDef *s_uart1;
static uint8_t s_rx_ring[DBG_RX_CAP];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t s_rx_byte;

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
  s_uart1 = &huart1;
  s_rx_head = 0U;
  s_rx_tail = 0U;
  (void)HAL_UART_Receive_IT(s_uart1, &s_rx_byte, 1U);
}

HAL_StatusTypeDef DebugUart_Send(const uint8_t *buf, uint16_t len, uint32_t tout_ms)
{
  if (buf == NULL || len == 0U || s_uart1 == NULL) {
    return HAL_ERROR;
  }
  return HAL_UART_Transmit(s_uart1, (uint8_t *)buf, len, tout_ms);
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

  if (out == NULL) {
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

  if (buf == NULL || maxlen == 0U) {
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
  if (huart == &huart5) {
    DflinkUart5_HAL_RxCpltCallback(huart);
    return;
  }

  if (s_uart1 == NULL || huart != s_uart1) {
    return;
  }

  (void)dbg_rx_try_push_isr(s_rx_byte);
  (void)HAL_UART_Receive_IT(s_uart1, &s_rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart5) {
    DflinkUart5_HAL_ErrorCallback(huart);
    return;
  }

  if (s_uart1 == NULL || huart != s_uart1) {
    return;
  }

  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  (void)HAL_UART_Receive_IT(s_uart1, &s_rx_byte, 1U);
}
