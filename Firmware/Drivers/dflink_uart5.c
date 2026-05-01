/*
 * DFLink/V2.0 串口帧（UART5 → 底盘模块）。
 *
 * 线格式（与公开说明一致；若与厂家最新文档有出入，只改本文件）：
 *   [0]     帧头 0xDF
 *   [1]     目标地址
 *   [2]     本机地址
 *   [3]     数据类型 A
 *   [4]     数据类型 B
 *   [5]     LEN — 数据位 C 的字节数
 *   [6..6+LEN-1] 数据位 C
 *   [6+LEN]     帧尾 0xFD
 *   [7+LEN]     校验和低字节
 *   [8+LEN]     校验和高字节
 *
 * 校验和：从「帧头 0xDF」到「帧尾 0xFD」逐字节无符号相加，取累加和的低 16 位；
 *         线上按小端发送（例：和为 0x02B3 时先发 0xB3 再发 0x02）。ACC 两字节不参与累加。
 */
#include "dflink_uart5.h"

#include <string.h>

#define DFLINK_RX_CAP 512U

/** 1 帧头 + 目标/本机/A/B/LEN + 1 帧尾 0xFD + 2 字节 ACC */
#define DFLINK_V2_OVERHEAD 8U

/* 透传时用轮询 RXNE；为 1 时禁止 Error/Rx Cplt 里再 Receive_IT */
static uint8_t s_uart5_poll_rx_only;

static UART_HandleTypeDef *s_huart;
static uint8_t s_host_id = DFLINK_UART5_ADDR_HOST_DEFAULT;
static uint8_t s_chassis_id = DFLINK_UART5_ADDR_CHASSIS_DEFAULT;

static uint8_t s_rx_ring[DFLINK_RX_CAP];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t s_rx_byte;

static int rx_try_push_isr(uint8_t b)
{
  uint16_t h;
  uint16_t next_h;

  h = s_rx_head;
  next_h = (uint16_t)((h + 1U) % DFLINK_RX_CAP);
  if (next_h == s_rx_tail) {
    return 0;
  }
  s_rx_ring[h] = b;
  s_rx_head = next_h;
  return 1;
}

static size_t build_frame_v2(uint8_t *out, size_t out_cap, uint8_t type_a,
                             uint8_t type_b, const uint8_t *c, uint8_t c_len)
{
  size_t c_end;
  uint32_t acc;
  size_t i;
  uint16_t sum16;

  if (out == 0 || c_len > DFLINK_UART5_MAX_PAYLOAD) {
    return 0U;
  }
  if (c_len > 0U && c == 0) {
    return 0U;
  }

  if (out_cap < (DFLINK_V2_OVERHEAD + (size_t)c_len)) {
    return 0U;
  }

  out[0] = DFLINK_V2_HEADER;
  out[1] = s_chassis_id;
  out[2] = s_host_id;
  out[3] = type_a;
  out[4] = type_b;
  out[5] = c_len;
  if (c_len > 0U) {
    (void)memcpy(out + 6U, c, (size_t)c_len);
  }

  c_end = 6U + (size_t)c_len;
  out[c_end] = DFLINK_V2_TAIL;

  acc = 0U;
  for (i = 0U; i <= c_end; i++) {
    acc += (uint32_t)out[i];
  }
  sum16 = (uint16_t)(acc & 0xFFFFU);

  out[c_end + 1U] = (uint8_t)(sum16 & 0xFFU);
  out[c_end + 2U] = (uint8_t)((sum16 >> 8) & 0xFFU);

  return c_end + 3U;
}

void DflinkUart5_Init(UART_HandleTypeDef *huart)
{
  s_uart5_poll_rx_only = 0U;
  s_huart = huart;
  s_rx_head = 0U;
  s_rx_tail = 0U;
}

void DflinkUart5_SetAddrs(uint8_t host_id, uint8_t chassis_id)
{
  s_host_id = host_id;
  s_chassis_id = chassis_id;
}

void DflinkUart5_StartRx(void)
{
  if (s_huart == 0) {
    return;
  }

  s_uart5_poll_rx_only = 0U;

  HAL_NVIC_SetPriority(UART5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);

  if (HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U) != HAL_OK) {
    /* 调用方可根据返回值或后续 RxAvail 判断 */
  }
}

void DflinkUart5_StopRxIt(void)
{
  if (s_huart == 0) {
    return;
  }

  s_uart5_poll_rx_only = 1U;
  /* 关掉 IT 接收，避免透传 poll 与同口 Receive_IT 双读 DR */
  (void)HAL_UART_AbortReceive_IT(s_huart);
}

HAL_StatusTypeDef DflinkUart5_SendFrame(uint8_t type_a, uint8_t type_b,
                                        const uint8_t *payload_c, uint8_t c_len,
                                        uint32_t tout_ms)
{
  uint8_t wire[DFLINK_V2_OVERHEAD + DFLINK_UART5_MAX_PAYLOAD];
  size_t n;

  if (s_huart == 0) {
    return HAL_ERROR;
  }

  n = build_frame_v2(wire, sizeof(wire), type_a, type_b, payload_c, c_len);
  if (n == 0U) {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(s_huart, wire, (uint16_t)n, tout_ms);
}

uint32_t DflinkUart5_RxAvail(void)
{
  uint16_t h;
  uint16_t t;

  h = s_rx_head;
  t = s_rx_tail;
  return (uint32_t)((DFLINK_RX_CAP + h - t) % DFLINK_RX_CAP);
}

uint32_t DflinkUart5_ReadBytes(uint8_t *buf, uint32_t maxlen)
{
  uint32_t n;

  if (buf == 0 || maxlen == 0U) {
    return 0U;
  }

  n = 0U;
  while (n < maxlen && s_rx_tail != s_rx_head) {
    buf[n] = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % DFLINK_RX_CAP);
    n++;
  }
  return n;
}

void DflinkUart5_HAL_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (s_uart5_poll_rx_only != 0U) {
    return;
  }

  if (s_huart == 0 || huart != s_huart) {
    return;
  }

  (void)rx_try_push_isr(s_rx_byte);
  if (HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U) != HAL_OK) {
    /* 与透传阻塞发送等瞬时竞争时可能失败；再补一次以减少丢链概率 */
    (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U);
  }
}

void DflinkUart5_HAL_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (s_uart5_poll_rx_only != 0U) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    return;
  }

  if (s_huart == 0 || huart != s_huart) {
    return;
  }

  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);

  (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U);
}
