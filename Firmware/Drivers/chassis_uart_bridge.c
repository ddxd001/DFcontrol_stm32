#include "chassis_uart_bridge.h"

#include "debug_uart.h"
#include "dflink_uart5.h"
#include "fw_config.h"
#include "usart.h"

#define CHASSIS_BRIDGE_CHUNK 256U
#define UART5_RXNE_BURST_MAX_PER_TICK 64U

static bool s_passthrough;

/**
 * 透传不宜用 HAL_UART_Transmit：其 __HAL_LOCK 与同一 UART 上中断里
 * HAL_UART_Receive_IT 重启竞争可能导致收包链断裂。
 * 此处仅轮询 TXE/TC 写 DR，不碰 HAL 互斥锁。
 */
static HAL_StatusTypeDef uart_poll_send(UART_HandleTypeDef *hu,
                                        const uint8_t *buf, uint16_t len,
                                        uint32_t tout_ms)
{
  uint32_t t_start;
  uint16_t i;

  if (hu == NULL || buf == NULL || len == 0U) {
    return HAL_ERROR;
  }

  t_start = HAL_GetTick();
  for (i = 0U; i < len; i++) {
    while (__HAL_UART_GET_FLAG(hu, UART_FLAG_TXE) == RESET) {
      if ((HAL_GetTick() - t_start) > tout_ms) {
        return HAL_TIMEOUT;
      }
    }
    hu->Instance->DR = (uint16_t)buf[i];
  }

  while (__HAL_UART_GET_FLAG(hu, UART_FLAG_TC) == RESET) {
    if ((HAL_GetTick() - t_start) > tout_ms) {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

static void bridge_sync_uart5_rx_listen(void)
{
  if (!s_passthrough) {
    /* 恢复环形缓冲中断收包，供 DFLink_SendFrame / ReadBytes */
    DflinkUart5_StartRx();
    return;
  }

  /*
   * 透传时必须停掉 Receive_IT：
   * 460800 + 周期任务若为 10ms 易 ORE，且环形缓冲链路易与 IRQ 交错丢字节。
   * 随后在 Process 中对 UART5 RXNE 做轮询读出再转发 USART1。
   */
  DflinkUart5_StopRxIt();
}

void ChassisUartBridge_Init(void)
{
#if FW_CHASSIS_UART_BRIDGE_DEFAULT != 0
  s_passthrough = true;
#else
  s_passthrough = false;
#endif
  bridge_sync_uart5_rx_listen();
}

void ChassisUartBridge_SetPassthrough(bool enable)
{
  s_passthrough = enable;
  bridge_sync_uart5_rx_listen();
}

bool ChassisUartBridge_IsPassthrough(void)
{
  return s_passthrough;
}

void ChassisUartBridge_Process(void)
{
  uint8_t buf[CHASSIS_BRIDGE_CHUNK];
  uint8_t b;
  uint32_t n;
  uint32_t tout;
  unsigned k;

  if (!s_passthrough) {
    return;
  }

  tout = (uint32_t)FW_CHASSIS_BRIDGE_TX_TIMEOUT_MS;

  /* UART5：轮询硬件 RX → USART1（不经过 dflink 环缓） */
  for (k = 0U; k < UART5_RXNE_BURST_MAX_PER_TICK; k++) {
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE) == RESET) {
      break;
    }
    b = (uint8_t)(huart5.Instance->DR & 0xFFU);
    (void)uart_poll_send(&huart1, &b, 1U, tout);
  }
  if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_ORE) != RESET) {
    __HAL_UART_CLEAR_OREFLAG(&huart5);
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE) != RESET) {
      (void)((uint16_t)(huart5.Instance->DR & 0xFFU));
    }
  }

  /* USART1 环缓 → UART5 TX */
  n = DebugUart_ReadBytes(buf, CHASSIS_BRIDGE_CHUNK);
  if (n > 0U) {
    (void)uart_poll_send(&huart5, buf, (uint16_t)n, tout);
  }
}
