#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* USART1 调试串口（Cube 默认 PA9 TX / PA10 RX，波特率见 MX_USART1_UART_Init）。*/

void DebugUart_Init(void);

/* 开机后调用：打印固件名、MCU、SYSCLK、USART1 波特率及回显提示 */
void DebugUart_PrintDeviceInfo(void);

HAL_StatusTypeDef DebugUart_Send(const uint8_t *buf, uint16_t len, uint32_t tout_ms);

/* 从 RX 缓冲尽可能读出并原样发出（适于周期任务轮询）。返回本次回显的字节数 */
uint32_t DebugUart_EchoRxToTx(uint32_t max_chunk, uint32_t tout_ms);

/* RX 环形缓冲可读字节数 */
uint32_t DebugUart_RxAvail(void);

bool DebugUart_ReadByte(uint8_t *out);

uint32_t DebugUart_ReadBytes(uint8_t *buf, uint32_t maxlen);

#endif
