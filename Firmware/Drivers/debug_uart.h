#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/* USART1 中断接收 + 环形缓冲。 */

void DebugUart_Init(void);
HAL_StatusTypeDef DebugUart_Send(const uint8_t *buf, uint16_t len, uint32_t tout_ms);
uint32_t DebugUart_RxAvail(void);
bool DebugUart_ReadByte(uint8_t *out);
uint32_t DebugUart_ReadBytes(uint8_t *buf, uint32_t maxlen);

/* USART6（Linux开发板链路）：中断接收 + 环形缓冲。 */
HAL_StatusTypeDef DebugUart6_Send(const uint8_t *buf, uint16_t len, uint32_t tout_ms);
uint32_t DebugUart6_RxAvail(void);
bool DebugUart6_ReadByte(uint8_t *out);
uint32_t DebugUart6_ReadBytes(uint8_t *buf, uint32_t maxlen);

#endif
