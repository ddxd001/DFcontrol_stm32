#ifndef CHASSIS_UART_BRIDGE_H
#define CHASSIS_UART_BRIDGE_H

#include <stdbool.h>

/* 调试透传：USART1（PC）与 UART5（底盘）双向字节桥接。 */

void ChassisUartBridge_Init(void);
void ChassisUartBridge_SetPassthrough(bool enable);
bool ChassisUartBridge_IsPassthrough(void);
void ChassisUartBridge_Process(void);

#endif
