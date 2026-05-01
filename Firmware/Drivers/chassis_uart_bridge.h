#ifndef CHASSIS_UART_BRIDGE_H
#define CHASSIS_UART_BRIDGE_H

#include <stdbool.h>

/* 调试：USART1（PC 调试串口）与 UART5（底盘 DFLink）双向字节透传。 */

void ChassisUartBridge_Init(void);

/** 启用后：USART1 RX → UART5 TX，UART5 RX → USART1 TX */
void ChassisUartBridge_SetPassthrough(bool enable);
bool ChassisUartBridge_IsPassthrough(void);

/** 在低周期任务中调用（例如 10 ms），内部按块搬运 */
void ChassisUartBridge_Process(void);

#endif
