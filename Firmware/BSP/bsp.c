#include "bsp.h"

#include "buzzer_drv.h"
#include "chassis_uart_bridge.h"
#include "debug_uart.h"
#include "dflink_uart5.h"
#include "usart.h"

void Bsp_Init(void)
{
  BuzzerDrv_Init();
  DebugUart_Init();
  DflinkUart5_Init(&huart5);
  DflinkUart5_StartRx();
  ChassisUartBridge_Init();
}
