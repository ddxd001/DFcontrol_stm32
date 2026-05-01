#include "bsp.h"

#include "buzzer_drv.h"
#include "debug_uart.h"

void Bsp_Init(void)
{
  BuzzerDrv_Init();
  DebugUart_Init();
  DebugUart_PrintDeviceInfo();
}
