#ifndef FW_CONFIG_H
#define FW_CONFIG_H

/* Project-wide constants ( CubeMX re-generates Core; keep tuning here ). */
#define FW_SCHED_MAX_TASKS       12U
#define FW_FIRMWARE_NAME         "DFcontrol_stm32"

/* 底盘调试：USART1 ↔ UART5 透传（见 chassis_uart_bridge） */
#ifndef FW_CHASSIS_UART_BRIDGE_DEFAULT
#define FW_CHASSIS_UART_BRIDGE_DEFAULT  0 /* 0=默认关透传；1=上电即 USART1↔UART5 透传 */
#endif
#define FW_CHASSIS_BRIDGE_TX_TIMEOUT_MS   100U

#endif
