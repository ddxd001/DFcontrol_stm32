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

/* 题目第一问：左转90度 -> 直走1m，循环4次 */
#ifndef FW_Q1_ENABLE
#define FW_Q1_ENABLE  1
#endif

/* 路口测距标定：M = dt_ms * 0.15（dt_ms 为相邻路口间隔 ms）；实际距离 D(cm) = 0.106986 * M - 1.633710。
 * 下发 dist_cm_x10 = round(10*D) = (160479*dt_ms - 16337100 + 500000) / 1000000（≤0 时按 0）。 */
#define APP_CROSS_DIST_LIN_A_NUM   160479L
#define APP_CROSS_DIST_LIN_B_NUM   16337100L
#define APP_CROSS_DIST_LIN_ROUND   500000L
#define APP_CROSS_DIST_LIN_DIV     1000000L

#endif
