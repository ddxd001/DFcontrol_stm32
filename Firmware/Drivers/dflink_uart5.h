#ifndef DFLINK_UART5_H
#define DFLINK_UART5_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

/* 底盘模块接 UART5（PC12 TX / PD2 RX，波特率见 MX_UART5_Init）。 */

#define DFLINK_UART5_ADDR_HOST_DEFAULT    0x97U
#define DFLINK_UART5_ADDR_CHASSIS_DEFAULT 0x01U

/** DFLink/V2.0 线帧常量 */
#define DFLINK_V2_HEADER 0xDFU
#define DFLINK_V2_TAIL   0xFDU

/** 数据位 C 的最大长度（LEN 字段为 1 字节时的上限） */
#define DFLINK_UART5_MAX_PAYLOAD 64U

void DflinkUart5_Init(UART_HandleTypeDef *huart);

/** @param host_id 本机地址 @param chassis_id 目标地址（战车等） */
void DflinkUart5_SetAddrs(uint8_t host_id, uint8_t chassis_id);

/* 启动 RX 中断（每字节进环缓）；须在 MX_UART5_Init 之后调用 */
void DflinkUart5_StartRx(void);

/**
 * 停止 UART5 字节接收中断（用于串口透传改为轮询读 DR）。
 * 与 StartRx 成对使用；请勿在透传与其它驱动共用字节 IT 接收时同时使用。
 */
void DflinkUart5_StopRxIt(void);

/**
 * 按 DFLink/V2.0 组帧并发送。
 * @param type_a 数据类型 A（如 0x01 控制类）
 * @param type_b 数据类型 B（与 A 联合表示具体操作）
 * @param payload_c 数据位 C，可为 NULL 且 c_len=0
 */
HAL_StatusTypeDef DflinkUart5_SendFrame(uint8_t type_a, uint8_t type_b,
                                        const uint8_t *payload_c, uint8_t c_len,
                                        uint32_t tout_ms);

uint32_t DflinkUart5_RxAvail(void);
uint32_t DflinkUart5_ReadBytes(uint8_t *buf, uint32_t maxlen);

/* 由 HAL_UART_RxCpltCallback / HAL_UART_ErrorCallback 转发 */
void DflinkUart5_HAL_RxCpltCallback(UART_HandleTypeDef *huart);
void DflinkUart5_HAL_ErrorCallback(UART_HandleTypeDef *huart);

#endif
