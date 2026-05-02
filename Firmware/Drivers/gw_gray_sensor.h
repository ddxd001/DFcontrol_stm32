#ifndef GW_GRAY_SENSOR_H
#define GW_GRAY_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "i2c.h"
#include "gw_gray_regs.h"

/* 等价于 TI 工程中 GRAYSENSOR + GraySensor_*，使用 HAL Mem 接口访问寄存器。
 * STM32 HAL DevAddress：7-bit 地址左移 1 位，本驱动内部已处理。
 */

typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint8_t addr_7bit;
  uint32_t timeout_ms;

  uint8_t digital_raw;
  uint8_t digital_inv; /* ~digital_raw ，与 TI update_graySensor 一致 */
} GwGraySensor;

void GwGraySensor_InitDefaults(GwGraySensor *s, I2C_HandleTypeDef *hi2c);

/* 上电后轮询 Ping，若在 max_wait_ms 内成功返回 true（参考 GraySensor_Init 阻塞逻辑） */
bool GwGraySensor_InitPingWait(GwGraySensor *s, uint32_t max_wait_ms);

bool GwGray_Ping(GwGraySensor *s);

/* 读取 0xDD 数字量并刷新 digital_raw / digital_inv（inv 与原工程一样按位取反） */
bool GwGray_ReadDigitalUpdate(GwGraySensor *s);

bool GwGray_ReadAnalog8(GwGraySensor *s, uint8_t out8[8]);
bool GwGray_ReadNormalized8(GwGraySensor *s, uint8_t out8[8]);
bool GwGray_ReadSingleAnalog(GwGraySensor *s, uint8_t channel_1_to_8,
                             uint8_t *out);

/* 与原 IIC_Get_Offset 相同的 16-bit 组装：dat[0] | (dat[1]<<8) */
bool GwGray_ReadLineOffsetU16(GwGraySensor *s, uint16_t *out);

bool GwGray_MemWriteByte(GwGraySensor *s, uint8_t reg, uint8_t val);
bool GwGray_ChannelEnableMask(GwGraySensor *s, uint8_t mask);

/*
 * 十字路口检测（基于 digital_inv 位图）：
 * - digital_inv 的 bit=1 表示该路灰度检测到线；
 * - 判定条件：左区(bit0~2)有线 + 中区(bit3~4)有线 + 右区(bit5~7)有线，
 *   且总命中路数 >= min_active_bits。
 * 推荐 min_active_bits=5（可按现场线宽调参）。
 */
bool GwGray_IsCrossByDigitalInv(uint8_t digital_inv, uint8_t min_active_bits);

/* 先刷新 digital_raw/digital_inv，再执行十字检测。 */
bool GwGray_ReadAndDetectCross(GwGraySensor *s, uint8_t min_active_bits, bool *out_cross);

#endif
