/*
 * 参考 TI_BaseCar/User/Sensor/GraySensor.c（感为寄存器语义相同）。
 */
#include "gw_gray_sensor.h"

static uint16_t gw_i2c_addr8(GwGraySensor *s)
{
  return (uint16_t)(((uint16_t)s->addr_7bit) << 1);
}

static bool gw_mem_rd(GwGraySensor *s, uint8_t reg, uint8_t *buf, uint16_t len)
{
  if (s == 0 || s->hi2c == 0 || buf == 0 || len == 0U) {
    return false;
  }

  return HAL_I2C_Mem_Read(s->hi2c, gw_i2c_addr8(s), reg, I2C_MEMADD_SIZE_8BIT,
                          buf, len, (uint32_t)s->timeout_ms) == HAL_OK;
}

static bool gw_mem_wr(GwGraySensor *s, uint8_t reg, const uint8_t *buf,
                      uint16_t len)
{
  if (s == 0 || s->hi2c == 0 || buf == 0 || len == 0U) {
    return false;
  }

  return HAL_I2C_Mem_Write(s->hi2c, gw_i2c_addr8(s), reg, I2C_MEMADD_SIZE_8BIT,
                           (uint8_t *)buf, len, (uint32_t)s->timeout_ms) == HAL_OK;
}

void GwGraySensor_InitDefaults(GwGraySensor *s, I2C_HandleTypeDef *hi2c)
{
  if (s == 0 || hi2c == 0) {
    return;
  }

  s->hi2c = hi2c;
  s->addr_7bit = GW_GRAY_ADDR_DEF;
  s->timeout_ms = 80U;
  s->digital_raw = 0U;
  s->digital_inv = 0xFFU;
}

bool GwGraySensor_InitPingWait(GwGraySensor *s, uint32_t max_wait_ms)
{
  uint32_t t0;

  if (s == 0) {
    return false;
  }

  t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < max_wait_ms) {
    if (GwGray_Ping(s)) {
      return true;
    }
    HAL_Delay(1);
  }

  return false;
}

bool GwGray_Ping(GwGraySensor *s)
{
  uint8_t d;

  if (s == 0) {
    return false;
  }

  if (!gw_mem_rd(s, GW_GRAY_PING, &d, 1U)) {
    return false;
  }

  return (bool)(d == GW_GRAY_PING_OK);
}

bool GwGray_ReadDigitalUpdate(GwGraySensor *s)
{
  uint8_t d;

  if (s == 0) {
    return false;
  }

  if (!gw_mem_rd(s, GW_GRAY_DIGITAL_MODE, &d, 1U)) {
    return false;
  }

  s->digital_raw = d;
  s->digital_inv = (uint8_t)(~d);
  return true;
}

bool GwGray_ReadAnalog8(GwGraySensor *s, uint8_t out8[8])
{
  if (out8 == 0) {
    return false;
  }

  return gw_mem_rd(s, GW_GRAY_ANALOG_BASE_, out8, 8U);
}

bool GwGray_ReadNormalized8(GwGraySensor *s, uint8_t out8[8])
{
  if (out8 == 0) {
    return false;
  }

  return gw_mem_rd(s, GW_GRAY_ANALOG_NORMALIZE, out8, 8U);
}

bool GwGray_ReadSingleAnalog(GwGraySensor *s, uint8_t channel_1_to_8,
                             uint8_t *out)
{
  if (out == 0 || channel_1_to_8 < 1U || channel_1_to_8 > 8U) {
    return false;
  }

  return gw_mem_rd(s, GW_GRAY_ANALOG(channel_1_to_8), out, 1U);
}

bool GwGray_ReadLineOffsetU16(GwGraySensor *s, uint16_t *out)
{
  uint8_t raw[2];

  if (out == 0) {
    return false;
  }

  if (!gw_mem_rd(s, GW_GRAY_OFFSET_REG, raw, 2U)) {
    return false;
  }

  *out = (uint16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
  return true;
}

bool GwGray_MemWriteByte(GwGraySensor *s, uint8_t reg, uint8_t val)
{
  return gw_mem_wr(s, reg, &val, 1U);
}

bool GwGray_ChannelEnableMask(GwGraySensor *s, uint8_t mask)
{
  return GwGray_MemWriteByte(s, GW_GRAY_ANALOG_CHANNEL_ENABLE, mask);
}
