/*
 * 寄存器常量参考：济南感为智能 GW 八路灰度模组（与你的 TI_BaseCar/User/Sensor/gw_grayscale_sensor.h 一致）。
 * 具体以模组说明书为准。
 */
#ifndef GW_GRAY_REGS_H
#define GW_GRAY_REGS_H

#include <stdint.h>

#define GW_GRAY_ADDR_DEF        0x4CU

#define GW_GRAY_PING            0xAAU
#define GW_GRAY_PING_OK         0x66U
#define GW_GRAY_PING_RSP        GW_GRAY_PING_OK

#define GW_GRAY_DIGITAL_MODE    0xDDU

#define GW_GRAY_ANALOG_BASE_    0xB0U
#define GW_GRAY_ANALOG_MODE     (GW_GRAY_ANALOG_BASE_ + 0U)
#define GW_GRAY_ANALOG_NORMALIZE 0xCFU

#define GW_GRAY_ANALOG(n)       (GW_GRAY_ANALOG_BASE_ + (uint8_t)(n))

#define GW_GRAY_CALIBRATION_BLACK 0xD0U
#define GW_GRAY_CALIBRATION_WHITE 0xD1U

#define GW_GRAY_ANALOG_CHANNEL_ENABLE 0xCEU
#define GW_GRAY_ANALOG_CH_EN_1   (0x1U << 0)
#define GW_GRAY_ANALOG_CH_EN_2   (0x1U << 1)
#define GW_GRAY_ANALOG_CH_EN_3   (0x1U << 2)
#define GW_GRAY_ANALOG_CH_EN_4   (0x1U << 3)
#define GW_GRAY_ANALOG_CH_EN_5   (0x1U << 4)
#define GW_GRAY_ANALOG_CH_EN_6   (0x1U << 5)
#define GW_GRAY_ANALOG_CH_EN_7   (0x1U << 6)
#define GW_GRAY_ANALOG_CH_EN_8   (0x1U << 7)
#define GW_GRAY_ANALOG_CH_EN_ALL 0xFFU

#define GW_GRAY_ERROR           0xDEU
#define GW_GRAY_REBOOT          0xC0U
#define GW_GRAY_FIRMWARE        0xC1U
#define GW_GRAY_CHANGE_ADDR     0xADU

#define GW_GRAY_OFFSET_REG      0x88U

#define GW_GRAY_GET_NTH_BIT(sensor_value, nth_bit) \
  (((sensor_value) >> (((nth_bit)-1)U)) & 0x01U)

#define GW_GRAY_SEP_ALL_BIT8(sensor_value, v1, v2, v3, v4, v5, v6, v7, v8) \
  do {                                                                       \
    (v1) = GW_GRAY_GET_NTH_BIT((sensor_value), 1);                          \
    (v2) = GW_GRAY_GET_NTH_BIT((sensor_value), 2);                          \
    (v3) = GW_GRAY_GET_NTH_BIT((sensor_value), 3);                          \
    (v4) = GW_GRAY_GET_NTH_BIT((sensor_value), 4);                          \
    (v5) = GW_GRAY_GET_NTH_BIT((sensor_value), 5);                          \
    (v6) = GW_GRAY_GET_NTH_BIT((sensor_value), 6);                          \
    (v7) = GW_GRAY_GET_NTH_BIT((sensor_value), 7);                          \
    (v8) = GW_GRAY_GET_NTH_BIT((sensor_value), 8);                          \
  } while (0)

#endif
