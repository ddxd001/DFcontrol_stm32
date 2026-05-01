#ifndef DFLINK_CHASSIS_MOTION_H
#define DFLINK_CHASSIS_MOTION_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

/* sendVelDisplacement: A=0x02, B=0x64, LEN=14 */
#define DFLINK_VEL_DISP_TYPE_A 0x02U
#define DFLINK_VEL_DISP_TYPE_B 0x64U
#define DFLINK_VEL_DISP_C_LEN  14U
/* AdaptConstPMove: A=0x02, B=0x65, LEN=14 */
#define DFLINK_ADAPT_MOVE_TYPE_A 0x02U
#define DFLINK_ADAPT_MOVE_TYPE_B 0x65U
#define DFLINK_ADAPT_MOVE_C_LEN  14U
/* sendrot: A=0x02, B=0x66, LEN=10 */
#define DFLINK_ROT_TYPE_A      0x02U
#define DFLINK_ROT_TYPE_B      0x66U
#define DFLINK_ROT_C_LEN       10U

/**
 * 匀速位移 sendVelDisplacement（数据位 C 共 14 字节）
 * payload:
 *  - V_x: S32 LE, m * 21739
 *  - V_y: S32 LE, m * 21739
 *  - V_z: S32 LE, m * 21739
 *  - r_max: S16 LE, m/s * 100
 *
 * @note 与串口透传同时使用会争用 UART5；发命令前请关闭透传。
 */
HAL_StatusTypeDef DflinkChassis_SendVelDisplacement(int32_t vx_m_times21739,
                                                    int32_t vy_m_times21739,
                                                    int32_t vz_m_times21739,
                                                    int16_t rmax_mps_times100,
                                                    uint32_t tout_ms);

/** 位移米 ×21739 换算：支持整数米（可自行扩展带小数版本） */
#define DFLINK_MOVE_M_AS_UNIT21739(m_int) ((int32_t)(m_int) * 21739)

/**
 * 自适应位移 AdaptConstPMove（A=0x02, B=0x65, LEN=14）
 * payload:
 *  - Px: S32 LE, m * 21739
 *  - Py: S32 LE, m * 21739
 *  - Pz: S32 LE, m * 21739
 *  - Speed: S16 LE, m/s * 100
 */
HAL_StatusTypeDef DflinkChassis_SendAdaptConstPMove(int32_t px_m_times21739,
                                                    int32_t py_m_times21739,
                                                    int32_t pz_m_times21739,
                                                    int16_t speed_mps_times100,
                                                    uint32_t tout_ms);

#define DFLINK_MOVE_M_AS_UNIT21739_FOR_ADAPT(m_int) ((int32_t)(m_int) * 21739)

/**
 * 旋转 sendrot（数据位 C 共 10 字节）
 * payload:
 *  - r_x: S16 LE, deg * 100
 *  - r_y: S16 LE, deg * 100
 *  - r_z: S32 LE, deg * 10000
 *  - r_max: S16 LE, deg/s * 100
 */
HAL_StatusTypeDef DflinkChassis_SendRotation(int16_t rx_deg_times100,
                                             int16_t ry_deg_times100,
                                             int32_t rz_deg_times10000,
                                             int16_t rmax_dps_times100,
                                             uint32_t tout_ms);

#define DFLINK_ROT_DEG_AS_UNIT100(deg_int)      ((int16_t)((deg_int) * 100))
#define DFLINK_ROT_DEG_AS_UNIT10000(deg_int)    ((int32_t)((deg_int) * 10000))

#endif
