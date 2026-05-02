#include "dflink_chassis_motion.h"

#include "dflink_uart5.h"

/* 将有符号 32 位按 DFLink 约定打包为小端字节序。 */
static void Dflink_PackI32Le(uint8_t *p4, int32_t v)
{
  uint32_t u;

  u = (uint32_t)v;
  p4[0] = (uint8_t)(u & 0xFFU);
  p4[1] = (uint8_t)((u >> 8) & 0xFFU);
  p4[2] = (uint8_t)((u >> 16) & 0xFFU);
  p4[3] = (uint8_t)((u >> 24) & 0xFFU);
}

/* 将有符号 16 位按小端打包。 */
static void Dflink_PackI16Le(uint8_t *p2, int16_t v)
{
  uint16_t u;

  u = (uint16_t)v;
  p2[0] = (uint8_t)(u & 0xFFU);
  p2[1] = (uint8_t)((u >> 8) & 0xFFU);
}

/* 将无符号 16 位按小端打包（用于自适应位移速度字段）。 */
static void Dflink_PackU16Le(uint8_t *p2, uint16_t v)
{
  p2[0] = (uint8_t)(v & 0xFFU);
  p2[1] = (uint8_t)((v >> 8) & 0xFFU);
}

HAL_StatusTypeDef DflinkChassis_SendVelDisplacement(int32_t vx_m_times21739,
                                                    int32_t vy_m_times21739,
                                                    int32_t vz_m_times21739,
                                                    int16_t rmax_mps_times100,
                                                    uint32_t tout_ms)
{
  uint8_t c[DFLINK_VEL_DISP_C_LEN];

  /* C 负载布局: [0..3]Vx [4..7]Vy [8..11]Vz [12..13]r_max (LE)。 */
  Dflink_PackI32Le(&c[0], vx_m_times21739);
  Dflink_PackI32Le(&c[4], vy_m_times21739);
  Dflink_PackI32Le(&c[8], vz_m_times21739);
  Dflink_PackI16Le(&c[12], rmax_mps_times100);

  return DflinkUart5_SendFrame(DFLINK_VEL_DISP_TYPE_A, DFLINK_VEL_DISP_TYPE_B, c,
                               DFLINK_VEL_DISP_C_LEN, tout_ms);
}

HAL_StatusTypeDef DflinkChassis_SendRotation(int16_t rx_deg_times100,
                                             int16_t ry_deg_times100,
                                             int32_t rz_deg_times10000,
                                             int16_t rmax_dps_times100,
                                             uint32_t tout_ms)
{
  uint8_t c[DFLINK_ROT_C_LEN];

  /* C 负载布局: [0..1]Rx [2..3]Ry [4..7]Rz [8..9]r_max (LE)。 */
  Dflink_PackI16Le(&c[0], rx_deg_times100);
  Dflink_PackI16Le(&c[2], ry_deg_times100);
  Dflink_PackI32Le(&c[4], rz_deg_times10000);
  Dflink_PackI16Le(&c[8], rmax_dps_times100);

  return DflinkUart5_SendFrame(DFLINK_ROT_TYPE_A, DFLINK_ROT_TYPE_B, c,
                               DFLINK_ROT_C_LEN, tout_ms);
}

HAL_StatusTypeDef DflinkChassis_SendAdaptConstPMove(int32_t px_m_times21739,
                                                    int32_t py_m_times21739,
                                                    int32_t pz_m_times21739,
                                                    int16_t speed_mps_times100,
                                                    uint32_t tout_ms)
{
  uint8_t c[DFLINK_ADAPT_MOVE_C_LEN];

  /* C 负载布局: [0..3]Px [4..7]Py [8..11]Pz [12..13]Speed (LE)。 */
  Dflink_PackI32Le(&c[0], px_m_times21739);
  Dflink_PackI32Le(&c[4], py_m_times21739);
  Dflink_PackI32Le(&c[8], pz_m_times21739);
  Dflink_PackU16Le(&c[12], (uint16_t)speed_mps_times100);

  return DflinkUart5_SendFrame(DFLINK_ADAPT_MOVE_TYPE_A, DFLINK_ADAPT_MOVE_TYPE_B, c,
                               DFLINK_ADAPT_MOVE_C_LEN, tout_ms);
}

HAL_StatusTypeDef DflinkChassis_SetHeadingLock(uint8_t enable, uint32_t tout_ms)
{
  uint8_t c[DFLINK_HEADING_LOCK_C_LEN];

  c[0] = (enable != 0U) ? 1U : 0U;
  return DflinkUart5_SendFrame(DFLINK_HEADING_LOCK_TYPE_A, DFLINK_HEADING_LOCK_TYPE_B, c,
                               DFLINK_HEADING_LOCK_C_LEN, tout_ms);
}
