#include "stepper_motor_drv.h"

static HAL_StatusTypeDef stepper_tx(const StepperMotorDrv *drv, const uint8_t *buf,
                                    uint16_t len)
{
  if (drv == NULL || drv->huart == NULL || buf == NULL || len == 0U) {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(drv->huart, (uint8_t *)buf, len, drv->tx_timeout_ms);
}

void StepperMotorDrv_Init(StepperMotorDrv *drv, UART_HandleTypeDef *huart, uint8_t id)
{
  if (drv == NULL) {
    return;
  }

  drv->huart = huart;
  drv->id = id;
  drv->speed = STEPPER_DRV_DEFAULT_SPEED;
  drv->accelerator = STEPPER_DRV_DEFAULT_ACCEL;
  drv->pulse = 0;
  drv->pos_mode = STEPPER_POS_ABSOLUTE;
  drv->return_mode = STEPPER_RETURN_NEAREST;
  drv->tx_timeout_ms = STEPPER_DRV_DEFAULT_TIMEOUT_MS;
}

HAL_StatusTypeDef StepperMotorDrv_Enable(const StepperMotorDrv *drv)
{
  uint8_t cmd[6];

  if (drv == NULL) {
    return HAL_ERROR;
  }

  cmd[0] = drv->id;
  cmd[1] = 0xF3U;
  cmd[2] = 0xABU;
  cmd[3] = 0x01U;
  cmd[4] = 0x00U;
  cmd[5] = 0x6BU;
  return stepper_tx(drv, cmd, sizeof(cmd));
}

HAL_StatusTypeDef StepperMotorDrv_Disable(const StepperMotorDrv *drv)
{
  uint8_t cmd[6];

  if (drv == NULL) {
    return HAL_ERROR;
  }

  cmd[0] = drv->id;
  cmd[1] = 0xF3U;
  cmd[2] = 0xABU;
  cmd[3] = 0x00U;
  cmd[4] = 0x00U;
  cmd[5] = 0x6BU;
  return stepper_tx(drv, cmd, sizeof(cmd));
}

HAL_StatusTypeDef StepperMotorDrv_RunPosition(const StepperMotorDrv *drv)
{
  int32_t pulse;
  uint32_t pulse_abs;
  StepperDrvDir dir;
  uint16_t speed_abs;
  uint8_t cmd[13];

  if (drv == NULL) {
    return HAL_ERROR;
  }

  pulse = drv->pulse;
  if (pulse >= 0) {
    dir = STEPPER_DIR_CCW;
    pulse_abs = (uint32_t)pulse;
  } else {
    dir = STEPPER_DIR_CW;
    pulse_abs = (uint32_t)(-pulse);
  }

  speed_abs = (uint16_t)((drv->speed >= 0) ? drv->speed : -drv->speed);

  cmd[0] = drv->id;
  cmd[1] = 0xFDU;
  cmd[2] = (uint8_t)dir;
  cmd[3] = (uint8_t)((speed_abs >> 8) & 0xFFU);
  cmd[4] = (uint8_t)(speed_abs & 0xFFU);
  cmd[5] = drv->accelerator;
  cmd[6] = (uint8_t)((pulse_abs >> 24) & 0xFFU);
  cmd[7] = (uint8_t)((pulse_abs >> 16) & 0xFFU);
  cmd[8] = (uint8_t)((pulse_abs >> 8) & 0xFFU);
  cmd[9] = (uint8_t)(pulse_abs & 0xFFU);
  cmd[10] = (uint8_t)drv->pos_mode;
  cmd[11] = 0x00U;
  cmd[12] = 0x6BU;
  return stepper_tx(drv, cmd, sizeof(cmd));
}

HAL_StatusTypeDef StepperMotorDrv_RunSpeed(const StepperMotorDrv *drv)
{
  int32_t speed;
  uint16_t speed_abs;
  StepperDrvDir dir;
  uint8_t cmd[8];

  if (drv == NULL) {
    return HAL_ERROR;
  }

  speed = drv->speed;
  if (speed >= 0) {
    dir = STEPPER_DIR_CCW;
    speed_abs = (uint16_t)speed;
  } else {
    dir = STEPPER_DIR_CW;
    speed_abs = (uint16_t)(-speed);
  }

  cmd[0] = drv->id;
  cmd[1] = 0xF6U;
  cmd[2] = (uint8_t)dir;
  cmd[3] = (uint8_t)((speed_abs >> 8) & 0xFFU);
  cmd[4] = (uint8_t)(speed_abs & 0xFFU);
  cmd[5] = drv->accelerator;
  cmd[6] = 0x00U;
  cmd[7] = 0x6BU;
  return stepper_tx(drv, cmd, sizeof(cmd));
}

HAL_StatusTypeDef StepperMotorDrv_Stop(const StepperMotorDrv *drv)
{
  uint8_t cmd[5];

  if (drv == NULL) {
    return HAL_ERROR;
  }

  cmd[0] = drv->id;
  cmd[1] = 0xFEU;
  cmd[2] = 0x98U;
  cmd[3] = 0x00U;
  cmd[4] = 0x6BU;
  return stepper_tx(drv, cmd, sizeof(cmd));
}

HAL_StatusTypeDef StepperMotorDrv_ReturnZero(const StepperMotorDrv *drv)
{
  uint8_t cmd[5];

  if (drv == NULL) {
    return HAL_ERROR;
  }

  cmd[0] = drv->id;
  cmd[1] = 0x9AU;
  cmd[2] = (uint8_t)drv->return_mode;
  cmd[3] = 0x00U;
  cmd[4] = 0x6BU;
  return stepper_tx(drv, cmd, sizeof(cmd));
}

HAL_StatusTypeDef StepperMotorDrv_SetZeroPoint(const StepperMotorDrv *drv)
{
  uint8_t cmd[5];

  if (drv == NULL) {
    return HAL_ERROR;
  }

  cmd[0] = drv->id;
  cmd[1] = 0x93U;
  cmd[2] = 0x88U;
  cmd[3] = 0x01U;
  cmd[4] = 0x6BU;
  return stepper_tx(drv, cmd, sizeof(cmd));
}

HAL_StatusTypeDef StepperMotorDrv_ClearState(const StepperMotorDrv *drv)
{
  uint8_t cmd[4];

  if (drv == NULL) {
    return HAL_ERROR;
  }

  cmd[0] = drv->id;
  cmd[1] = 0x0AU;
  cmd[2] = 0x6DU;
  cmd[3] = 0x6BU;
  return stepper_tx(drv, cmd, sizeof(cmd));
}

HAL_StatusTypeDef StepperMotorDrv_TriggerCalibration(const StepperMotorDrv *drv)
{
  uint8_t cmd[4];

  if (drv == NULL) {
    return HAL_ERROR;
  }

  cmd[0] = drv->id;
  cmd[1] = 0x06U;
  cmd[2] = 0x45U;
  cmd[3] = 0x6BU;
  return stepper_tx(drv, cmd, sizeof(cmd));
}

HAL_StatusTypeDef StepperMotorDrv_RestoreDefault(const StepperMotorDrv *drv)
{
  uint8_t cmd[4];

  if (drv == NULL) {
    return HAL_ERROR;
  }

  cmd[0] = drv->id;
  cmd[1] = 0x0FU;
  cmd[2] = 0x5FU;
  cmd[3] = 0x6BU;
  return stepper_tx(drv, cmd, sizeof(cmd));
}
