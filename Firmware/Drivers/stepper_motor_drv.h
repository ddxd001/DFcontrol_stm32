#ifndef STEPPER_MOTOR_DRV_H
#define STEPPER_MOTOR_DRV_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#define STEPPER_DRV_DEFAULT_TIMEOUT_MS 50U
#define STEPPER_DRV_DEFAULT_SPEED      1000
#define STEPPER_DRV_DEFAULT_ACCEL      0U

typedef enum {
  STEPPER_DIR_CW = 0x00U,
  STEPPER_DIR_CCW = 0x01U
} StepperDrvDir;

typedef enum {
  STEPPER_RETURN_NEAREST = 0x00U,
  STEPPER_RETURN_DIR = 0x01U,
  STEPPER_RETURN_SENSORLESS = 0x02U,
  STEPPER_RETURN_ENDSTOP = 0x03U
} StepperDrvReturnMode;

typedef enum {
  STEPPER_POS_RELATIVE = 0x00U,
  STEPPER_POS_ABSOLUTE = 0x01U
} StepperDrvPosMode;

typedef struct {
  UART_HandleTypeDef *huart;
  uint8_t id;
  int32_t speed;
  uint8_t accelerator;
  int32_t pulse;
  StepperDrvPosMode pos_mode;
  StepperDrvReturnMode return_mode;
  uint32_t tx_timeout_ms;
} StepperMotorDrv;

void StepperMotorDrv_Init(StepperMotorDrv *drv, UART_HandleTypeDef *huart, uint8_t id);

HAL_StatusTypeDef StepperMotorDrv_Enable(const StepperMotorDrv *drv);
HAL_StatusTypeDef StepperMotorDrv_Disable(const StepperMotorDrv *drv);
HAL_StatusTypeDef StepperMotorDrv_RunPosition(const StepperMotorDrv *drv);
HAL_StatusTypeDef StepperMotorDrv_RunSpeed(const StepperMotorDrv *drv);
HAL_StatusTypeDef StepperMotorDrv_Stop(const StepperMotorDrv *drv);
HAL_StatusTypeDef StepperMotorDrv_ReturnZero(const StepperMotorDrv *drv);
HAL_StatusTypeDef StepperMotorDrv_SetZeroPoint(const StepperMotorDrv *drv);
HAL_StatusTypeDef StepperMotorDrv_ClearState(const StepperMotorDrv *drv);
HAL_StatusTypeDef StepperMotorDrv_TriggerCalibration(const StepperMotorDrv *drv);
HAL_StatusTypeDef StepperMotorDrv_RestoreDefault(const StepperMotorDrv *drv);

#endif
