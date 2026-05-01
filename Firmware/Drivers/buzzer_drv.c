#include "buzzer_drv.h"
#include "stm32f4xx_hal.h"

#define BUZZER_PORT   GPIOA
#define BUZZER_PIN    GPIO_PIN_11

static uint16_t s_beep_remain_ms;

void BuzzerDrv_Init(void)
{
  s_beep_remain_ms = 0U;
  BuzzerDrv_Off();
}

void BuzzerDrv_On(void)
{
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
}

void BuzzerDrv_Off(void)
{
  s_beep_remain_ms = 0U;
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

void BuzzerDrv_Beep(uint16_t duration_ms)
{
  if (duration_ms == 0U) {
    BuzzerDrv_Off();
    return;
  }
  s_beep_remain_ms = duration_ms;
  BuzzerDrv_On();
}

void BuzzerDrv_Process(void)
{
  if (s_beep_remain_ms == 0U) {
    return;
  }
  s_beep_remain_ms--;
  if (s_beep_remain_ms == 0U) {
    BuzzerDrv_Off();
  }
}
