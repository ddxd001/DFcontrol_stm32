#include "button_drv.h"

#include "gpio.h"
#include "stm32f4xx_hal.h"

#define BUTTON_DEBOUNCE_TICKS 3U

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t stable_level;
  uint8_t sample_level;
  uint8_t same_cnt;
  uint8_t press_event;
} button_state_t;

static button_state_t s_btn[BUTTON_DRV_COUNT] = {
    {GPIOD, GPIO_PIN_3, 1U, 1U, 0U, 0U},
    {GPIOD, GPIO_PIN_4, 1U, 1U, 0U, 0U},
    {GPIOD, GPIO_PIN_5, 1U, 1U, 0U, 0U},
};

static uint8_t read_level(button_state_t *b)
{
  return (HAL_GPIO_ReadPin(b->port, b->pin) == GPIO_PIN_SET) ? 1U : 0U;
}

void ButtonDrv_Init(void)
{
  uint32_t i;
  for (i = 0U; i < BUTTON_DRV_COUNT; i++) {
    uint8_t lv = read_level(&s_btn[i]);
    s_btn[i].stable_level = lv;
    s_btn[i].sample_level = lv;
    s_btn[i].same_cnt = 0U;
    s_btn[i].press_event = 0U;
  }
}

void ButtonDrv_Process(void)
{
  uint32_t i;
  for (i = 0U; i < BUTTON_DRV_COUNT; i++) {
    uint8_t lv = read_level(&s_btn[i]);

    if (lv == s_btn[i].sample_level) {
      if (s_btn[i].same_cnt < 0xFFU) {
        s_btn[i].same_cnt++;
      }
    } else {
      s_btn[i].sample_level = lv;
      s_btn[i].same_cnt = 1U;
    }

    if (s_btn[i].same_cnt >= BUTTON_DEBOUNCE_TICKS && s_btn[i].stable_level != lv) {
      s_btn[i].stable_level = lv;
      if (lv == 0U) {
        s_btn[i].press_event = 1U;
      }
    }
  }
}

bool ButtonDrv_WasPressed(ButtonDrv_Id id)
{
  if (id >= BUTTON_DRV_COUNT) {
    return false;
  }
  if (s_btn[id].press_event == 0U) {
    return false;
  }
  s_btn[id].press_event = 0U;
  return true;
}
