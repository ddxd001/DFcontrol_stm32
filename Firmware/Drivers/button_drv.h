#ifndef BUTTON_DRV_H
#define BUTTON_DRV_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  BUTTON_DRV_PD3 = 0,
  BUTTON_DRV_PD4,
  BUTTON_DRV_PD5,
  BUTTON_DRV_COUNT
} ButtonDrv_Id;

void ButtonDrv_Init(void);
void ButtonDrv_Process(void);
bool ButtonDrv_WasPressed(ButtonDrv_Id id);

#endif
