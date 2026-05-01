#ifndef BUZZER_DRV_H
#define BUZZER_DRV_H

#include <stdint.h>

void BuzzerDrv_Init(void);

void BuzzerDrv_On(void);
void BuzzerDrv_Off(void);

/* Non-blocking: drive line high until duration elapsed; Process() clears it. Zero = stop. */
void BuzzerDrv_Beep(uint16_t duration_ms);

void BuzzerDrv_Process(void);

#endif
