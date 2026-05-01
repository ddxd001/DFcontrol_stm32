#ifndef FW_FAULT_H
#define FW_FAULT_H

#include <stdint.h>

void Fault_Init(void);
void Fault_SetCode(uint32_t code);

#endif
