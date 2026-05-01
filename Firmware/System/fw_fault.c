#include "fw_fault.h"

static uint32_t s_fault_code;

void Fault_Init(void)
{
  s_fault_code = 0U;
}

void Fault_SetCode(uint32_t code)
{
  s_fault_code = code;
}
