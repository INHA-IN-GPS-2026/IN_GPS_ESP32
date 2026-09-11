#pragma once
#include <stdint.h>
uint32_t test_ticks(void);
#define SET_PERI_REG_MASK(reg, mask) ((void)0)
#define READ_PERI_REG(reg) test_ticks()
