#pragma once
#include <stdbool.h>
#include <stdint.h>
void soft_i2c_init(void);
bool soft_i2c_recover(void);
bool soft_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, unsigned len);
bool soft_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, unsigned len);
