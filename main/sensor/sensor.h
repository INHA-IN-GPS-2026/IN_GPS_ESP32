#pragma once

#include <stdint.h>

/* ADXL335 가속도센서 캘리브레이션 (3.3V 공급, 12-bit ADC) */
#define ADXL335_ZERO_X   1920.01f
#define ADXL335_ZERO_Y   1860.66f
#define ADXL335_ZERO_Z   1963.66f
#define ADXL335_SENS_X    406.845f
#define ADXL335_SENS_Y    407.095f
#define ADXL335_SENS_Z    399.405f

/* NTC 서미스터 설정 (VCC → NTC → ADC → R_pulldown → GND) */
#define THERMISTOR_R_PULLDOWN   10000.0f
#define THERMISTOR_R0           10000.0f
#define THERMISTOR_T0           298.15f
#define THERMISTOR_BETA         3981.0f
#define ADC_REF_VOLTAGE_MV      3300
#define ADC_MAX_RAW             4095.0f

/**
 * @brief NTC raw ADC 값을 온도(x100, int16)로 변환
 *        예: 25.50°C → 2550
 */
int16_t raw_to_temp_x100(uint16_t raw);

/**
 * @brief ADXL335 raw ADC 값을 각도(x100, int16)로 변환
 *        예: 45.23° → 4523
 */
void raw_to_accel_angles(int16_t rx, int16_t ry, int16_t rz,
                         int16_t *ax100, int16_t *ay100, int16_t *az100);
