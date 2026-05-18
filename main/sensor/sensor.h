#pragma once

#include <stdint.h>

/* ADXL335 가속도 센서 캘리브레이션 (3.3V 공급, 12-bit ADC) */
#define ADXL335_ZERO_X   1920.01f
#define ADXL335_ZERO_Y   1860.66f
#define ADXL335_ZERO_Z   1963.66f
#define ADXL335_SENS_X    406.845f
#define ADXL335_SENS_Y    407.095f
#define ADXL335_SENS_Z    399.405f

/* NTC 서미스터 (VCC → NTC → ADC → R_pulldown → GND) */
#define THERMISTOR_R_PULLDOWN   10000.0f
#define THERMISTOR_R0           10000.0f
#define THERMISTOR_T0           298.15f
#define THERMISTOR_BETA         3950.0f
#define ADC_REF_VOLTAGE_MV      3300
#define ADC_MAX_RAW             4095.0f

/**
 * @brief 누적 sum_sq와 샘플 수로부터 RMS 진동을 mg 단위로 변환.
 *        N=0이면 0 반환.
 */
uint16_t accel_rms_to_mg(uint32_t sum_sq, uint32_t n, float sens);

/**
 * @brief NTC raw ADC 값을 온도(x100, int16)로 변환. 예: 25.50°C → 2550.
 */
int16_t raw_to_temp_x100(uint16_t raw);
