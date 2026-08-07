#pragma once

#include <stdint.h>

/* ADXL335 감도 (3.3V 공급, 12-bit ADC, counts/g).
   ⚠ PCB 실장 시 ADXL 방향(정/역)을 확인하고 재산출할 것. */
#define ADXL335_SENS_X    406.845f
#define ADXL335_SENS_Y    407.095f
#define ADXL335_SENS_Z    399.405f

#define ADC_REF_VOLTAGE_MV      3300
#define ADC_MAX_RAW             4095.0f

/* NTC 써미스터 분압 상수와 Steinhart-Hart 계수는 이 브랜치에서 제거했다.
   온도는 AS6221(I2C) 디지털 센서가 담당한다 — sensor/as6221.h 참조.
   아날로그 변환 체인(mv_to_resistance / resistance_to_temp_steinhart_x100 /
   adc_cal 곡선보정)이 필요하면 Analog_1.0.0_ver 브랜치를 볼 것. */

/**
 * @brief 누적 sum_sq, dx 합(sum_dx), 샘플 수로부터 RMS 진동을 mg 단위로 변환.
 *        윈도우 평균(DC)을 빼고 분산으로 계산하므로 자세/기울기에 독립적이다:
 *          var = sum_sq/n - (sum_dx/n)^2,  rms = sqrt(var).
 *        N=0이면 0 반환.
 */
uint16_t accel_rms_to_mg(uint32_t sum_sq, int32_t sum_dx, uint32_t n, float sens);
