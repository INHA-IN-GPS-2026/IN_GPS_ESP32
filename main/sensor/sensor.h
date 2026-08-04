#pragma once

#include <stdint.h>

/* ADXL335 감도 (3.3V 공급, 12-bit ADC, counts/g).
   ⚠ PCB 실장 시 ADXL 방향(정/역)을 확인하고 재산출할 것. */
#define ADXL335_SENS_X    406.845f
#define ADXL335_SENS_Y    407.095f
#define ADXL335_SENS_Z    399.405f

/* NTC 서미스터 분압: VCC → NTC → ADC → R_pulldown → GND */
#define THERMISTOR_R_PULLDOWN   10000.0f
#define ADC_REF_VOLTAGE_MV      3300
#define ADC_MAX_RAW             4095.0f

/* Steinhart-Hart 계수: 1/T = A + B·ln(R) + C·(ln R)^3
   데이터시트 8307 특성(B25/100=3492K, R25=10k)의 Rt/R25 표 15~75°C 13점을
   전 구간 최소제곱 피팅. 잔차 피크 0.6 mC = 목표 0.1°C의 1/160.
   → 단일 3계수로 15~75°C 전 구간 커버, 구간별 계수 스위칭(밴딩) 불필요. */
#define STEINHART_A             8.4781767957e-04f
#define STEINHART_B             2.6110639311e-04f
#define STEINHART_C             1.2967307160e-07f

/**
 * @brief 누적 sum_sq, dx 합(sum_dx), 샘플 수로부터 RMS 진동을 mg 단위로 변환.
 *        윈도우 평균(DC)을 빼고 분산으로 계산하므로 자세/기울기에 독립적이다:
 *          var = sum_sq/n - (sum_dx/n)^2,  rms = sqrt(var).
 *        N=0이면 0 반환.
 */
uint16_t accel_rms_to_mg(uint32_t sum_sq, int32_t sum_dx, uint32_t n, float sens);

/**
 * @brief 보정된 ADC 핀 전압(mV) → 써미스터 저항(Ω).
 *        r = R_pulldown·(Vcc−Vadc)/Vadc.
 *        Vadc는 eFuse curve fitting으로 INL 보정된 실전압을 넣어야 정확하다
 *        (adc_cal_raw_frac_to_mv 참조).
 *        ⚠ Vcc(분압 상단)를 고정 ADC_REF_VOLTAGE_MV로 가정한다. 분압 상단이
 *          전지 레일이면 전지 전압이 처질수록 오차가 커진다(ratiometric 미구현).
 */
float mv_to_resistance(float v_adc_mv);

/**
 * @brief 저항(Ω) → 온도(x100, int16). Steinhart-Hart. 예: 25.50°C → 2550.
 */
int16_t resistance_to_temp_steinhart_x100(float r_ntc);
