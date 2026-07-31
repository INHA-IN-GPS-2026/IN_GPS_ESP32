#pragma once

#include <stdint.h>

/* ADXL335 가속도 센서 캘리브레이션 (3.3V 공급, 12-bit ADC) */
// #define ADXL335_ZERO_X   1920.01f
// #define ADXL335_ZERO_Y   1860.66f
// #define ADXL335_ZERO_Z   1963.66f
#define ADXL335_SENS_X    406.845f
#define ADXL335_SENS_Y    407.095f
#define ADXL335_SENS_Z    399.405f


//ADXL335 가속도 센서 캘리브레이션 (3.3V 공급, 12-bit ADC)
// 전달 사항 PCB 기판 뜨고 ADXL 방향이 정방향인지 역방향인지 어떤 방향인지 확인하고 계산 필요

/* NTC 서미스터 (VCC → NTC → ADC → R_pulldown → GND) */
#define THERMISTOR_R_PULLDOWN   10000.0f
#define THERMISTOR_R0           10000.0f
#define THERMISTOR_T0           298.15f
/* 데이터시트: B25/100 = 3492 (Rt/R25: 25°C=1, 35°C=0.68954, 45°C=0.48525) */
#define THERMISTOR_BETA         3492.0f
#define ADC_REF_VOLTAGE_MV      3300
#define ADC_MAX_RAW             4095.0f

/* Steinhart-Hart 계수: 1/T = A + B·ln(R) + C·(ln R)^3
   데이터시트 8307 특성(B25/100=3492K, R25=10k)의 Rt/R25 표 15~75°C 13점을
   전 구간 최소제곱(LSQ) 피팅. 잔차 피크 0.6 mC = 목표 0.1°C의 1/160.
   → 단일 3계수로 15~75 전 구간 커버, 구간별 계수 스위칭(밴딩) 불필요.
   (기존 3점(25/35/45) 계수는 끝단에서 ~1.8mC로 더 컸음.) */
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
 * @brief 보정된 ADC 핀 전압(mV) → 써미스터 저항(Ω, float).
 *        분압(VCC→NTC→ADC→R_pulldown→GND): r = R_pulldown·(Vcc−Vadc)/Vadc.
 *        Vadc는 eFuse curve fitting으로 INL 보정된 실전압을 넣어야 정확하다
 *        (adc_cal_raw_to_mv 참조). Vcc(분압 상단)는 ADC_REF_VOLTAGE_MV.
 */
float mv_to_resistance(float v_adc_mv);

/**
 * @brief NTC raw ADC 값 → 써미스터 저항(Ω, float). raw를 선형(raw/4095·Vref)으로
 *        환산 후 분압 역산. INL 미보정 경로(폴백/하위호환). 정확도가 필요하면
 *        adc_cal_raw_to_mv() + mv_to_resistance()를 사용할 것.
 */
float raw_to_resistance(uint16_t raw);

/**
 * @brief 저항(Ω) → 온도(x100, int16). Beta 모델: 1/T = 1/T0 + ln(R/R0)/BETA.
 */
int16_t resistance_to_temp_beta_x100(float r_ntc);

/**
 * @brief 저항(Ω) → 온도(x100, int16). Steinhart-Hart: 1/T = A + B·lnR + C·(lnR)^3.
 */
int16_t resistance_to_temp_steinhart_x100(float r_ntc);

/**
 * @brief NTC raw ADC 값을 온도(x100, int16)로 변환(Beta 모델, 하위호환). 예: 25.50°C → 2550.
 */
int16_t raw_to_temp_x100(uint16_t raw);
