#pragma once

#include <stdint.h>

/* 가속도계 감도 상수는 이제 드라이버 헤더가 소유한다.
     ADXL345 (rev 4.0, I2C)  -> sensor/adxl345.h 의 ADXL345_SENS_FULL_RES (256 counts/g)
   ADXL335(아날로그 3축, GPIO5/6/7 + ULP SARADC)는 rev 4.0에서 제거됐다.
   구 감도 상수(ADXL335_SENS_X/Y/Z)와 ADC 스케일이 필요하면 digitalVer 이전
   커밋(5e8998c) 또는 Analog_1.0.0_ver 브랜치를 볼 것.

   NTC 써미스터 분압 상수와 Steinhart-Hart 계수도 이 브랜치에는 없다.
   온도는 AS6221(I2C) 디지털 센서가 담당한다 - sensor/as6221.h 참조. */

/**
 * @brief 누적 sum_sq, 원시값 합(sum), 샘플 수로부터 RMS 진동을 mg 단위로 변환.
 *        윈도우 평균(DC)을 빼고 분산으로 계산하므로 자세/기울기에 독립적이다:
 *          var = sum_sq/n - (sum/n)^2,  rms = sqrt(var).
 *        이 DC 제거가 있기 때문에 ADXL345 경로에서는 zero 캘리브레이션이
 *        아예 필요 없다(중력 1g가 그대로 들어와도 대수적으로 소거된다).
 *        N=0이면 0 반환.
 *
 * @param sum_sq  샘플 제곱합
 * @param sum     샘플 단순합 (ADXL335 시절엔 raw-zero였던 dx의 합)
 * @param n       샘플 수
 * @param sens    counts/g
 */
uint16_t accel_rms_to_mg(uint32_t sum_sq, int32_t sum, uint32_t n, float sens);
