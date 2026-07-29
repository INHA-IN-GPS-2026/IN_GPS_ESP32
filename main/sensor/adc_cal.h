#pragma once

#include <stdint.h>
#include <stdbool.h>

/* =====================================================================
   ADC INL 보정 (per-chip eFuse curve fitting)
   ---------------------------------------------------------------------
   온도채널의 지배 오차는 ESP32-S3 ADC의 비선형(INL, S-curve)이다
   (저항치환 실측: 미보정 시 최대 ~1.7°C, 노이즈의 40~b70배).
   INL은 per-chip이라 직접 뽑은 곡선은 공유 불가 → Espressif가 공장에서
   칩마다 특성화해 eFuse에 구운 curve fitting 계수를 그대로 사용한다.

   ULP(RISC-V)는 adc_cali API를 호출할 수 없으므로, ULP는 raw만 누적하고
   보정은 여기(메인 CPU)에서 raw→mV 변환 시점에 적용한다. 이것이
   "raw 버퍼링 → wake/메인에서 per-chip 보정"(option 1) 구조이며,
   ULP가 뱉은 raw에 그 칩의 eFuse 곡선을 적용하므로 per-chip 정답이다.
   Deep sleep이어도 동일: ULP가 raw 누적 → 깨어난 메인이 이 함수로 보정.
   ===================================================================== */

/**
 * @brief per-chip curve fitting 캘리브레이션 핸들 초기화(1회 호출).
 *        ADC_UNIT_1 / ATTEN_DB_12 / 12-bit 기준(ULP 설정과 일치).
 * @return true  = eFuse 곡선 사용 가능(per-chip INL 보정 ON)
 *         false = eFuse 미소성/미지원 → 선형 폴백으로 자동 동작
 */
bool adc_cal_init(void);

/** @return 현재 per-chip 곡선 보정이 활성인지 여부. */
bool adc_cal_is_enabled(void);

/**
 * @brief raw ADC 코드 → 보정된 ADC 핀 전압(mV).
 *        곡선 핸들이 있으면 eFuse curve fitting(INL 보정) 적용,
 *        없으면 선형 환산(raw/4095·Vref)으로 폴백.
 */
float adc_cal_raw_to_mv(uint16_t raw);
