#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ADC 스케일 상수. rev 4.0에서 sensor.h가 RMS 수식만 남기고 정리되면서
   ADC 전용인 이 둘을 여기로 옮겼다(유일한 사용처가 adc_cal.c다). */
#define ADC_REF_VOLTAGE_MV      3300
#define ADC_MAX_RAW             4095.0f

/* =====================================================================
   ★ I2C 브랜치 주의: 이 모듈은 현재 아무도 호출하지 않는다.
      온도가 AS6221(I2C 디지털 센서)로 넘어가면서 NTC raw→mV 곡선보정이
      필요 없어졌다. 빌드에는 남겨 두었으나 adc_cal_init()을 부르는 곳은 없다.
      아래 설명은 아날로그(NTC) 시절의 설계 근거이며, 배터리 모니터링 등
      새 아날로그 채널을 붙일 때 그대로 재사용할 수 있다.
      아날로그 온도 경로 전체는 Analog_1.0.0_ver 브랜치 참조.
   =====================================================================

   ADC INL 보정 (per-chip eFuse curve fitting)
   ---------------------------------------------------------------------
   온도채널의 지배 오차는 ESP32-S3 ADC의 비선형(INL, S-curve)이다
   (저항치환 실측: 미보정 시 최대 ~1.7°C, 노이즈의 40~70배).
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

/**
 * @brief 소수 raw → 보정 mV. 창 평균이 만든 sub-LSB 정보를 보존한다.
 *
 * 왜 필요한가: IDF의 adc_cali_raw_to_voltage()는 **정수 mV**만 돌려준다.
 * 12dB 감쇠에서 1 LSB ≈ 0.76mV라, 정수 양자화(1mV)가 LSB보다 오히려 거칠다.
 * ULP 창 평균(200~2000샘플)으로 표준오차를 0.3~0.7 LSB까지 낮춰놔도 정수
 * 변환에서 그 정보가 통째로 날아간다. S-curve(INL) 보정 품질을 재려면
 * 잔차를 mV 단위 이하로 봐야 하므로 이 함수가 필요하다.
 *
 * 어떻게: raw 주변 ±ADC_CAL_INTERP_SPAN 코드의 두 점을 eFuse 곡선으로 구해
 * 선형보간한다. span은 트레이드오프다 — 작으면 1mV 양자화가 그대로 남고,
 * 크면 INL 곡선의 국소 구조가 뭉개진다. 기본 16(=32코드 폭)은 ESP32 S-curve
 * 주기(수백 코드)보다 충분히 좁으면서 보간 분해능 ~0.05mV를 준다.
 * 곡선을 날것 그대로 보고 싶으면 ADC_CAL_INTERP_SPAN=1로 빌드할 것.
 *
 * @param raw 소수부를 포함한 ADC 코드 (0 ~ 4095)
 * @return 보정된 ADC 핀 전압(mV, 소수 포함). eFuse 미가용 시 선형 폴백.
 */
float adc_cal_raw_frac_to_mv(float raw);
