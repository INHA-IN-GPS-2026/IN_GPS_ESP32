// sensor/as6221.h — ams-OSRAM AS6221 디지털 온도센서 (I2C) 드라이버.
//
// 근거: AS6221 Datasheet DS000751 v5-00 (2025-May-22).
//   · 16-bit, 1 LSB = 1/128 °C = 0.0078125 °C, 2의 보수
//   · 정확도 ±0.09°C(20~42°C) / ±0.1°C(-25~55°C) / ±0.12°C(-40~70°C)
//   · 동작범위 -40~125°C, VDD 1.71~3.6V
//   · 소비전류 4 conv/s에서 typ 6µA, 스탠바이 typ 0.1µA (버스 유휴 시)
//   · 변환시간 typ 36ms (max 51ms)
//
// 아날로그 버전(NTC 분압 + eFuse INL 보정 + Steinhart-Hart 피팅)이 겨우 도달한
// 0.1°C를 이 부품은 무보정으로 낸다. 그래서 이 브랜치에서 adc_cal / mv_to_resistance /
// resistance_to_temp_steinhart_x100 체인이 통째로 사라졌다.
// 덤으로 NTC 분압이 상시 소모하던 ~330µA도 같이 사라진다(2개 합쳐 ~4µA 수준).
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 채널 인덱스 — mfg_data의 temp1/temp2에 그대로 대응한다. */
#define AS6221_CH_TH1   0
#define AS6221_CH_TH2   1
#define AS6221_CH_COUNT 2

/* 읽기 실패 시 돌려주는 센티넬 (°C×100).
   -327.68°C는 서버의 유효 온도범위(mqtt_subscriber.py: -20 ~ 80°C) 밖이라
   temperature_log에 NULL로 저장되고 집계에서 자동 제외된다 — 아날로그 버전에서
   써미스터 개방/단락이 처리되던 방식과 같은 경로를 탄다.
   ⚠ 이 값을 adv_manager_update()에 넣지 말 것. ΔT 승격이 오작동한다. */
#define AS6221_TEMP_INVALID_X100  ((int16_t)-32768)

/**
 * @brief 공용 I2C 버스(GPIO8/9)에 AS6221 2개 등록 + 설정 기록.
 *
 * rev 4.0(I2C_final_ver)에서 TH 전용 버스가 사라지고 ADXL345/BQ35100과
 * 한 버스를 쓴다. 버스 생성은 sensor/i2c_bus.c가 담당하므로
 * ingps_i2c_bus_init() 성공 이후에 호출할 것.
 *
 * 주소 결정 순서:
 *   1) NVS 네임스페이스 "as6221"의 u8 키 "addr1"/"addr2"
 *   2) 컴파일 기본값 AS6221_DEF_ADDR_TH1 / _TH2 (0x48 / 0x49)
 *   3) 위 주소가 응답하지 않으면, 부팅 시 0x44~0x4B를 스캔해 응답한 순서대로 배정
 *
 * 3)이 있는 이유: 센서는 J6 커넥터 바깥의 별도 모듈에 있어 ADD0/ADD1 배선이
 * 스키매틱에 없다. 실물 주소를 모른 채 첫 브링업을 해야 하므로, 틀린 주소로
 * 조용히 죽는 대신 스캔 결과를 로그로 남기고 붙여 본다.
 * 확정되면 NVS knob이나 AS6221_DEF_ADDR_* 로 고정할 것.
 *
 * @return 최소 1개 센서가 응답하면 true. 버스 생성 실패 또는 전멸이면 false.
 *         false여도 앱은 계속 부팅한다(온도만 NULL, 진동은 정상).
 */
bool as6221_init(void);

/**
 * @brief 온도 1채널 읽기.
 * @param ch        AS6221_CH_TH1 / AS6221_CH_TH2
 * @param out_x100  성공 시 °C×100 (예: 25.50°C → 2550).
 *                  실패 시 AS6221_TEMP_INVALID_X100.
 * @return 성공 여부.
 *
 * 연속 실패한 채널은 AS6221_RETRY_AFTER_US 동안 버스 접근을 건너뛴다 —
 * 센서가 빠졌을 때 매 광고 사이클마다 I2C 타임아웃을 두 번씩 무는 것을 막는다.
 */
bool as6221_read_x100(int ch, int16_t *out_x100);

/** @return 해당 채널에 배정된 7-bit 주소 (미배정이면 0). 진단·로그용. */
uint8_t as6221_addr(int ch);

/** @return 마지막 접근에서 살아 있던 채널인지. 진단·로그용. */
bool as6221_present(int ch);
