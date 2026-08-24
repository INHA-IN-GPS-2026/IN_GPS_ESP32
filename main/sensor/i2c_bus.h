// sensor/i2c_bus.h — rev 4.0 단일 I2C 버스의 유일한 소유자.
//
// 구 리비전(I2C_init_ver)은 버스가 둘이었다: TH(GPIO13/14)와 BQ(GPIO8/9).
// 그래서 as6221.c가 자기 버스를 직접 만들어 썼다. rev 4.0(I2C_final_ver)에서
// ADXL345 · AS6221 ×2 · BQ35100이 한 버스(GPIO8/9)에 모이면서, 드라이버마다
// i2c_new_master_bus()를 부르면 두 번째 호출이 ESP_ERR_INVALID_STATE로 실패한다.
// 버스 생성을 여기로 올리고 각 드라이버는 핸들만 받아 device를 add한다.
#pragma once

#include <stdbool.h>
#include "driver/i2c_master.h"

/**
 * @brief 단일 I2C 마스터 버스 생성(멱등 — 두 번째 호출부터는 no-op).
 *        app_main에서 어떤 센서 드라이버보다 먼저 1회 호출할 것.
 * @return 성공 여부. false면 온도·진동 드라이버가 모두 붙지 못한다.
 */
bool ingps_i2c_bus_init(void);

/** @return 버스 핸들. ingps_i2c_bus_init() 성공 전에는 NULL. */
i2c_master_bus_handle_t ingps_i2c_bus(void);

/** 버스 공통 파라미터 — 드라이버들이 device 등록 시 같은 값을 써야 한다.
    100kHz인 이유는 board_pins.h의 PIN_I2C_SDA 주석 참조(외부 풀업 10kΩ). */
#define INGPS_I2C_SCL_HZ      100000

/** 트랜잭션 타임아웃(ms). 부품이 없으면 이 시간만큼 블로킹된다.
    최악: AS6221 스캔 8주소 + ADXL345 1회 = 9 × 50ms = 0.45s.
    광고 사이클(최소 1s)과 TWDT(8s) 대비 안전하다. */
#define INGPS_I2C_TMO_MS      50
