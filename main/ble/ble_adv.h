#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ============================================================
 * STM32 게이트웨이가 알아보는 보드 번호. main.c의 ESP32_NUM과 동일 의미.
 * 이 한 줄만 바꿔도 MAC 첫 옥텟과 device_id가 자동으로 ADDR_A 케이스로 따라감.
 * ============================================================ */
#define ESP32_NUM  1

#if   ESP32_NUM == 1
#define ESP_DEVICE_ID  0x01    /* main.c data_A[10] */
#elif ESP32_NUM == 2
    #define ESP_DEVICE_ID  0x03
#elif ESP32_NUM == 3
#define ESP_DEVICE_ID  0x08
#elif ESP32_NUM == 4
#define ESP_DEVICE_ID  0x0A
#else
#error "ESP32_NUM must be 1..4"
#endif

extern uint8_t g_own_addr_type;

/**
 * @brief BLE 광고 사이클 태스크 (FreeRTOS task entry). 구 상시광고 버전 — 딥슬립
 *        재설계 이후엔 app_main에서 안 씀(참고/롤백용으로 남겨둠).
 *        광고를 1회 시작하고 1초마다 mfg_data만 갱신.
 */
void adv_cycle_task(void *arg);

/* === 딥슬립 버스트 광고 (2026-07-15 추가) ===============================
   매 wake마다: mfg_data 1회 빌드 → duration_ms 동안만 광고 → NimBLE이 자동으로
   멈추고 BLE_GAP_EVENT_ADV_COMPLETE 발생 → ble_adv_wait_burst_done()이 깨어남.
   app_main이 이걸로 "광고 끝날 때까지"를 기다린 뒤 바로 딥슬립 재진입. */

/** @brief 세마포어 생성. nimble_port_init() 호출 전에 1회 호출할 것. */
void ble_adv_init(void);

/** @brief NimBLE sync 콜백(on_sync)에서 호출 — mfg_data 빌드 + 광고 시작. */
void adv_burst_start(uint32_t duration_ms);

/**
 * @brief 광고 버스트 완료(또는 timeout_ms 경과)까지 블록.
 * @return true면 정상 완료(ADV_COMPLETE), false면 timeout(안전장치 — 그래도
 *         호출자는 딥슬립으로 진행해야 함, 무한 대기 금지).
 */
bool ble_adv_wait_burst_done(uint32_t timeout_ms);
