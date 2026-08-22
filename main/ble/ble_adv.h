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
    #define ESP_DEVICE_ID  0x02
#elif ESP32_NUM == 3
#define ESP_DEVICE_ID  0x08
#elif ESP32_NUM == 4
#define ESP_DEVICE_ID  0x0A
#else
#error "ESP32_NUM must be 1..4"
#endif

extern uint8_t g_own_addr_type;

/* ★★ floor(라이트슬립 잔류) 전류 실측 빌드 스위치 — 여기가 유일한 정의다.
   0 = 정상 빌드, 1 = 측정 빌드(안정화 후 광고·센서읽기 정지 + 생존 토글).

   app_main.c와 ble_adv.c가 같은 값을 봐야 해서 헤더에 둔다
   (INGPS_ULP_ADC_OFF가 ulp_shared.h에 있는 것과 같은 이유). app_main.c에서
   #define 하면 ble_adv.c는 0으로 컴파일되어 반쪽짜리 빌드가 된다.

   동작 설명·안정화 시간·생존 토글 핀은 app_main.c의 knob 주석,
   측정 절차 전문은 Docs/INGPS_floor전류_측정절차_2026-08-11.md 참조.
   ⚠ 측정이 끝나면 반드시 0으로 원복할 것 — 이 빌드는 광고를 하지 않는다. */
#ifndef INGPS_FLOOR_TEST
#define INGPS_FLOOR_TEST  0
#endif

/**
 * @brief BLE 광고 사이클 태스크 (FreeRTOS task entry). 구 상시광고 버전 — 딥슬립
 *        재설계 이후엔 app_main에서 안 씀(참고/롤백용으로 남겨둠).
 *        광고를 1회 시작하고 1초마다 mfg_data만 갱신.
 */
void adv_cycle_task(void *arg);

/* === NVS 안전 커밋 지원 (2026-07-31, BOD LVL7 원복과 세트) ===============
   nvs_safe_commit()이 flash write를 TX 전류 스파이크와 시간 분리할 때 사용.
   pause 중엔 adv_cycle_task도 인터벌 변경 재시작을 보류한다. */

/** @brief 광고 일시정지. 아직 광고 시작 전이면 no-op(0 반환). @return 0=성공 */
int ble_adv_pause(void);

/** @brief 광고 재개(현재 adv_manager 인터벌 사용). pause와 짝. @return 0=성공 */
int ble_adv_resume(void);

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
