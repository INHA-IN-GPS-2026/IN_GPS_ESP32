/* watchdog/wdt_test.c — 워치독 장애 주입 구현 (2026-07-18)
   WDT_TEST_MODE == 0 이면 이 파일 전체가 빌드에서 빠진다(헤더의 no-op inline 사용). */
#include "wdt_test.h"

#if WDT_TEST_MODE != 0

#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensor/adxl345.h"

static const char *TAG = "WDT_TEST";

static uint32_t s_ticks = 0;   /* adv 루프 1초 주기로 증가 */

void wdt_test_tick(void)
{
    s_ticks++;

    if (s_ticks < WDT_TEST_TRIGGER_S) {
        if (s_ticks == 1 || (s_ticks % 10) == 0) {
            ESP_LOGW(TAG, "fault injection mode %d fires in %us",
                     WDT_TEST_MODE, (unsigned)(WDT_TEST_TRIGGER_S - s_ticks));
        }
        return;
    }
    if (s_ticks > WDT_TEST_TRIGGER_S) {
        return;   /* 발동은 1회만 (mode 2는 suppress 함수 쪽에서 지속) */
    }

    ESP_LOGW(TAG, "=== WDT_TEST_MODE %d FIRING (t=%us) ===",
             WDT_TEST_MODE, (unsigned)s_ticks);

#if WDT_TEST_MODE == 1
    /* 가속도 표본 정지: ADXL345를 standby로 내려 FIFO가 더 이상 차지 않게 한다.
       경로: 드레인 n=0 -> 3회 연속 실패 -> 60s 백오프 -> WDT_HB_ACCEL 정지
             -> L1 모니터가 "accelerometer samples stalled" 재부팅.
       ⚠ 구 ULP 정지(≤6s)보다 훨씬 느리다. 백오프(60s)를 포함한 stale 한도가
         75s이므로 재부팅까지 최대 ~80초를 기다려야 한다. */
    if (!adxl345_test_force_standby()) {
        ESP_LOGE(TAG, "standby 주입 실패 — 센서가 안 붙어 있으면 이 모드는 무의미하다");
    }
    ESP_LOGW(TAG, "ADXL345 standby — expect L1 reboot within ~80s");

#elif WDT_TEST_MODE == 2
    /* ADV 갱신 정지: 이후 heartbeat kick을 억제(광고 자체는 계속 갱신됨).
       기대: 15s 후 "BLE adv update stalled" 재부팅. */
    ESP_LOGW(TAG, "heartbeat suppressed — expect L1 reboot in ~15s");

#elif WDT_TEST_MODE == 3
    /* 태스크 무한 spin (yield 없음): adv 태스크가 feed를 멈추고 idle도 굶는다.
       기대: TWDT 8s 내 "Task watchdog got triggered" → panic → 재부팅. */
    ESP_LOGW(TAG, "entering busy spin — expect TWDT panic within 8s");
    for (;;) { }

#elif WDT_TEST_MODE == 4
    /* 인터럽트 정지 + spin.
       기대: INT WDT 300ms 내 "Interrupt wdt timeout" panic → 재부팅. */
    ESP_LOGW(TAG, "disabling interrupts — expect INT WDT panic within 300ms");
    portDISABLE_INTERRUPTS();
    for (;;) { }

#elif WDT_TEST_MODE == 5
    /* NULL 역참조 크래시.
       기대: LoadProhibited/StoreProhibited panic → 백트레이스 출력 후 즉시 재부팅
       (구 설정 PRINT_HALT였다면 여기서 영구 정지했을 것). */
    ESP_LOGW(TAG, "dereferencing NULL — expect immediate panic reboot");
    {
        volatile int *p = (volatile int *)0;
        *p = 42;
    }

#else
#error "WDT_TEST_MODE must be 0..5"
#endif
}

bool wdt_test_suppress_heartbeat(void)
{
#if WDT_TEST_MODE == 2
    return s_ticks >= WDT_TEST_TRIGGER_S;
#else
    return false;
#endif
}

#endif /* WDT_TEST_MODE != 0 */
