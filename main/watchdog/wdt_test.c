/* watchdog/wdt_test.c — 워치독 장애 주입 구현 (2026-07-18)
   WDT_TEST_MODE == 0 이면 이 파일 전체가 빌드에서 빠진다(헤더의 no-op inline 사용). */
#include "wdt_test.h"

#if WDT_TEST_MODE != 0

#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ulp_riscv.h"

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
    /* ULP 정지: ULP 타이머를 멈춰 total_samples 전진을 중단시킨다.
       기대: L1 모니터가 2s 주기 3회(≤6s) 후 "ULP stalled" 재부팅. */
    ulp_riscv_timer_stop();
    ESP_LOGW(TAG, "ULP timer stopped — expect L1 reboot within ~6s");

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
