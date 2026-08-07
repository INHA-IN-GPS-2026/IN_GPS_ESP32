/* watchdog/wdt_guard.c — 통합 워치독/자가복구 관리
 *
 * 보호 계층(아래로 갈수록 마지막 방어선, 상세는 Docs/Watchdog_설계.md):
 *   L1  앱 헬스모니터 태스크(이 파일): ULP 샘플 정지·BLE 광고 갱신 정지 등
 *       "태스크는 살아있는데 논리적으로 죽은" 상태 감지 → 사유 로그 후 재부팅.
 *   L2  Task WDT(esp_task_wdt, 8s, panic): 등록 태스크(부팅 구간의 main,
 *       adv_cycle, 모니터 자신)와 양쪽 idle이 제때 못 돌면 panic.
 *   L3  Interrupt WDT(300ms, sdkconfig): ISR/스케줄러 정지 시 panic.
 *   L4  panic handler = PRINT_REBOOT(sdkconfig 변경): 모든 panic이 재부팅으로 수렴.
 *   L5  부트로더 RTC WDT(9s, sdkconfig): 부팅 중 멈춤 방어.
 *
 * 복구 정책:
 *   - ULP 정지 복구는 "ULP만 재시작"이 아니라 전체 재부팅으로 한다. ULP 런타임
 *     재시작에는 미해결 버그가 있고, 전체 재부팅이 초기화 경로가 결정적이라 안전하다.
 *   - 비정상 리셋 후엔 RTC_NOINIT에 보관한 ADXL zero로 재캘리브를 건너뛰어
 *     (fast resume) "주기적 BLE 송신" 요구사항의 다운타임을 최소화한다.
 */
#include "wdt_guard.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_sleep.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ulp_shared.h"

static const char *TAG = "WDT_GUARD";

/* === 튜닝 파라미터 ==================================================== */
#define WDT_TWDT_TIMEOUT_MS    8000   /* Task WDT. adv 루프(1s)·부팅 단계 대비 여유 */
#define WDT_MONITOR_PERIOD_MS  2000   /* 헬스모니터 점검 주기 */
#define WDT_ULP_STALL_LIMIT    3      /* 연속 3회(=6s) total_samples 정지 → 재부팅 */
#define WDT_HB_ADV_STALE_MS    15000  /* 광고 갱신 heartbeat 정지 허용 한도 */
#define WDT_CRASHLOOP_WARN     3      /* 연속 비정상 리셋 경고 임계 */
/* crash-loop escalation: BACKOFF_N회부터 재시도 전 지수 백오프 딥슬립
   (30s×2^k, 상한 600s)으로 배터리 소모를 억제하고, SAFE_N회부터는 SAFE 모드
   (10s 최소광고, 캘리브 50ms 단축)로 "존재 알림"만 유지한다.
   ⚠ SAFE에서도 zero 캘리브를 완전히 생략하면 안 된다 — SAFE cadence 10s는
     zero=0일 때 sum_sq(uint32) 오버플로 임계(5.3s)를 넘어 RMS가 3축 모두
     0으로 나간다. app_main.c의 ADXL_CAL_MS_SAFE 주석 참조.
   정상 가동 1h 지속 시 카운터 자동 클리어. */
#define WDT_ESC_BACKOFF_N      3      /* 이 횟수부터 백오프 딥슬립 */
#define WDT_ESC_SAFE_N         8      /* 이 횟수부터 SAFE 모드 */
#define WDT_ESC_BACKOFF_BASE_S 30
#define WDT_ESC_BACKOFF_MAX_S  600
#define WDT_ESC_STABLE_CLEAR_S 3600   /* 무사고 1h → 카운터 리셋 */
/* 1 = 비정상 리셋 후 저장된 zero로 재캘리브 생략(fast resume). 끄려면 0. */
#ifndef WDT_FAST_RESUME
#define WDT_FAST_RESUME        1
#endif

/* === RTC_NOINIT 생존 데이터 ==========================================
   esp_restart()/WDT 리셋에는 살아남고, 전원 차단 시엔 소멸(→ magic 검사로
   무효 처리). 딥슬립 미사용 구조이므로 RTC slow mem은 이 용도로 비어 있음. */
#define WDT_RTC_MAGIC 0x47445722u   /* "!WDG"+1: backoff_pending 필드 추가 */
typedef struct {
    uint32_t magic;
    int16_t  zero_x, zero_y, zero_z;
    uint16_t sum;              /* zero_* 단순 체크섬 */
    uint32_t abnormal_resets;  /* 연속 비정상(WDT/panic/자가복구) 리셋 카운터 */
    uint32_t backoff_pending;  /* 1=직전 부팅이 백오프 딥슬립 진입 (wake 시 카운터 보존) */
} wdt_rtc_state_t;
static RTC_NOINIT_ATTR wdt_rtc_state_t s_rtc;

static uint16_t rtc_sum(const wdt_rtc_state_t *s)
{
    return (uint16_t)(0x5AA5u ^ (uint16_t)s->zero_x
                              ^ (uint16_t)s->zero_y
                              ^ (uint16_t)s->zero_z);
}

/* === heartbeat 슬롯 =================================================== */
typedef struct {
    volatile int64_t last_us;   /* 마지막 kick 시각(µs). 0 = 아직 arm 안 됨 */
    uint32_t stale_ms;          /* 이 시간 넘게 kick 없으면 재부팅 */
    const char *name;
} hb_slot_t;

static hb_slot_t s_hb[WDT_HB_MAX] = {
    [WDT_HB_ADV] = { .last_us = 0, .stale_ms = WDT_HB_ADV_STALE_MS,
                     .name = "BLE adv update stalled" },
};

static volatile bool s_armed = false;   /* boot_done 후 앱 헬스체크 활성화 */
static bool s_abnormal_boot  = false;   /* 이번 부팅이 비정상 리셋에서 온 것인지 */
static bool s_safe_mode      = false;   /* crash-loop 한계 초과 → SAFE 모드 */

extern volatile ulp_shared_t ulp_shared;

/* === 공용 API ========================================================= */
void wdt_guard_feed(void)
{
    esp_task_wdt_reset();
}

void wdt_guard_task_subscribe(void)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    esp_task_wdt_reset();
}

void wdt_guard_heartbeat(wdt_hb_id_t id)
{
    if (id < WDT_HB_MAX) {
        s_hb[id].last_us = esp_timer_get_time();
    }
}

bool wdt_guard_safe_mode(void)
{
    return s_safe_mode;
}

void wdt_guard_set_hb_stale(wdt_hb_id_t id, uint32_t stale_ms)
{
    /* 검증된 기본 한도(15s)보다 느슨해지는 방향만 허용 — 적응형 광고가
       cadence를 3s/10s로 늘릴 때 오탐 재부팅을 막기 위한 용도. */
    if (id < WDT_HB_MAX) {
        s_hb[id].stale_ms = stale_ms < WDT_HB_ADV_STALE_MS
                            ? WDT_HB_ADV_STALE_MS : stale_ms;
    }
}

void wdt_guard_reboot(const char *reason)
{
    ESP_LOGE(TAG, "self-recovery reboot: %s", reason ? reason : "(null)");
    /* UART 로그 플러시 시간만 주고 즉시 재부팅. esp_restart()는 SW 리셋이라
       RTC_NOINIT(s_rtc)이 살아남음 → 다음 부팅에서 fast resume 경로. */
    vTaskDelay(pdMS_TO_TICKS(150));
    esp_restart();
    while (1) { }   /* noreturn 보장용, 도달 불가 */
}

/* === 헬스모니터 태스크 =================================================
   자기 자신도 TWDT에 등록 → 모니터가 죽으면 TWDT(L2)가 잡는 이중 구조. */
static void wdt_monitor_task(void *arg)
{
    (void)arg;
    wdt_guard_task_subscribe();

    uint32_t last_total = 0;
    int      ulp_stall  = 0;
    bool     ulp_base   = false;   /* 첫 사이클은 기준값 스냅샷만 */

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(WDT_MONITOR_PERIOD_MS));
        esp_task_wdt_reset();

        if (!s_armed) {
            continue;   /* 부팅(zero 캘리브 포함) 완료 전엔 앱 체크 보류 */
        }

        /* 1) ULP 생존 확인: total_samples 전진 여부.
           ULP 코프로세서는 TWDT/INT WDT 감시 범위 밖이라 여기서만 잡힘.
           (ULP는 매 5ms 사이클마다 total_samples++ — 2s에 ~390 전진 정상.)
           INGPS_ULP_ADC_OFF 실험 빌드에서는 ULP를 의도적으로 안 띄우므로
           이 감시를 끈다 — 안 그러면 6s 만에 재부팅 루프에 빠진다. */
#if !INGPS_ULP_ADC_OFF
        uint32_t total = ulp_shared.total_samples;
        if (!ulp_base) {
            ulp_base = true;
        } else if (total == last_total) {
            ulp_stall++;
            ESP_LOGW(TAG, "ULP stalled: total_samples=%u (%d/%d)",
                     (unsigned)total, ulp_stall, WDT_ULP_STALL_LIMIT);
            if (ulp_stall >= WDT_ULP_STALL_LIMIT) {
                wdt_guard_reboot("ULP sample counter stalled");
            }
        } else {
            ulp_stall = 0;
        }
        last_total = total;
#else
        (void)last_total; (void)ulp_stall; (void)ulp_base;
#endif

        /* 2) 앱 heartbeat 신선도 확인 */
        int64_t now = esp_timer_get_time();
        for (int i = 0; i < WDT_HB_MAX; i++) {
            int64_t last = s_hb[i].last_us;
            if (last == 0) {
                continue;   /* 아직 arm 안 된 슬롯 */
            }
            if (now - last > (int64_t)s_hb[i].stale_ms * 1000) {
                wdt_guard_reboot(s_hb[i].name);
            }
        }
    }
}

/* === 초기화 / 부팅 API ================================================ */
void wdt_guard_init(void)
{
    esp_reset_reason_t rr = esp_reset_reason();

    /* 전원 인가·브라운아웃이거나 RTC 데이터가 깨졌으면 상태 초기화 */
    if (s_rtc.magic != WDT_RTC_MAGIC ||
        rr == ESP_RST_POWERON || rr == ESP_RST_BROWNOUT) {
        s_rtc.magic  = WDT_RTC_MAGIC;
        s_rtc.zero_x = 0;
        s_rtc.zero_y = 0;
        s_rtc.zero_z = 0;
        s_rtc.sum    = 0;
        s_rtc.abnormal_resets = 0;
    }

    /* 자가복구 esp_restart()는 ESP_RST_SW로 옴 — 비정상으로 분류(이 펌웨어에
       다른 SW 리셋 경로 없음). */
    s_abnormal_boot = (rr == ESP_RST_TASK_WDT || rr == ESP_RST_INT_WDT ||
                       rr == ESP_RST_WDT      || rr == ESP_RST_PANIC   ||
                       rr == ESP_RST_SW);
    if (rr == ESP_RST_DEEPSLEEP && s_rtc.backoff_pending) {
        /* 백오프 딥슬립에서 복귀한 재시도 부팅: 카운터를 보존(리셋도 증가도 X).
           이번 시도가 또 죽으면 다음 비정상 리셋에서 카운터가 이어서 증가한다. */
        s_rtc.backoff_pending = 0;
        ESP_LOGW(TAG, "backoff retry boot (abnormal_resets=%u preserved)",
                 (unsigned)s_rtc.abnormal_resets);
    } else if (s_abnormal_boot) {
        s_rtc.abnormal_resets++;
    } else {
        s_rtc.abnormal_resets = 0;
    }

    ESP_LOGI(TAG, "reset reason=%d, consecutive abnormal resets=%u",
             (int)rr, (unsigned)s_rtc.abnormal_resets);

    /* === crash-loop escalation ======================================== */
    if (s_rtc.abnormal_resets >= WDT_ESC_SAFE_N) {
        /* SAFE: 재부팅 루프를 멈추지 않되 기능 최소화(10s 광고, 캘리브 50ms).
           BLE "존재 알림"은 유지 — 회수/진단 가능성을 남긴다. */
        s_safe_mode = true;
        ESP_LOGE(TAG, "crash-loop ESCALATION: SAFE mode (%u abnormal resets)",
                 (unsigned)s_rtc.abnormal_resets);
        if (s_rtc.abnormal_resets > WDT_ESC_SAFE_N) {
            /* SAFE 부팅 자체도 반복 크래시 중 → 최대 백오프로 스로틀.
               (10분 주기 재시도 — 배터리 방전 억제가 최우선) */
            ESP_LOGE(TAG, "SAFE still crashing — throttle %us deep sleep",
                     (unsigned)WDT_ESC_BACKOFF_MAX_S);
            s_rtc.backoff_pending = 1;
            esp_sleep_enable_timer_wakeup(
                (uint64_t)WDT_ESC_BACKOFF_MAX_S * 1000000ULL);
            esp_deep_sleep_start();
        }
    } else if (s_rtc.abnormal_resets >= WDT_ESC_BACKOFF_N) {
        /* 지수 백오프: 30s×2^k (상한 600s) 딥슬립 후 재시도.
           RTC_NOINIT은 딥슬립에 살아남고, wake 경로는 위 backoff_pending으로
           식별한다. 크래시 루프 중 배터리 방전과 brownout 연쇄를 차단. */
        uint32_t k = s_rtc.abnormal_resets - WDT_ESC_BACKOFF_N;
        uint32_t sleep_s = WDT_ESC_BACKOFF_BASE_S << (k > 4 ? 4 : k);
        if (sleep_s > WDT_ESC_BACKOFF_MAX_S) sleep_s = WDT_ESC_BACKOFF_MAX_S;
        ESP_LOGE(TAG, "crash-loop ESCALATION: backoff deep sleep %us"
                 " (%u abnormal resets) — check HW/power",
                 (unsigned)sleep_s, (unsigned)s_rtc.abnormal_resets);
        s_rtc.backoff_pending = 1;
        esp_sleep_enable_timer_wakeup((uint64_t)sleep_s * 1000000ULL);
        esp_deep_sleep_start();   /* noreturn */
    }

    /* Task WDT 재설정: sdkconfig 기본(5s)을 코드에서 확정적으로 덮어씀.
       trigger_panic=true + (sdkconfig) PANIC_PRINT_REBOOT → TWDT 발화 시 재부팅.
       idle 양코어 감시 유지 — 어떤 태스크가 CPU를 독점(spin)해도 잡힌다. */
    esp_task_wdt_config_t twdt_cfg = {
        .timeout_ms     = WDT_TWDT_TIMEOUT_MS,
        .idle_core_mask = (1 << 0) | (1 << 1),
        .trigger_panic  = true,
    };
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&twdt_cfg));

    /* 부팅 구간 감시: app_main(호출 태스크)을 TWDT에 등록.
       단계 사이마다 wdt_guard_feed(), 완료 시 wdt_guard_boot_done()에서 해제. */
    wdt_guard_task_subscribe();

    /* 헬스모니터 기동. adv(5)보다 높은 prio 6 — 누가 5에서 spin해도 모니터는
       돈다(그 경우 idle 기아는 어차피 TWDT가 잡음 — 계층 중복 방어).
       Core 1 고정(크로스코어 감시): BLE 스택이 사는 Core 0이 통째로 멈춰도
       모니터는 Core 1에서 살아남아 heartbeat 미갱신을 감지·재부팅한다. */
    xTaskCreatePinnedToCore(wdt_monitor_task, "wdt_guard", 3072, NULL, 6, NULL, 1);
}

static void esc_stable_clear_cb(void *arg)
{
    (void)arg;
    if (s_rtc.abnormal_resets != 0) {
        ESP_LOGI(TAG, "stable %us — abnormal reset counter cleared (was %u)",
                 (unsigned)WDT_ESC_STABLE_CLEAR_S,
                 (unsigned)s_rtc.abnormal_resets);
        s_rtc.abnormal_resets = 0;
    }
}

void wdt_guard_boot_done(void)
{
    /* 무사고 1h 지속 시 escalation 카운터 자동 클리어(one-shot) */
    const esp_timer_create_args_t targs = {
        .callback = esc_stable_clear_cb,
        .name     = "wdt_esc_clear",
    };
    esp_timer_handle_t th;
    if (esp_timer_create(&targs, &th) == ESP_OK) {
        esp_timer_start_once(th, (uint64_t)WDT_ESC_STABLE_CLEAR_S * 1000000ULL);
    }

    /* ADV heartbeat 시계 arm: 지금부터 15s 안에 광고 갱신이 한 번도 성공하지
       못하면(예: NimBLE sync 자체가 안 옴) 모니터가 재부팅시킨다. */
    wdt_guard_heartbeat(WDT_HB_ADV);
    s_armed = true;

    /* IDF main task는 app_main 리턴 직후 자기삭제됨 — TWDT에 남겨두면 유령
       항목이 리셋을 못 해 오탐하므로 반드시 해제. */
    ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
    ESP_LOGI(TAG, "armed: TWDT=%ums(panic), monitor=%ums,"
             " ULP stall=%ux, ADV stale=%ums",
             (unsigned)WDT_TWDT_TIMEOUT_MS, (unsigned)WDT_MONITOR_PERIOD_MS,
             (unsigned)WDT_ULP_STALL_LIMIT, (unsigned)WDT_HB_ADV_STALE_MS);
}

bool wdt_guard_fast_resume(int16_t *zx, int16_t *zy, int16_t *zz)
{
#if WDT_FAST_RESUME
    if (s_abnormal_boot &&
        s_rtc.magic == WDT_RTC_MAGIC &&
        s_rtc.sum == rtc_sum(&s_rtc) &&
        !(s_rtc.zero_x == 0 && s_rtc.zero_y == 0 && s_rtc.zero_z == 0)) {
        *zx = s_rtc.zero_x;
        *zy = s_rtc.zero_y;
        *zz = s_rtc.zero_z;
        return true;
    }
#else
    (void)zx; (void)zy; (void)zz;
#endif
    return false;
}

void wdt_guard_save_zero(int16_t zx, int16_t zy, int16_t zz)
{
    s_rtc.zero_x = zx;
    s_rtc.zero_y = zy;
    s_rtc.zero_z = zz;
    s_rtc.sum    = rtc_sum(&s_rtc);
}
