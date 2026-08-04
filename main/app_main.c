#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include <stdio.h>
#include "nvs_flash.h"
#include "led_strip.h"

#define PIN_RGB_LED  48

/* 1이면 15초마다 light sleep 실적(진입 횟수·누적 수면시간·수면 점유율)과
   PM 락 목록을 출력한다.
     - "Light sleep: ENABLED" 로그는 esp_pm_configure()가 설정을 수락했다는 뜻일 뿐
       실제 진입을 보장하지 않는다. 진입 여부는 아래 카운터로만 확정할 수 있다.
     - 수면 점유율이 0.0%면 아예 못 자는 것 → 락 목록에서 type=NO_LIGHT_SLEEP이면서
       count>0인 항목의 소유자가 범인이다.
     - 락이 전부 0인데 점유율이 낮으면 락 문제가 아니라 wake 빈도 문제다.
   진입 횟수 계측에는 CONFIG_PM_LIGHT_SLEEP_CALLBACKS=y가 필요하다(없으면 락 목록만).
   ⚠ 전류 실측 시에는 0으로 되돌릴 것 — UART 출력 자체가 CPU를 깨우고 락을 잡는다. */
#define INGPS_PM_DEBUG  0

/* light sleep on/off 토글. 1 = 정상(1년 전력 설계의 기본값),
   0 = DFS만 쓰고 CPU 상시 on. 0은 평균 전류가 ~5배 나빠져 1년 목표가 불가능하며
   슬립/웨이크 트랜지언트를 제거한 대조군 측정 전용이다. */
#define INGPS_LIGHT_SLEEP  1

/* ★벌크캡 축전 검증 모드 (branch I_Current_test 전용).
   가설: 1000µF 벌크캡이 충분히 충전되기 전에 부하가 걸려, 첫 TX에서 레일이
   무너진다. 이를 가르려면 "부하가 거의 없는 구간"을 강제로 만들어 그때
   레일이 실제로 올라오는지 봐야 한다.

   동작:
     콜드부팅(전원 인가/브라운아웃 리셋) → 아무것도 켜지 않고 즉시 deep sleep
       → CAP_TEST_SLEEP_S 동안 축전 (ESP 자체 소모 ~7µA)
     타이머 wake → 정상 부팅(광고 시작) → CAP_TEST_ACTIVE_MS 관측
       → 다시 deep sleep → 반복

   스코프로 레일을 보면 축전 구간에서 전압이 올라오고, 관측 구간에서 TX 펄스마다
   얼마나 내려앉는지가 한 화면에 나온다.

   ⚠ 아날로그 프런트엔드(ADXL335 ~350µA + NTC 분압 ~330µA)는 전원 게이팅 회로가
     없어 deep sleep 중에도 계속 먹는다. 즉 축전 구간 실측은 7µA가 아니라
     보드 기준 ~680µA가 정상이다. 이 값이 안 나오면 회로를 먼저 의심할 것.
   ⚠ ULP가 돌면 deep sleep 중에도 샘플링을 계속하므로, 이 테스트는
     INGPS_ULP_ADC_OFF=1과 함께 써야 축전 구간이 깨끗하다. */
#define INGPS_CAP_CHARGE_TEST  0
#define CAP_TEST_SLEEP_S       30      /* 축전 구간(deep sleep) */
/* 관측 구간. ⚠ INGPS_ULP_ADC_OFF=0(실부하)이면 첫 광고까지 ~11.5s가 걸린다:
   deep sleep wake는 ESP_RST_DEEPSLEEP이라 wdt_guard가 비정상 리셋으로 보지 않고
   → fast_resume=false → 10s ADXL 캘리브가 매번 돌고 + NimBLE init 1.1s.
   이 값이 그보다 짧으면 광고가 시작되기도 전에 다시 잠들어 TX를 하나도 못 본다. */
#define CAP_TEST_ACTIVE_MS     30000   /* 관측 구간(광고 동작) */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "ulp/ulp_init.h"
#include "ulp_shared.h"
#include "ble/ble_adv.h"
#include "ble/adv_manager.h"
#include "sensor/adc_cal.h"
#include "sensor/sensor.h"
#include "watchdog/wdt_guard.h"

static const char *TAG = "APP_MAIN";

uint8_t g_own_addr_type = BLE_OWN_ADDR_RANDOM;

static const uint8_t s_ble_random_addr[6] = {
    ESP_DEVICE_ID, 0xEE, 0xDD, 0xCC, 0xBB, 0xCA
};

static void on_sync(void)
{
    int rc = ble_hs_id_set_rnd(s_ble_random_addr);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_set_rnd failed: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE random address set (wire LSB-first 01 EE DD CC BB CA, air MAC CA:BB:CC:DD:EE:01)");

    /* TX 파워 knob(advm/tx_dbm, 기본 +9dBm) 적용 — 컨트롤러 기동 후에만 유효 */
    adv_manager_apply_tx_power();

    /* Core 1 고정: BLE 컨트롤러/NimBLE 호스트(Core 0)와 경합 제거.
       광고 페이로드 빌드·온도변환이 BLE 타이밍에 영향 주지 않도록 분리. */
    xTaskCreatePinnedToCore(adv_cycle_task, "adv_cycle", 4096, NULL, 5, NULL, 1);
}

static void nimble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
    vTaskDelete(NULL);
}

#if INGPS_PM_DEBUG

#if CONFIG_PM_LIGHT_SLEEP_CALLBACKS
static volatile uint32_t s_ls_count;
static volatile int64_t  s_ls_total_us;

/* IDLE 태스크 컨텍스트에서 호출된다 — 블로킹·로깅 금지. 카운터만 만진다.
   exit 콜백의 sleep_time_us는 "실제로 잔 시간"이다(entry 콜백은 예상 시간). */
static IRAM_ATTR esp_err_t pm_ls_exit_cb(int64_t sleep_time_us, void *arg)
{
    (void)arg;
    s_ls_count++;
    s_ls_total_us += sleep_time_us;
    return ESP_OK;
}
#endif

/* 부팅 직후에는 NimBLE 초기화·ULP 캘리브가 잡은 일시적 락이 남아 오판하기
   쉬우므로 정상 광고 루프에 들어간 뒤부터 주기적으로 찍는다.
   printf/esp_pm_dump_locks는 로그 시스템이 아니라 stdout으로 직접 쓰므로
   로그 레벨 NONE에서도 출력된다. */
static void pm_debug_task(void *arg)
{
    (void)arg;

#if CONFIG_PM_LIGHT_SLEEP_CALLBACKS
    esp_pm_sleep_cbs_register_config_t cbs = {
        .exit_cb       = pm_ls_exit_cb,
        .exit_cb_prior = 100,
    };
    if (esp_pm_light_sleep_register_cbs(&cbs) != ESP_OK) {
        printf("*** light sleep callback 등록 실패 ***\n");
    }
#else
    printf("*** CONFIG_PM_LIGHT_SLEEP_CALLBACKS=n : 수면 횟수 계측 불가,"
           " 락 목록만 출력 ***\n");
#endif

    int64_t t_prev = esp_timer_get_time();
#if CONFIG_PM_LIGHT_SLEEP_CALLBACKS
    uint32_t c_prev = 0;
    int64_t  s_prev = 0;
#endif

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(15000));

        int64_t now     = esp_timer_get_time();
        int64_t elapsed = now - t_prev;
        if (elapsed <= 0) elapsed = 1;
        (void)elapsed;

        printf("\n===== PM STATUS (T+%llds) =====\n", (long long)(now / 1000000));
#if CONFIG_PM_LIGHT_SLEEP_CALLBACKS
        uint32_t c_now = s_ls_count;
        int64_t  s_now = s_ls_total_us;
        uint32_t d_cnt = c_now - c_prev;
        int64_t  d_us  = s_now - s_prev;
        /* 수면 점유율을 0.1% 단위 정수로 — float printf 의존 회피 */
        int duty = (int)((d_us * 1000) / elapsed);
        printf("light sleep: 누적 %u회 / 최근 15s간 %u회, 수면점유율 %d.%d%%\n",
               (unsigned)c_now, (unsigned)d_cnt, duty / 10, duty % 10);
        if (d_cnt == 0) {
            printf(">>> 수면 0회: 아래 락 목록에서 type=NO_LIGHT_SLEEP & count>0 확인\n");
        }
        c_prev = c_now;
        s_prev = s_now;
#endif
        esp_pm_dump_locks(stdout);
        printf("==============================\n");
        fflush(stdout);

        t_prev = now;
    }
}
#endif

#if INGPS_CAP_CHARGE_TEST
static const char *cap_test_reset_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_BROWNOUT: return "BROWNOUT";   /* ← 축전 실패의 직접 증거 */
    case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
    case ESP_RST_SW:       return "SW";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_INT_WDT:  return "INT_WDT";
    default:               return "OTHER";
    }
}

static void cap_test_sleep(const char *why)
{
    printf("[CAP] %s -> deep sleep %ds (축전 구간)\n", why, CAP_TEST_SLEEP_S);
    fflush(stdout);
    esp_sleep_enable_timer_wakeup((uint64_t)CAP_TEST_SLEEP_S * 1000000ULL);
    esp_deep_sleep_start();   /* noreturn */
}

/* 관측 구간이 끝나면 다시 축전 구간으로. */
static void cap_test_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(CAP_TEST_ACTIVE_MS));
    cap_test_sleep("관측 구간 종료");
}
#endif

void app_main(void)
{
#if INGPS_CAP_CHARGE_TEST
    /* 다른 어떤 초기화보다 먼저 판단한다 — 콜드부팅이면 부하를 하나도 켜지 않고
       바로 축전에 들어가야 의미가 있다. */
    {
        esp_reset_reason_t rr = esp_reset_reason();
        printf("\n[CAP] boot: reset=%s, wakeup=%d\n",
               cap_test_reset_str(rr), (int)esp_sleep_get_wakeup_cause());
        fflush(stdout);
        if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER) {
            cap_test_sleep("콜드부팅(축전 안 된 상태로 가정)");
        }
        printf("[CAP] 타이머 wake -> 관측 구간 %dms 시작\n", CAP_TEST_ACTIVE_MS);
        fflush(stdout);
    }
#endif

    wdt_guard_init();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* 적응형 광고 정책 초기화(NVS knob 로드).
       crash-loop escalation이 SAFE를 선언했으면 여기서 10s 최소광고로 고정. */
    adv_manager_init();

    /* 매초 UART 로그는 평균 소비전류를 올려 LS14500(Li-SOCl2, 낮은 연속전류
       스펙)의 브라운아웃 마진을 깎으므로 전역 묵음이 기본이다.
       아래 4개 태그만 예외 — 전부 저빈도(부팅 1회 또는 상태 전이)라 전류 영향이
       없으면서, 없으면 리셋사유·PM 설정 적용·광고 시작·정책 전이를 확인할
       방법이 사라진다. 매초 도는 데이터 로그는 ESP_LOGD라 여기서도 묵음. */
    esp_log_level_set("*", ESP_LOG_NONE);
    esp_log_level_set("WDT_GUARD", ESP_LOG_INFO);
    esp_log_level_set("WDT_TEST", ESP_LOG_WARN);
    esp_log_level_set("pm", ESP_LOG_INFO);
    esp_log_level_set("BLE_ADV", ESP_LOG_INFO);
    esp_log_level_set("ADV_MGR", ESP_LOG_INFO);
    /* eFuse curve fitting이 실제로 걸렸는지(= INL 보정 유효) 부팅 시 1회 확인.
       이게 닫혀 있으면 선형 폴백으로 조용히 동작해도 알 방법이 없다. */
    esp_log_level_set("ADC_CAL", ESP_LOG_INFO);

    /* RGB LED(GPIO48) 소등. WS2812B는 led_strip으로 RGB(0,0,0)를 보내야 꺼진다. */
    led_strip_handle_t led_strip;
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = PIN_RGB_LED,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    if (led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &led_strip) == ESP_OK) {
        led_strip_clear(led_strip);
        /* RMT 채널을 해제하지 않으면 드라이버가 ESP_PM_APB_FREQ_MAX 락을 계속
           쥐고 있어 light sleep에 못 들어간다. clear 후 즉시 del. */
        led_strip_del(led_strip);
    }
    wdt_guard_feed();

    /* max 80MHz: 이 펌웨어의 CPU 부하(페이로드 빌드+온도변환 수 ms/사이클)에
       160MHz는 불필요하고, 80MHz가 액티브 구간 전류와 BLE TX 피크 겹침
       (브라운아웃 마진)을 함께 낮춘다. BLE 컨트롤러는 80MHz에서 정상 동작.
       light sleep을 끄는 대조군에서는 min을 max와 붙여 DFS 전환 트랜지언트까지
       제거해 "웨이크 스텝만"의 기여를 분리한다.
       ⚠ 이 호출이 sdkconfig의 CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ(160)와
          CONFIG_PM_DFS_INIT_AUTO를 덮어쓴다 — 실동작 클럭은 여기가 결정한다. */
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz = 80,
        .min_freq_mhz = (INGPS_LIGHT_SLEEP ? 40 : 80),
        .light_sleep_enable = (INGPS_LIGHT_SLEEP != 0),
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_cfg));
#if !INGPS_LIGHT_SLEEP
    printf("\n*** INGPS_LIGHT_SLEEP=0 : light sleep DISABLED (진단 빌드) ***\n"
           "*** 평균 전류가 ~5배 높습니다. 측정 후 1로 원복하세요.      ***\n\n");
    fflush(stdout);
#endif

    /* per-chip ADC INL 보정(eFuse curve fitting) 준비. ULP는 raw만 누적하고
       실제 raw→mV 변환은 메인 CPU가 ble_adv.c에서 적용한다. 보정은 순수 SW
       변환이라 HW를 점유하지 않으므로 ULP 기동 순서와 무관하다.
       eFuse 미소성 시 내부적으로 선형 폴백. */
    if (!adc_cal_init()) {
        ESP_LOGW(TAG, "ADC INL cali off -> temperature uses linear ADC (less accurate)");
    }

#if INGPS_ULP_ADC_OFF
    printf("\n*** INGPS_ULP_ADC_OFF=1 : ULP/SARADC 미기동 (전류 A/B 실험 빌드) ***\n"
           "*** 온도·RMS는 전부 0으로 광고됩니다. 측정 후 0으로 원복하세요.   ***\n\n");
    fflush(stdout);
#else
    /* 비정상 리셋(WDT/panic/자가복구)에서 온 부팅이면 RTC_NOINIT에 보관해 둔
       직전 zero를 재사용해 10초 재캘리브를 생략 → 복구 다운타임 ~11.5s → ~1.5s. */
    int16_t saved_zx = 0, saved_zy = 0, saved_zz = 0;
    bool fast_resume = wdt_guard_fast_resume(&saved_zx, &saved_zy, &saved_zz);
    if (wdt_guard_safe_mode() && !fast_resume) {
        /* SAFE(crash-loop 한계 초과): 유효 zero가 없어도 캘리브를 생략하고 최대한
           빨리 최소광고로 복귀. zero=0이면 RMS에 DC 오프셋이 실리지만 SAFE에선
           adv_manager가 고정 cadence라 모션 승격에 쓰이지 않는다. */
        fast_resume = true;
    }

    ESP_LOGI(TAG, "Start ULP ADXL vibration sampler%s...",
             fast_resume ? " [fast resume]" : "");
    start_ulp_adc_measurement(/*do_zero_cal=*/!fast_resume,
                              saved_zx, saved_zy, saved_zz);

    if (!fast_resume) {
        /* 동적 zero 캘리브레이션: 정지 상태 raw 평균을 모아 zero로 설정.
           이 동안 sum_sq 누적이 멈춰 RMS=0이지만, 첫 광고가 이 이후에 시작되므로
           노출되지 않는다. TWDT보다 긴 대기라 1초 단위로 쪼개 feed한다. */
        const uint32_t CAL_MS = 10000;
        ESP_LOGI(TAG, "Calibrating ADXL zero (hold device still for %ums)...", (unsigned)CAL_MS);
        for (uint32_t t = 0; t < CAL_MS; t += 1000) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            wdt_guard_feed();
        }

        uint32_t n = ulp_shared.sample_count;
        if (n > 0) {
            ulp_shared.zero_x = (int16_t)(ulp_shared.sum_raw_x / n);
            ulp_shared.zero_y = (int16_t)(ulp_shared.sum_raw_y / n);
            ulp_shared.zero_z = (int16_t)(ulp_shared.sum_raw_z / n);
        }
        ESP_LOGI(TAG, "Calibrated zero (N=%u): X=%d Y=%d Z=%d",
                 (unsigned)n,
                 ulp_shared.zero_x, ulp_shared.zero_y, ulp_shared.zero_z);

        wdt_guard_save_zero(ulp_shared.zero_x, ulp_shared.zero_y, ulp_shared.zero_z);

        ulp_shared.sum_sq_x     = 0;
        ulp_shared.sum_sq_y     = 0;
        ulp_shared.sum_sq_z     = 0;
        ulp_shared.sum_dx_x     = 0;
        ulp_shared.sum_dx_y     = 0;
        ulp_shared.sum_dx_z     = 0;
        ulp_shared.sample_count = 0;
        ulp_shared.cal_phase    = 0;
    } else {
        /* start_ulp_adc_measurement(false, ...)가 zero 적용+cal_phase=0까지 처리 */
        ESP_LOGW(TAG, "WDT recovery boot: reusing saved zero (%d,%d,%d), skip 10s cal",
                 saved_zx, saved_zy, saved_zz);
    }
#endif  /* !INGPS_ULP_ADC_OFF */
    wdt_guard_feed();

    nimble_port_init();   /* BLE_INIT 실측 1.1s+ — TWDT 8s 내 여유 */
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set("IN_GPS");
    wdt_guard_feed();

    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(nimble_host_task);

#if INGPS_PM_DEBUG
    xTaskCreatePinnedToCore(pm_debug_task, "pm_dbg", 3072, NULL, 1, NULL, 0);
#endif

    /* 부팅 감시 종료 + 앱 헬스체크(ULP·ADV) 활성화. 이 시점부터 15s 내
       광고 갱신이 한 번도 성공하지 못하면(sync 실패 포함) 자가 재부팅. */
    wdt_guard_boot_done();

#if INGPS_CAP_CHARGE_TEST
    xTaskCreatePinnedToCore(cap_test_task, "cap_test", 2560, NULL, 2, NULL, 0);
#endif
}
