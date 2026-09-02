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

#include "board_pins.h"

/* Light-sleep 통계와 PM lock을 15초마다 출력한다. 전류 측정 시에는 끈다. */
#define INGPS_PM_DEBUG  0

/* 정상 운용은 light sleep을 사용한다. 0은 전력 비교 시험 전용이다. */
#define INGPS_LIGHT_SLEEP  1

/* BLE를 끄고 센서와 light-sleep의 바닥 전류만 측정하는 진단 옵션이다. */
#define INGPS_BLE_DISABLED  1
#define INGPS_BLE_OFF_HB_STALE_MS  (24U * 3600U * 1000U)

/* ★워치독 전 계층을 끄는 진단 스위치는 watchdog/wdt_guard.h의
   INGPS_WDT_DISABLED에 있다(여기가 아니다 — wdt_guard.c에서도 보여야 하므로).
   1로 두면 L1 헬스모니터(2초 주기 wake)와 L2 Task WDT가 사라진다.
   INGPS_BLE_DISABLED=1과 함께 쓰면 주기적 waker가 하나도 남지 않는다. */

/* 콜드부팅 직후 deep sleep으로 축전한 뒤 활성 구간을 반복하는 전원 시험 옵션이다. */
#define INGPS_CAP_CHARGE_TEST  0
#define CAP_TEST_SLEEP_S       30
#define CAP_TEST_ACTIVE_MS     30000

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "ble/ble_adv.h"
#include "ble/adv_manager.h"
#include "sensor/i2c_bus.h"
#include "sensor/as6221.h"
#include "sensor/adxl345.h"
#include "sensor/sensor.h"
#include "watchdog/wdt_guard.h"

#if INGPS_HAS_RGB_LED
#include "led_strip.h"
#endif

static const char *TAG = "APP_MAIN";

uint8_t g_own_addr_type = BLE_OWN_ADDR_RANDOM;

#if !INGPS_BLE_DISABLED
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
#endif /* !INGPS_BLE_DISABLED */

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

/* 부팅 직후에는 NimBLE 초기화·I2C 센서 기동이 잡은 일시적 락이 남아 오판하기
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
    /* I2C 스캔 결과와 채널별 배정 주소를 부팅 시 1회 확인. 이게 닫혀 있으면
       "온도만 전부 NULL"일 때 센서 미연결인지 주소 불일치인지 구분할 수 없다. */
    esp_log_level_set("AS6221", ESP_LOG_INFO);
    /* 버스 생성 성공/실패와 ADXL345 DEVID 확인 결과. 이게 닫혀 있으면
       "RMS가 전부 0"일 때 센서 미실장인지 버스 문제인지 구분할 수 없다. */
    esp_log_level_set("I2C_BUS", ESP_LOG_INFO);
    esp_log_level_set("ADXL345", ESP_LOG_INFO);

#if INGPS_HAS_RGB_LED
    /* RGB LED(GPIO48) 소등. WS2812B는 led_strip으로 RGB(0,0,0)를 보내야 꺼진다.
       ⚠ rev 4.0 실장 보드에는 WS2812B가 없다(IO48 미접속) — 기본 0.
       데브킷 브링업에서만 board_pins.h의 INGPS_HAS_RGB_LED를 1로 둘 것.
       슈퍼캡 부팅 마진이 빠듯하므로 실장 보드에서는 이 RMT 초기화 비용도 아깝다. */
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
#endif

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

    /* === 센서 프런트엔드 (rev 4.0: 단일 I2C 버스 GPIO8/9) ==============
       버스를 먼저 세우고 드라이버들은 device만 붙인다. 순서가 중요한 이유는
       i2c_new_master_bus()가 포트당 1회만 성공하기 때문이다 — 구 리비전처럼
       드라이버가 각자 버스를 만들면 두 번째가 ESP_ERR_INVALID_STATE로 죽는다.

       무엇이 실패해도 부팅은 계속한다. 온도는 NULL 센티넬로, 진동은 0으로
       광고되고 게이트웨이/서버 계약(mfg_data 앞 13B, offset 0..12)은 그대로
       유지된다. (mfg_data 전체는 15B — [13..14]에 SHF 추정 core_temp가
       추가됐으나 게이트웨이/서버는 이를 읽지 않는다. ble/ble_adv.c 참조.) */
    if (!ingps_i2c_bus_init()) {
        ESP_LOGE(TAG, "I2C 버스 기동 실패 -> 온도·진동 전부 무효로 광고된다");
    } else {
        /* AS6221을 먼저 붙인다. 전원 인가 후 typ 36ms에 첫 변환이 끝나므로
           ADXL345 설정을 하는 사이에 온도가 유효해진다. */
        if (!as6221_init()) {
            ESP_LOGW(TAG, "AS6221 unavailable -> temp1/temp2 will advertise as invalid (NULL)");
        }
        wdt_guard_feed();   /* 최악(버스 사망) 8회 프로브 x 50ms = 0.4s 소모 */

        /* ADXL345(0x53). ADXL335 + ULP SARADC 경로를 대체한다.
           ★zero 캘리브레이션이 없다 — RMS를 분산 공식으로 내므로 중력 DC가
             대수적으로 소거된다(sensor.h 참조). 구 경로가 부팅마다 쓰던
             1초 캘리브(약 0.3J)가 여기서 사라졌고, 이것이 슈퍼캡 콜드스타트
             마진에 직접 기여한다(E_boot 0.75J -> 0.45J).
           여기서 측정을 시작해 두면 뒤이은 NimBLE init(~1.1s) 동안 FIFO가
           가득 차(32샘플 = 320ms) 첫 광고부터 유효한 RMS가 실린다. */
        if (!adxl345_init()) {
            ESP_LOGW(TAG, "ADXL345 unavailable -> rms_x/y/z will advertise as 0");
        }
    }
    wdt_guard_feed();

#if !INGPS_BLE_DISABLED
    nimble_port_init();   /* BLE_INIT 실측 1.1s+ — TWDT 8s 내 여유 */
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set("IN_GPS");
    wdt_guard_feed();

    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(nimble_host_task);
#else
    /* BLE 미기동 -> adv_cycle_task도 없음 -> WDT_HB_ADV/ACCEL을 kick할 사람이
       없다. wdt_guard_boot_done()이 곧 ADV 시계를 arm하므로 그 전에 두
       heartbeat의 stale 한도를 늘려 벤치 측정 중 오탐 재부팅을 막는다. */
    printf("\n*** INGPS_BLE_DISABLED=1 : BLE 스택 미기동 (light sleep floor 단독 측정 빌드) ***\n"
           "*** 게이트웨이에 안 잡힘 — 진단 전용, 측정 후 0으로 원복할 것 ***\n\n");
    fflush(stdout);
    wdt_guard_set_hb_stale(WDT_HB_ADV, INGPS_BLE_OFF_HB_STALE_MS);
    wdt_guard_set_hb_stale(WDT_HB_ACCEL, INGPS_BLE_OFF_HB_STALE_MS);
#endif

#if INGPS_PM_DEBUG
    xTaskCreatePinnedToCore(pm_debug_task, "pm_dbg", 3072, NULL, 1, NULL, 0);
#endif

    /* 부팅 감시 종료 + 앱 헬스체크(ACCEL·ADV) 활성화. 이 시점부터 15s 내
       광고 갱신이 한 번도 성공하지 못하면(sync 실패 포함) 자가 재부팅. */
    wdt_guard_boot_done();

#if INGPS_CAP_CHARGE_TEST
    xTaskCreatePinnedToCore(cap_test_task, "cap_test", 2560, NULL, 2, NULL, 0);
#endif
}
