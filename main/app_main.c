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
#include "driver/gpio.h"   /* floor_test_task 생존 토글 */

#include "board_pins.h"

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

/* ★floor(라이트슬립 잔류) 전류 실측 모드 — 스코프/DMM 측정용.

   ⚠ 스위치는 여기가 아니라 ble/ble_adv.h의 INGPS_FLOOR_TEST(기본 0)다.
     app_main.c와 ble_adv.c 두 번역 단위가 같은 값을 봐야 하므로 헤더에 두었다.
     여기서 #define 하면 ble_adv.c는 여전히 0으로 컴파일되어 광고만 멈추고
     센서 읽기는 계속 도는 어중간한 빌드가 된다.

   왜 필요한가:
     전력예산의 지배 항목인 "라이트슬립 잔류 240µA"는 🔶설계 가정일 뿐 한 번도
     측정된 적이 없다. 그런데 TX 버스트(피크 ~294mA)가 섞여 있으면 floor를
     분리할 수 없다 — 두 값의 동적 범위가 1000:1이라 한 션트로 둘 다 못 잡는다.
     그래서 "광고만 멈춘 상태"를 만들어 floor만 남긴다.

   동작:
     정상 부팅 → FLOOR_TEST_SETTLE_MS 동안 평소대로 광고(부팅 성공 확인 구간)
       → WDT_HB_ADV 무장해제 → ble_adv_pause()
       → adv_cycle_task가 센서 읽기·페이로드 갱신까지 중단(ble_adv.c)
       → 이후 FLOOR_TEST_TOGGLE_S 주기 생존 토글만 남음

   ⚠ 안정화 구간을 먼저 두는 이유: 과거 "R15 단선 DMM 측정 60~70µA"가
     부팅 실패 상태를 측정한 값이라 무효 처리된 이력이 있다
     (Docs/INGPS_전력예산_디지털전환_ADXL345_2026-08-07.md 56행).
     이 구간에 BLE 스캐너로 광고가 보여야 이후 측정이 유효하다.

   ⚠ 저측 션트(0.1Ω, BAT- 경로)를 쓸 때 UART를 PC에 물리면 PC 접지가 션트를
     우회시켜 측정이 무의미해진다. 측정 중에는 UART를 분리할 것.
     그래서 "살아 있는가"를 UART 없이 확인할 수단이 생존 토글이다.

   측정 절차 전문: Docs/INGPS_floor전류_측정절차_2026-08-11.md */

/* 광고를 멈추기 전 정상 동작 확인 구간. BLE 스캐너로 광고가 잡히는 것을
   눈으로 확인할 시간이 필요하므로 너무 짧게 잡지 말 것. */
#define FLOOR_TEST_SETTLE_MS     30000

/* 생존 토글 주기(초). 매 토글마다 CPU가 라이트슬립에서 한 번 깨므로
   이 값이 짧을수록 측정 대상인 floor 자체를 오염시킨다.
   10s면 웨이크 기여가 ~2µA(240µA의 1%) 수준으로 추정된다 — 🔶미검증.
   더 깨끗하게 재려면 30~60s로 늘릴 것. */
#define FLOOR_TEST_TOGGLE_S      10

/* 생존 토글 출력 핀. -1이면 토글 없이 전류 파형만으로 판정한다.
   ⚠ 기본값 GPIO3은 board_pins.h 기준 rev 4.0에서 "어떤 네트도 붙지 않는" 핀이라
     경합이 없다. 대신 트레이스가 없어 모듈 패드에서 직접 프로빙해야 한다.
   ⚠ 아래 핀들은 절대 쓰지 말 것 — 전부 외부 소자가 구동하는 입력이라
     출력으로 잡으면 충돌한다:
       GPIO10 LBO_OUT(레귤레이터 출력)  GPIO11 PGOOD_OUT(레귤레이터 출력)
       GPIO12 ALERT_OUT(BQ35100 출력)   GPIO15/16 ALERT_TH1/2(AS6221 출력 가능)
   ⚠ GPIO0은 J6 커넥터에 나와 있어 프로빙은 쉽지만, 토글이 LOW인 순간에 리셋이
     걸리면 다운로드 모드로 빠진다. 브라운아웃이 날 수 있는 측정에서는 금물. */
#define FLOOR_TEST_TOGGLE_PIN    3

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

   ⚠ ADXL335(~350µA)는 전원 게이팅 회로가 없어 deep sleep 중에도 계속 먹는다.
     즉 축전 구간 실측은 7µA가 아니라 보드 기준 ~350µA대가 정상이다.
     이 값이 안 나오면 회로를 먼저 의심할 것.
     (아날로그 버전의 NTC 분압 ~330µA는 AS6221 I2C 전환으로 사라졌다 —
      AS6221은 2개 합쳐 ~4µA 수준이라 이 기준선이 ~680µA에서 내려갔다.
      ★수치는 데이터시트 기반 추정이며 실측 미검증.)
   ⚠ ULP가 돌면 deep sleep 중에도 샘플링을 계속하므로, 이 테스트는
     INGPS_ULP_ADC_OFF=1과 함께 써야 축전 구간이 깨끗하다. */
/* ★플래시 사용량 리포트 (branch flash_info_check 전용).
   1이면 부팅 시 1회, 플래시 물리 크기 / 파티션 레이아웃 / app 이미지 실크기 /
   NVS 사용률을 UART로 덤프한다.

   왜 런타임 확인이 필요한가:
     esptool flash_id는 "칩이 16MB"까지만 알려준다. 정작 중요한
     "sdkconfig가 2MB로 잡혀 있어 14MB가 죽어 있다"와 "app 파티션 1MB 중
     실제 몇 %를 쓰는가"는 칩 위에서 esp_flash_get_physical_size()와
     esp_flash_get_size()를 나란히 읽어야 확정된다.

   ⚠ 전류 실측(INGPS_FLOOR_TEST / INGPS_CAP_CHARGE_TEST) 빌드에서는 0으로
     되돌릴 것. 부팅 1회 출력이지만 UART 송신이 CPU를 깨우고 PM 락을 잡는다.
   ⚠ 이 리포트는 읽기 전용이다 — 플래시에 아무것도 쓰지 않는다. */
#define INGPS_FLASH_REPORT     1

#define INGPS_CAP_CHARGE_TEST  0
#define CAP_TEST_SLEEP_S       30      /* 축전 구간(deep sleep) */
/* 관측 구간. ⚠ INGPS_ULP_ADC_OFF=0(실부하)이면 첫 광고까지 ~2.5s가 걸린다:
   deep sleep wake는 ESP_RST_DEEPSLEEP이라 wdt_guard가 비정상 리셋으로 보지 않고
   → fast_resume=false → ADXL zero 캘리브 1s가 매번 돌고 + NimBLE init 1.1s.
   (캘리브가 10s이던 시절엔 ~11.5s였다 — 아래 ADXL_CAL_MS_NORMAL 주석 참조.)
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
#include "sensor/as6221.h"
#include "sensor/sensor.h"
#include "watchdog/wdt_guard.h"
#if INGPS_FLASH_REPORT
#include "diag/flash_report.h"
#endif

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

#if INGPS_FLOOR_TEST
/* 이 태스크는 의도적으로 wdt_guard_task_subscribe()를 하지 않는다.
   TWDT 감시 대상이 되면 8s 안에 feed하려고 4s마다 깨야 하는데, 그 웨이크가
   바로 측정 대상인 floor를 오염시킨다. 미등록 태스크는 TWDT가 보지 않고,
   idle 태스크 감시는 IDF가 라이트슬립 중에도 idle 훅으로 처리한다. */
static void floor_test_task(void *arg)
{
    (void)arg;

    /* 1) 안정화 — 이 구간에는 평소대로 광고한다. BLE 스캐너로 잡히는지
          확인할 것. 안 잡히면 이후 측정값은 전부 무효다. */
    vTaskDelay(pdMS_TO_TICKS(FLOOR_TEST_SETTLE_MS));

    /* 2) ADV heartbeat 신선도 감시를 먼저 푼다. 순서가 중요 — 광고를 멈춘 뒤에
          풀면 그 사이 stale(15s) 판정으로 자가 재부팅이 걸린다.
          wdt_guard_set_hb_stale은 느슨해지는 방향만 허용하므로 안전하다. */
    wdt_guard_set_hb_stale(WDT_HB_ADV, 0xFFFFFFFFu);

    /* 3) 광고 정지. adv_cycle_task는 INGPS_FLOOR_TEST 빌드에서 s_adv_paused를
          보고 센서 읽기·페이로드 갱신까지 건너뛴다(ble_adv.c). */
    ble_adv_pause();

    printf("\n*** INGPS_FLOOR_TEST : 광고 정지, floor 측정 구간 시작 ***\n"
           "*** 지금부터 UART를 분리하고 측정하세요. 측정 후 0으로 원복. ***\n\n");
    fflush(stdout);

#if FLOOR_TEST_TOGGLE_PIN >= 0
    gpio_config_t tog = {
        .pin_bit_mask = 1ULL << FLOOR_TEST_TOGGLE_PIN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&tog);
    int lvl = 0;
    gpio_set_level(FLOOR_TEST_TOGGLE_PIN, lvl);
#endif

    /* 4) 생존 토글. 스코프 CH2에서 주기 2×FLOOR_TEST_TOGGLE_S의 구형파가 보이면
          살아 있는 것이고, 레벨이 굳으면 죽은 것이다. 토글을 끈 경우(-1)에도
          CH1 전류 파형 자체가 판정 근거가 된다 — 부팅 버스트 → 정숙 구간이
          보여야 하고, 재부팅 루프면 버스트가 반복된다. */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)FLOOR_TEST_TOGGLE_S * 1000u));
#if FLOOR_TEST_TOGGLE_PIN >= 0
        lvl ^= 1;
        gpio_set_level(FLOOR_TEST_TOGGLE_PIN, lvl);
#endif
    }
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

#if INGPS_FLASH_REPORT
    /* NVS 초기화 직후에 둔다 — nvs_get_stats()가 초기화를 전제로 한다.
       광고 정책(adv_manager_init)보다 앞이라 NVS 사용량은 "knob 로드 전"
       기준이지만, 엔트리 수 차이는 knob 몇 개 수준이라 판독에 지장 없다.
       출력이 UART로 나가는 동안(≈0.3s @115200) TWDT 8s를 소모하므로
       끝나고 바로 feed한다. */
    flash_report_print();
    wdt_guard_feed();
#endif

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

    /* 온도 I2C 프런트엔드(AS6221 ×2, GPIO13/14). ULP/SARADC와 핀·페리페럴이
       겹치지 않으므로 기동 순서는 무관하지만, ULP보다 먼저 붙여 두면 첫 광고
       시점에 온도가 이미 유효하다(AS6221은 전원 인가 후 typ 36ms에 첫 변환 완료).
       실패해도 부팅은 계속한다 — 온도만 NULL로 나가고 진동은 정상 동작한다. */
    if (!as6221_init()) {
        ESP_LOGW(TAG, "AS6221 unavailable -> temp1/temp2 will advertise as invalid (NULL)");
    }
    wdt_guard_feed();   /* 최악(버스 사망) 8회 프로브 × 50ms = 0.4s 소모 */

#if INGPS_ULP_ADC_OFF
    printf("\n*** INGPS_ULP_ADC_OFF=1 : ULP/SARADC 미기동 (전류 A/B 실험 빌드) ***\n"
           "*** 온도·RMS는 전부 0으로 광고됩니다. 측정 후 0으로 원복하세요.   ***\n\n");
    fflush(stdout);
#else
    /* 비정상 리셋(WDT/panic/자가복구)에서 온 부팅이면 RTC_NOINIT에 보관해 둔
       직전 zero를 재사용해 재캘리브를 생략 → 복구 다운타임 ~1.5s. */
    int16_t saved_zx = 0, saved_zy = 0, saved_zz = 0;
    bool fast_resume = wdt_guard_fast_resume(&saved_zx, &saved_zy, &saved_zz);

    /* === zero 캘리브 시간 결정 ==========================================
       zero의 유일한 역할은 dx(=raw−zero)를 노이즈 수준으로 붙들어 ULP의
       sum_sq(uint32)가 넘치지 않게 하는 것이다. RMS 값 자체는 분산 공식
           var = E[dx²] − (E[dx])²  =  E[raw²] − raw̄²
       이라 zero가 대수적으로 소거된다 — 중력 1g든 기울기든 zero가 뭐든
       상관없이 DC가 빠진다(sensor.c 참조). 즉 "정확한" zero가 아니라
       "대략 맞는" zero면 충분하고, 종전 10초 평균은 어차피 버려지는
       정밀도를 사고 있었다. 1초(≈195샘플)면 평균 표준오차가 1 count 아래다.

       ⚠ 반대로 zero=0으로 두면 dx≈raw≈2048 → dx²≈4.2e6이 매 샘플 쌓여
         194.7Hz 기준 n>1025(창 ≈5.3s)에서 sum_sq가 uint32를 넘긴다.
         그러면 var이 음수로 계산되고 sensor.c의 클램프에 걸려 RMS가
         세 축 모두 0으로 광고된다 — "고장"이 아니라 "진동 없음"처럼 보인다.
         SAFE cadence가 10s이므로 SAFE에서 zero=0은 반드시 이 함정에 빠진다.
         그래서 SAFE에서도 캘리브를 완전히 생략하지 않고 최소 표본만 잡는다. */
    const uint32_t ADXL_CAL_MS_NORMAL = 1000;
    /* SAFE는 복귀 속도가 우선이라 최소 표본만. FreeRTOS tick이 10ms(HZ=100)라
       실제 대기는 40~50ms → 194.7Hz에서 ≈8~10샘플. zero 정밀도는 필요 없다. */
    const uint32_t ADXL_CAL_MS_SAFE   = 50;

    uint32_t cal_ms;
    if (fast_resume) {
        cal_ms = 0;                            /* 저장된 zero 재사용 */
    } else if (wdt_guard_safe_mode()) {
        cal_ms = ADXL_CAL_MS_SAFE;
    } else {
        cal_ms = ADXL_CAL_MS_NORMAL;
    }

    ESP_LOGI(TAG, "Start ULP ADXL vibration sampler%s...",
             fast_resume ? " [fast resume]" : "");
    start_ulp_adc_measurement(/*do_zero_cal=*/cal_ms > 0,
                              saved_zx, saved_zy, saved_zz);

    if (cal_ms > 0) {
        /* 동적 zero 캘리브레이션: 정지 상태 raw 평균을 모아 zero로 설정.
           이 동안 sum_sq 누적이 멈춰 RMS=0이지만, 첫 광고가 이 이후에 시작되므로
           노출되지 않는다. TWDT(8s)보다 길어질 경우를 대비해 1초 단위로 쪼개 feed. */
        ESP_LOGI(TAG, "Calibrating ADXL zero (hold device still for %ums)...",
                 (unsigned)cal_ms);
        for (uint32_t t = 0; t < cal_ms; ) {
            uint32_t chunk = (cal_ms - t > 1000) ? 1000 : (cal_ms - t);
            vTaskDelay(pdMS_TO_TICKS(chunk));
            wdt_guard_feed();
            t += chunk;
        }

        /* SAFE의 50ms 창은 ULP 기동 지연(첫 wake까지 ~5ms)과 겹치면 표본이
           0으로 나올 수 있다. 그대로 내려가면 zero=0이 되어 위의 오버플로가
           그대로 재현되므로, 표본이 생길 때까지 상한을 두고 더 기다린다. */
        uint32_t n = ulp_shared.sample_count;
        for (int retry = 0; n == 0 && retry < 10; retry++) {
            vTaskDelay(pdMS_TO_TICKS(20));
            wdt_guard_feed();
            n = ulp_shared.sample_count;
        }

        if (n > 0) {
            ulp_shared.zero_x = (int16_t)(ulp_shared.sum_raw_x / n);
            ulp_shared.zero_y = (int16_t)(ulp_shared.sum_raw_y / n);
            ulp_shared.zero_z = (int16_t)(ulp_shared.sum_raw_z / n);
            ESP_LOGI(TAG, "Calibrated zero (N=%u, %ums): X=%d Y=%d Z=%d",
                     (unsigned)n, (unsigned)cal_ms,
                     ulp_shared.zero_x, ulp_shared.zero_y, ulp_shared.zero_z);
            wdt_guard_save_zero(ulp_shared.zero_x, ulp_shared.zero_y, ulp_shared.zero_z);
        } else {
            /* ULP가 한 표본도 못 냈다 = ULP 자체가 안 돌고 있다는 뜻.
               zero=0으로 남으므로 긴 cadence에서 RMS가 0으로 나온다.
               헬스모니터의 ULP stall 감시가 곧 재부팅시킬 상황이다. */
            ESP_LOGE(TAG, "ADXL zero cal: ULP 표본 0개 — zero=0 유지."
                          " 긴 cadence에서 RMS가 0으로 나갈 수 있음 (ULP 미동작 의심)");
        }

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
        ESP_LOGW(TAG, "WDT recovery boot: reusing saved zero (%d,%d,%d), skip cal",
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

#if INGPS_FLOOR_TEST
    /* Core 0(BLE 호스트 쪽)에 낮은 우선순위로. 하는 일이 vTaskDelay와 GPIO
       토글뿐이라 adv_cycle_task(Core 1, prio 5)와 경합하지 않는다. */
    xTaskCreatePinnedToCore(floor_test_task, "floor_test", 2560, NULL, 1, NULL, 0);
#endif
}
