#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include <stdio.h>
#include "nvs_flash.h"
#include "led_strip.h"

#define PIN_RGB_LED  48

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
#include "sensor/sensor.h"
#include "watchdog/wdt_guard.h"

static const char *TAG = "APP_MAIN";

uint8_t g_own_addr_type = BLE_OWN_ADDR_RANDOM;

/* ★2026-07-15: 딥슬립 burst 재설계(wake→정착→측정→BLE burst→딥슬립)를 시도했다가
   롤백함. 이유: BLE_INIT(NimBLE 컨트롤러+호스트 초기화)만으로 실측 1.1초+ 걸려서,
   "매 사이클 완전 재부팅+BLE 재초기화" 구조로는 1초 주기 송신을 물리적으로 못 맞춤
   (사용자 요구사항: 측정값을 1초마다 BLE로 송신). → 상시가동+상시광고로 복귀.
   ULP+ADC 상시전류(800µA~2mA, ESP-IDF #11407)는 이 요구사항 하에선 감수.
   딥슬립 관련 코드(RTC_DATA_ATTR, esp_sleep_*, adv_burst_start 등)는 참고/차후
   재시도용으로 ulp_init.c·ble_adv.c에 남겨뒀지만 여기선 안 씀. */

/* STM32 게이트웨이가 매칭하는 BLE address (NimBLE wire format, LSB-first).
   main.c의 ADDR_A 케이스와 동일한 byte order. 첫 옥텟은 ble_adv.h의
   ESP_DEVICE_ID에 자동 연동(ESP32_NUM=1 → 0x01).
   사람 표기 MAC: CA:BB:CC:DD:EE:<ESP_DEVICE_ID>. */
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

    /* Core 1 고정: BLE 컨트롤러/NimBLE 호스트(Core 0)와 경합 제거.
       광고 페이로드 갱신·온도변환이 BLE 타이밍에 영향 주지 않도록 분리. */
    xTaskCreatePinnedToCore(adv_cycle_task, "adv_cycle", 4096, NULL, 5, NULL, 1);
}

static void nimble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
    vTaskDelete(NULL);
}

// #if ADXL_RAW_CAPTURE
// /* 진단용 raw 스트리밍: ULP 링버퍼를 드레인해 UART로 CSV(x,y,z, counts @200Hz)를
//    흘린다. CSV 오염을 막으려 캡처 동안 로그레벨을 ERROR로 내리고, 끝나면 복원.
//    BLE init 전에 호출하면 TX 노이즈 없는 "ADC 베이스라인"을 얻는다. nimble init
//    뒤로 옮기면 BLE 광고 ON/OFF가 노이즈에 주는 영향을 같은 방식으로 볼 수 있다. */
// static void adxl_raw_capture(uint32_t seconds)
// {
//     esp_log_level_set("*", ESP_LOG_ERROR);

//     uint32_t tail   = ulp_shared.ring_head;   /* 호출 시점부터 캡처 */
//     int64_t  t0     = esp_timer_get_time();
//     int64_t  t_end  = t0 + (int64_t)seconds * 1000000;
//     uint32_t n      = 0;

//     printf("\n# ADXL_RAW_CAPTURE BEGIN cols=x,y,z units=counts fs=200\n");
//     while (esp_timer_get_time() < t_end) {
//         uint32_t head = ulp_shared.ring_head;
//         /* 오버런(미드레인 덮어쓰기) 시 최신 RAW_RING_LEN 구간으로 점프 */
//         if ((uint32_t)(head - tail) > RAW_RING_LEN) {
//             tail = head - RAW_RING_LEN;
//         }
//         while ((int32_t)(head - tail) > 0) {
//             uint32_t i = tail & RAW_RING_MASK;
//             printf("%d,%d,%d\n",
//                    ulp_shared.ring_x[i], ulp_shared.ring_y[i], ulp_shared.ring_z[i]);
//             tail++;
//             n++;
//             head = ulp_shared.ring_head;
//             if ((uint32_t)(head - tail) > RAW_RING_LEN) {
//                 tail = head - RAW_RING_LEN;
//             }
//         }
//         vTaskDelay(pdMS_TO_TICKS(20));   /* ~4샘플마다 드레인 */
//     }
//     int64_t dt_ms = (esp_timer_get_time() - t0) / 1000;
//     /* measured_fs ≈ n*1000/dt_ms. float printf 회피 위해 정수로 노출. */
//     printf("# ADXL_RAW_CAPTURE END n=%u elapsed_ms=%lld (fs=n*1000/elapsed_ms)\n",
//            (unsigned)n, (long long)dt_ms);

//     esp_log_level_set("*", ESP_LOG_INFO);
// }
// #endif

void app_main(void)
{
    /* ★2026-07-18: 워치독 계층 초기화(상세: Docs/Watchdog_설계.md).
       - 리셋 사유 로그 + 비정상 리셋 카운터 갱신
       - Task WDT 8s/panic 재설정 + app_main을 부팅 구간 감시 대상으로 등록
       - 헬스모니터 태스크 기동(ULP 정지·ADV 갱신 정지 감지 → 자가 재부팅)
       이후 부팅 단계 사이의 wdt_guard_feed()는 "이 지점까지 8s 내 도달"을
       보증하는 체크포인트다. 마지막에 wdt_guard_boot_done()으로 감시 이관. */
    wdt_guard_init();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* ★2026-07-18: 05-18(ccb0688) 진단 모드에서 INFO로 켜둔 뒤 안 돌아왔던 걸
       원복. 반복되는 Brownout 조사 중 "매초 UART 로그 출력 + 상시 160MHz(아래
       pm_cfg)"가 배터리(LS14500, 연속전류 스펙이 낮은 Li-SOCl2) 평균 소비전류를
       올려 브라운아웃 마진을 깎고 있다는 정황 확인. 로그 필요하면 특정 TAG만
       ESP_LOG_INFO로 올려서(esp_log_level_set(TAG, ...)) 쓸 것. */
    esp_log_level_set("*", ESP_LOG_NONE);
    /* 위에서 전체를 죽여도 워치독 관련 태그는 살려둔다 — 안 그러면 리셋사유/
       재부팅 사유("WDT_GUARD: reset reason=...", "self-recovery reboot: ...")가
       UART에 하나도 안 찍혀서 Docs/Watchdog_설계.md 9절 검증 자체가 불가능해짐.
       매초 도는 BLE_ADV(RMS/therm/raw 로그)는 그대로 묵음 — 전류절감 목적 유지. */
    esp_log_level_set("WDT_GUARD", ESP_LOG_INFO);
    esp_log_level_set("WDT_TEST", ESP_LOG_WARN);
    /* pm 태그도 살려둠 — 안 그러면 아래 esp_pm_configure(&pm_cfg) 호출이 찍는
       "Frequency switching config: ... Light sleep: ENABLED" 확인 로그가
       묵음 처리돼, 부팅 초반에 뜨는 시스템 기본값(DISABLED) 로그만 보이고
       실제로 우리가 건 설정이 적용됐는지 확인할 방법이 없어짐. 이 로그는
       설정 변경 시 1회만 찍혀서 매초 로그처럼 전류에 영향 없음. */
    esp_log_level_set("pm", ESP_LOG_INFO);
    /* BLE_ADV도 INFO까지만 열어둠: adv_cycle_task()의 1회성 "ADV started
       (continuous)" 확인 로그는 보이지만, build_mfg_data()의 매초 3줄짜리
       RMS/therm/raw 데이터 로그는 ESP_LOGD로 내려놨으므로(ble_adv.c) 안 찍힘
       — "광고가 실제로 시작됐는지"만 확인하고 매초 전류 소모는 피함. */
    esp_log_level_set("BLE_ADV", ESP_LOG_INFO);

    // RGB LED(GPIO48) 끄기 - WS2812B는 led_strip으로 RGB(0,0,0) 전송해야 꺼짐
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
           쥐고 있어 light sleep으로 못 들어감. clear 후 즉시 del. */
        led_strip_del(led_strip);
    }
    wdt_guard_feed();   /* 체크포인트: NVS+LED 초기화 완료 */

    /* BLE MAC은 base MAC 경로(esp_base_mac_addr_set) 대신 NimBLE 측에서
       Random Address로 직접 설정. on_sync()에서 ble_hs_id_set_rnd 호출. */

    /* ★2026-07-18: 브라운아웃 조사로 원복. 05-18(ccb0688) 이후 진단용으로
       max=min=160(상시 최대클럭)+light_sleep_enable=false 상태였던 걸 원래
       의도(주석에 본인이 남겨둔 min_freq_mhz=40, light_sleep_enable=true)대로
       되돌림 — 평균 소비전류를 낮춰 LS14500 브라운아웃 마진 확보.
       ⚠ 미해결 TODO 그대로 남아있음: light sleep 복귀 시 ULP ADC가 wake
       transient 영향을 받아 "3초 주기 floating"이 재발할 수 있음(이게 애초에
       진단모드를 켰던 이유). 이 값으로 실기기 테스트 중 raw 로그/캘리브
       안정성에서 3초 주기 흔들림이 다시 보이면, ULP ADC용 별도 PM lock 또는
       wake-delay 처리를 추가로 구현해야 함(아직 미구현). */
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz = 160,
        .min_freq_mhz = 40,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_cfg));

    /* GPIO domain must stay powered through light sleep so the ADC pads
       remain valid for ULP between sample cycles. */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    /* ★2026-07-18 fast resume: 비정상 리셋(WDT/panic/자가복구)에서 온 부팅이면
       RTC_NOINIT에 보관해 둔 직전 zero를 재사용해 10초 재캘리브를 생략 →
       "1초마다 BLE 송신" 요구사항의 복구 다운타임을 ~11.5s에서 ~1.5s로 단축.
       do_zero_cal=false 경로는 07-15 딥슬립 재설계 때 만들어 둔 것을 재활용. */
    int16_t saved_zx = 0, saved_zy = 0, saved_zz = 0;
    bool fast_resume = wdt_guard_fast_resume(&saved_zx, &saved_zy, &saved_zz);

    ESP_LOGI(TAG, "Start ULP ADXL vibration sampler (200 Hz)%s...",
             fast_resume ? " [fast resume]" : "");
    start_ulp_adc_measurement(/*do_zero_cal=*/!fast_resume,
                              saved_zx, saved_zy, saved_zz);
    /* NTC: ULP가 200Hz로 raw 누적(sum_ntc/ntc_count) → 온도변환은 ble_adv에서
       Ratiometric+S-H로. adc_cali는 ratiometric을 깨서 제거함. */

    if (!fast_resume) {
        /* 동적 zero 캘리브레이션: 10초 동안 정지 상태 raw 평균을 모아 zero 값으로 설정.
           이 동안엔 sum_sq 누적이 멈춰 RMS=0으로 광고되지만, BLE 첫 광고가 이 이후에
           시작되므로 사실상 노출되지 않는다. */
        const uint32_t CAL_MS = 10000;
        ESP_LOGI(TAG, "Calibrating ADXL zero (hold device still for %ums)...", (unsigned)CAL_MS);
        /* TWDT(8s)보다 긴 대기이므로 1초 단위로 쪼개 feed하며 기다린다. */
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

        /* 다음 비정상 리셋 대비 zero 보관(fast resume용) */
        wdt_guard_save_zero(ulp_shared.zero_x, ulp_shared.zero_y, ulp_shared.zero_z);

        /* 정상 모드 전환 + 누적 리셋 */
        ulp_shared.sum_sq_x     = 0;
        ulp_shared.sum_sq_y     = 0;
        ulp_shared.sum_sq_z     = 0;
        ulp_shared.sample_count = 0;
        ulp_shared.cal_phase    = 0;
    } else {
        /* start_ulp_adc_measurement(false, ...)가 zero 적용+cal_phase=0까지 처리 */
        ESP_LOGW(TAG, "WDT recovery boot: reusing saved zero (%d,%d,%d), skip 10s cal",
                 saved_zx, saved_zy, saved_zz);
    }
    wdt_guard_feed();   /* 체크포인트: ULP 기동+캘리브 완료 */

// #if ADXL_RAW_CAPTURE
//     /* 노이즈 분석용 raw 캡처: 기기 정지 상태로 두면 60초간 x,y,z(counts)를
//        UART로 CSV 출력. 시리얼 터미널 로그를 파일로 저장 후
//        adxl_noise_fft.py --units counts --fs 200 으로 분석.
//        운영 빌드에선 ulp_shared.h의 ADXL_RAW_CAPTURE를 0으로. */
//     ESP_LOGW(TAG, "ADXL raw capture mode: hold device still, streaming 60s CSV...");
//     adxl_raw_capture(60);
// #endif

    nimble_port_init();   /* BLE_INIT 실측 1.1s+ — TWDT 8s 내 여유 */
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set("IN_GPS");
    wdt_guard_feed();   /* 체크포인트: NimBLE 초기화 완료 */

    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(nimble_host_task);

    /* 부팅 감시 종료 + 앱 헬스체크(ULP·ADV) 활성화. 이 시점부터 15s 내
       광고 갱신이 한 번도 성공하지 못하면(sync 실패 포함) 자가 재부팅. */
    wdt_guard_boot_done();
}
