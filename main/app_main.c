#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_timer.h"
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
#include "sensor/adc_cal.h"

static const char *TAG = "APP_MAIN";

uint8_t g_own_addr_type = BLE_OWN_ADDR_RANDOM;

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

    xTaskCreate(adv_cycle_task, "adv_cycle", 4096, NULL, 5, NULL);
}

static void nimble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
    vTaskDelete(NULL);
}

/* 5초 후 한 번 PM 락 덤프 → BLE 컨트롤러 init과 첫 광고가 끝난 뒤의
   안정된 상태를 보기 위함. printf는 LOG NONE과 무관하게 출력됨. */
static void pm_diag_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(5000));
    printf("\n=== PM locks (post BLE init) ===\n");
    esp_pm_dump_locks(stdout);
    printf("================================\n");
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
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // 진단 중에는 INFO 로그 켜둠. 운영 펌웨어 만들 때 ESP_LOG_NONE으로 복귀.
    esp_log_level_set("*", ESP_LOG_INFO);

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

    /* BLE MAC은 base MAC 경로(esp_base_mac_addr_set) 대신 NimBLE 측에서
       Random Address로 직접 설정. on_sync()에서 ble_hs_id_set_rnd 호출. */

    /* 진단 모드: light sleep과 DFS를 끄고 ADC 안정성 확인.
       3초 주기 floating이 사라지면 light sleep transient가 원인. 운영 펌웨어로
       복귀할 땐 min_freq_mhz=40, light_sleep_enable=true로 되돌리되, ULP ADC가
       light sleep wake transient에 영향받지 않도록 별도 PM lock 또는 wake delay
       처리가 필요. */
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz = 160,
        .min_freq_mhz = 160,
        .light_sleep_enable = false,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_cfg));

    /* GPIO domain must stay powered through light sleep so the ADC pads
       remain valid for ULP between sample cycles. */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    /* per-chip ADC INL 보정(eFuse curve fitting) 준비. ULP는 raw만 누적하고
       실제 보정은 메인 CPU가 raw→mV 변환(ble_adv.c)에서 적용한다.
       eFuse 미소성 시 내부적으로 선형 폴백. ULP ADC와 독립적인 계수 객체라
       start_ulp 순서와 무관하다. */
    if (!adc_cal_init()) {
        ESP_LOGW(TAG, "ADC INL cali off -> temperature uses linear ADC (less accurate)");
    }

    ESP_LOGI(TAG, "Start ULP ADXL vibration sampler (200 Hz)...");
    start_ulp_adc_gpio4();

    /* 동적 zero 캘리브레이션: 10초 동안 정지 상태 raw 평균을 모아 zero 값으로 설정.
       이 동안엔 sum_sq 누적이 멈춰 RMS=0으로 광고되지만, BLE 첫 광고가 이 이후에
       시작되므로 사실상 노출되지 않는다. */
    const uint32_t CAL_MS = 10000;
    ESP_LOGI(TAG, "Calibrating ADXL zero (hold device still for %ums)...", (unsigned)CAL_MS);
    vTaskDelay(pdMS_TO_TICKS(CAL_MS));

    uint32_t n = ulp_shared.sample_count;
    if (n > 0) {
        ulp_shared.zero_x = (int16_t)(ulp_shared.sum_raw_x / n);
        ulp_shared.zero_y = (int16_t)(ulp_shared.sum_raw_y / n);
        ulp_shared.zero_z = (int16_t)(ulp_shared.sum_raw_z / n);
    }
    ESP_LOGI(TAG, "Calibrated zero (N=%u): X=%d Y=%d Z=%d",
             (unsigned)n,
             ulp_shared.zero_x, ulp_shared.zero_y, ulp_shared.zero_z);

    /* 정상 모드 전환 + 누적 리셋 */
    ulp_shared.sum_sq_x     = 0;
    ulp_shared.sum_sq_y     = 0;
    ulp_shared.sum_sq_z     = 0;
    ulp_shared.sum_dx_x     = 0;
    ulp_shared.sum_dx_y     = 0;
    ulp_shared.sum_dx_z     = 0;
    ulp_shared.sample_count = 0;
    ulp_shared.cal_phase    = 0;

// #if ADXL_RAW_CAPTURE
//     /* 노이즈 분석용 raw 캡처: 기기 정지 상태로 두면 60초간 x,y,z(counts)를
//        UART로 CSV 출력. 시리얼 터미널 로그를 파일로 저장 후
//        adxl_noise_fft.py --units counts --fs 200 으로 분석.
//        운영 빌드에선 ulp_shared.h의 ADXL_RAW_CAPTURE를 0으로. */
//     ESP_LOGW(TAG, "ADXL raw capture mode: hold device still, streaming 60s CSV...");
//     adxl_raw_capture(60);
// #endif

    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set("IN_GPS");

    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(nimble_host_task);

    xTaskCreate(pm_diag_task, "pm_diag", 3072, NULL, 1, NULL);
}
