#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"
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
#include "ble/ble_adv.h"

static const char *TAG = "APP_MAIN";

uint8_t g_own_addr_type = BLE_OWN_ADDR_PUBLIC;

static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }
    xTaskCreate(adv_cycle_task, "adv_cycle", 4096, NULL, 5, NULL);
}

static void nimble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // 배포 시 로그 끄기 (UART TX LED 소등 + 전류 절감)
    esp_log_level_set("*", ESP_LOG_NONE);

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
    }

    // Custom BLE MAC address (nimble_port_init 전에 설정)
    uint8_t custom_mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFD}; // BLE MAC = AA:BB:CC:DD:EE:FF (base+2)
    ESP_ERROR_CHECK(esp_base_mac_addr_set(custom_mac));

    ESP_LOGI(TAG, "Start ULP ADC (GPIO4=ADC1_CH3)...");
    start_ulp_adc_gpio4();

    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set("IN_GPS");

    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(nimble_host_task);
}
