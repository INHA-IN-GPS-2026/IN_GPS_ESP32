#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_sleep.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---- ULP ----
#include "ulp_riscv.h"
#include "ulp_adc.h"
#include "ulp_shared.h"

// ---- NimBLE ----
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_hs_adv.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// ULP binary
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static const char *TAG = "NIMBLE_ADV";

/* NTC 서미스터 설정 (VCC → NTC → ADC → R_pulldown → GND) */
#define THERMISTOR_R_PULLDOWN   10000.0f
#define THERMISTOR_R0           10000.0f
#define THERMISTOR_T0           298.15f     // 25°C in Kelvin
#define THERMISTOR_BETA         3981.0f
#define ADC_REF_VOLTAGE_MV      3300
#define ADC_MAX_RAW             4095.0f

static int16_t raw_to_temp_x100(uint16_t raw)
{
    float v_adc = (float)raw / ADC_MAX_RAW * (float)ADC_REF_VOLTAGE_MV;
    if (v_adc <= 0.0f) v_adc = 1.0f;

    float r_ntc = THERMISTOR_R_PULLDOWN * ((float)ADC_REF_VOLTAGE_MV - v_adc) / v_adc;
    if (r_ntc <= 0.0f) r_ntc = 1.0f;

    float temp_k = 1.0f / (1.0f / THERMISTOR_T0 +
                            logf(r_ntc / THERMISTOR_R0) / THERMISTOR_BETA);
    float temp_c = temp_k - 273.15f;
    return (int16_t)(temp_c * 100.0f);  // 예: 25.50°C → 2550
}

// manufacturer Data
static uint8_t mfg_data[] = {
    0x34, 0x12,   // company ID (LE)
    0x00, 0x00,   // adc_raw low/high byte (runtime update)
    0x00          // reason
};

static uint8_t g_own_addr_type = BLE_OWN_ADDR_PUBLIC;

// ULP ADC init
static void start_ulp_adc_gpio4(void)
{
    // RTC_PERIPH domain on
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    ulp_adc_cfg_t adc_config = {
        .adc_n    = ADC_UNIT_1,
        .channel  = ADC_CHANNEL_3,      // GPIO4 = ADC1_CH3
        .atten    = ADC_ATTEN_DB_12,
        .width    = ADC_BITWIDTH_12,
        .ulp_mode = ADC_ULP_MODE_RISCV,
    };

    ESP_ERROR_CHECK(ulp_adc_init(&adc_config));

    ESP_ERROR_CHECK(ulp_riscv_load_binary(
        ulp_main_bin_start,
        (size_t)(ulp_main_bin_end - ulp_main_bin_start)
    ));

    // ULP 0.2s 
    ESP_ERROR_CHECK(ulp_set_wakeup_period(0, 200000)); // 200ms

    ESP_ERROR_CHECK(ulp_riscv_run());
}

//GAP-Event
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    if (event->type == BLE_GAP_EVENT_ADV_COMPLETE) {
        ESP_LOGI(TAG, "ADV complete, reason=%d", event->adv_complete.reason);
    }
    return 0;
}

// Advertising
static void start_advertising_once(void)
{
    // ULP ADC raw → 온도 변환 후 mfg_data 업데이트
    uint16_t adc_raw = (uint16_t)ulp_shared.rpt.last_raw[0];
    int16_t temp_x100 = raw_to_temp_x100(adc_raw);   // 예: 25.50°C → 2550
    mfg_data[2] = (uint8_t)((uint16_t)temp_x100 & 0xFF);
    mfg_data[3] = (uint8_t)((uint16_t)temp_x100 >> 8);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = (uint8_t)strlen(name);
    fields.name_is_complete = 1;

    fields.mfg_data = mfg_data;
    fields.mfg_data_len = sizeof(mfg_data);

    uint8_t adv_data[BLE_HS_ADV_MAX_SZ];
    uint8_t adv_len = 0;

    int rc = ble_hs_adv_set_fields(&fields, adv_data, &adv_len, BLE_HS_ADV_MAX_SZ);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_adv_set_fields failed: %d", rc);
        return;
    }

    rc = ble_gap_adv_set_data(adv_data, adv_len);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_data failed: %d", rc);
        return;
    }

    rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "ADV started len=%u, temp=%.2f°C (raw=%u)",
             (unsigned)adv_len, temp_x100 / 100.0f, (unsigned)adc_raw);
}

//5s 0.5s advertisement
static void adv_cycle_task(void *arg)
{
    (void)arg;
    const uint32_t adv_on_ms  = 500;
    const uint32_t adv_off_ms = 5000;

    while (1) {
        start_advertising_once();
        vTaskDelay(pdMS_TO_TICKS(adv_on_ms));

        int rc = ble_gap_adv_stop();
        ESP_LOGI(TAG, "ADV stop rc=%d", rc);

        vTaskDelay(pdMS_TO_TICKS(adv_off_ms));
    }
}

// sync callback
static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }
    xTaskCreate(adv_cycle_task, "adv_cycle", 4096, NULL, 5, NULL);
}

//Nimble host task
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