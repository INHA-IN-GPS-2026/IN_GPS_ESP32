#include "ble_adv.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_hs_adv.h"
#include "services/gap/ble_svc_gap.h"

#include "sensor/sensor.h"
#include "ulp_shared.h"

static const char *TAG = "BLE_ADV";

extern volatile ulp_shared_t ulp_shared;

/*
 * Manufacturer Specific Data (13바이트).
 * ESP_DEVICE_ID는 ble_adv.h의 ESP32_NUM에서 파생.
 *   [0..1]   company ID (LE) = 0x1234
 *   [2..3]   temp1   (°C × 100, int16 LE)   GPIO4 NTC
 *   [4..5]   temp2   (°C × 100, int16 LE)   GPIO5 NTC
 *   [6..7]   rms_x   (mg, uint16 LE)
 *   [8..9]   rms_y   (mg, uint16 LE)
 *   [10..11] rms_z   (mg, uint16 LE)
 *   [12]     device ID (esp_test 고정값)
 */
static uint8_t mfg_data[] = {
    0x34, 0x12,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    ESP_DEVICE_ID
};

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    if (event->type == BLE_GAP_EVENT_ADV_COMPLETE) {
        ESP_LOGI(TAG, "ADV complete, reason=%d", event->adv_complete.reason);
    }
    return 0;
}

/* ULP 누적값을 읽고 0으로 리셋 (최대 1샘플 race 손실은 무시). */
static void read_and_reset_accum(uint32_t *sx, uint32_t *sy, uint32_t *sz,
                                 uint32_t *n)
{
    *n  = ulp_shared.sample_count;
    *sx = ulp_shared.sum_sq_x;
    *sy = ulp_shared.sum_sq_y;
    *sz = ulp_shared.sum_sq_z;

    ulp_shared.sum_sq_x     = 0;
    ulp_shared.sum_sq_y     = 0;
    ulp_shared.sum_sq_z     = 0;
    ulp_shared.sample_count = 0;
}

static void build_mfg_data(void)
{
    uint32_t sx, sy, sz, n;
    read_and_reset_accum(&sx, &sy, &sz, &n);

    uint16_t rms_x_mg = accel_rms_to_mg(sx, n, ADXL335_SENS_X);
    uint16_t rms_y_mg = accel_rms_to_mg(sy, n, ADXL335_SENS_Y);
    uint16_t rms_z_mg = accel_rms_to_mg(sz, n, ADXL335_SENS_Z);

    int16_t temp1 = raw_to_temp_x100((uint16_t)ulp_shared.last_raw_ntc1);
    int16_t temp2 = raw_to_temp_x100((uint16_t)ulp_shared.last_raw_ntc2);

    mfg_data[2]  = (uint8_t)((uint16_t)temp1 & 0xFF);
    mfg_data[3]  = (uint8_t)((uint16_t)temp1 >> 8);
    mfg_data[4]  = (uint8_t)((uint16_t)temp2 & 0xFF);
    mfg_data[5]  = (uint8_t)((uint16_t)temp2 >> 8);
    mfg_data[6]  = (uint8_t)(rms_x_mg & 0xFF);
    mfg_data[7]  = (uint8_t)(rms_x_mg >> 8);
    mfg_data[8]  = (uint8_t)(rms_y_mg & 0xFF);
    mfg_data[9]  = (uint8_t)(rms_y_mg >> 8);
    mfg_data[10] = (uint8_t)(rms_z_mg & 0xFF);
    mfg_data[11] = (uint8_t)(rms_z_mg >> 8);

    ESP_LOGI(TAG, "ADV  T1=%.2f T2=%.2f C  RMS X=%u Y=%u Z=%u mg | raw X=%d Y=%d Z=%d",
             temp1 / 100.0f, temp2 / 100.0f,
             rms_x_mg, rms_y_mg, rms_z_mg,
             ulp_shared.last_raw_x, ulp_shared.last_raw_y, ulp_shared.last_raw_z);
}

/* mfg_data → BLE advertising payload (adv_data 버퍼)로 인코딩 */
static int encode_adv_fields(uint8_t *adv_data, uint8_t *adv_len)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = (uint8_t)strlen(name);
    fields.name_is_complete = 1;

    fields.mfg_data = mfg_data;
    fields.mfg_data_len = sizeof(mfg_data);

    return ble_hs_adv_set_fields(&fields, adv_data, adv_len, BLE_HS_ADV_MAX_SZ);
}

void adv_cycle_task(void *arg)
{
    (void)arg;

    /* 1) 처음 1회: 첫 데이터 채워 광고 시작 */
    build_mfg_data();

    uint8_t adv_data[BLE_HS_ADV_MAX_SZ];
    uint8_t adv_len = 0;
    int rc = encode_adv_fields(adv_data, &adv_len);
    if (rc != 0) {
        ESP_LOGE(TAG, "encode_adv_fields failed: %d", rc);
        vTaskDelete(NULL);
        return;
    }

    rc = ble_gap_adv_set_data(adv_data, adv_len);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_data failed: %d", rc);
        vTaskDelete(NULL);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    /* 광고 인터벌은 기본값(약 100ms) 사용. 스캐너 친화적이면서 BLE
       컨트롤러가 인터벌 사이엔 modem sleep으로 들어감. */

    rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "ADV started (continuous)");

    /* 2) 이후: 1초마다 mfg_data 갱신 후 ble_gap_adv_set_data로 교체.
       광고는 멈추지 않으므로 스캐너에서 항상 보임. */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        build_mfg_data();
        rc = encode_adv_fields(adv_data, &adv_len);
        if (rc != 0) {
            ESP_LOGE(TAG, "encode_adv_fields failed: %d", rc);
            continue;
        }
        rc = ble_gap_adv_set_data(adv_data, adv_len);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_adv_set_data failed: %d", rc);
        }
    }
}
