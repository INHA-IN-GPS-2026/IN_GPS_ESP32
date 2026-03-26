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

// [0..1] company ID (LE), [2..11] sensor data, [12] reason
static uint8_t mfg_data[] = {
    0x34, 0x12,   // company ID (LE) = 0x1234
    0x00, 0x00,   // temp1 (GPIO4) x100, int16 LE
    0x00, 0x00,   // temp2 (GPIO5) x100, int16 LE
    0x00, 0x00,   // angle_x x100, int16 LE
    0x00, 0x00,   // angle_y x100, int16 LE
    0x00, 0x00,   // angle_z x100, int16 LE
    0x00          // reason
};

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    if (event->type == BLE_GAP_EVENT_ADV_COMPLETE) {
        ESP_LOGI(TAG, "ADV complete, reason=%d", event->adv_complete.reason);
    }
    return 0;
}

static void start_advertising_once(void)
{
    int16_t temp1_x100 = raw_to_temp_x100((uint16_t)ulp_shared.rpt.last_raw[0]);
    int16_t temp2_x100 = raw_to_temp_x100((uint16_t)ulp_shared.rpt.last_raw[1]);

    int16_t raw_x = ulp_shared.rpt.last_raw[2];
    int16_t raw_y = ulp_shared.rpt.extra_raw[0];
    int16_t raw_z = ulp_shared.rpt.extra_raw[1];

    ESP_LOGI(TAG, "ACCEL raw  X=%d Y=%d Z=%d", raw_x, raw_y, raw_z);

    int16_t angle_x, angle_y, angle_z;
    raw_to_accel_angles(raw_x, raw_y, raw_z, &angle_x, &angle_y, &angle_z);

    mfg_data[2]  = (uint8_t)((uint16_t)temp1_x100 & 0xFF);
    mfg_data[3]  = (uint8_t)((uint16_t)temp1_x100 >> 8);
    mfg_data[4]  = (uint8_t)((uint16_t)temp2_x100 & 0xFF);
    mfg_data[5]  = (uint8_t)((uint16_t)temp2_x100 >> 8);
    mfg_data[6]  = (uint8_t)((uint16_t)angle_x & 0xFF);
    mfg_data[7]  = (uint8_t)((uint16_t)angle_x >> 8);
    mfg_data[8]  = (uint8_t)((uint16_t)angle_y & 0xFF);
    mfg_data[9]  = (uint8_t)((uint16_t)angle_y >> 8);
    mfg_data[10] = (uint8_t)((uint16_t)angle_z & 0xFF);
    mfg_data[11] = (uint8_t)((uint16_t)angle_z >> 8);

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

    ESP_LOGI(TAG, "ADV started len=%u, temp1=%.2f°C temp2=%.2f°C angle X=%.2f° Y=%.2f° Z=%.2f°",
             (unsigned)adv_len,
             temp1_x100 / 100.0f, temp2_x100 / 100.0f,
             angle_x / 100.0f, angle_y / 100.0f, angle_z / 100.0f);
}

void adv_cycle_task(void *arg)
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
