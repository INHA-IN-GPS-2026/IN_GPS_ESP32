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
#include "sensor/as6221.h"
#include "sensor/adxl345.h"
#include "watchdog/wdt_guard.h"
#include "adv_manager.h"

static const char *TAG = "BLE_ADV";

/* STM32 게이트웨이가 이 ESP를 구분하는 device ID. */
#define ESP_DEVICE_ID  0x01

/*
 * Manufacturer Specific Data (13바이트) — 게이트웨이/서버와의 계약.
 * 바이트 배치를 바꾸면 게이트웨이 파서와 temperature_log 스키마가 함께 깨진다.
 * ★I2C 전환에서도 이 레이아웃은 그대로다 — 온도 취득 경로만 바뀌었고
 *   게이트웨이/서버/앱은 손댈 필요가 없다.
 *   [0..1]   company ID (LE) = 0x1234
 *   [2..3]   temp1   (°C × 100, int16 LE)   AS6221 #1 (TH1, I2C)
 *   [4..5]   temp2   (°C × 100, int16 LE)   AS6221 #2 (TH2, I2C)
 *   [6..7]   rms_x   (mg, uint16 LE)   ADXL345 (I2C 0x53)
 *   [8..9]   rms_y   (mg, uint16 LE)
 *   [10..11] rms_z   (mg, uint16 LE)
 *   [12]     device ID
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

/* 최신 측정 스냅샷 — adv_manager 상태 전이 입력용.
   t1/t2는 "마지막으로 유효했던" 온도다. 읽기 실패 시의 센티넬
   (AS6221_TEMP_INVALID_X100 = -327.68°C)을 그대로 넣으면 다음 성공 읽기에서
   |ΔT|가 수백 도로 잡혀 STATIONARY→MOVING 오승격이 나고 배터리를 태운다.
   센티넬은 mfg_data(=서버가 NULL로 거를 값)에만 싣는다. */
typedef struct {
    uint16_t rms_max_mg;
    int16_t  t1_x100;
    int16_t  t2_x100;
} adv_meas_t;

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    if (event->type == BLE_GAP_EVENT_ADV_COMPLETE) {
        ESP_LOGI(TAG, "ADV complete, reason=%d", event->adv_complete.reason);
    }
    return 0;
}

static void build_mfg_data(adv_meas_t *out)
{
    /* ADXL345 FIFO(stream, 32-deep @100Hz)를 드레인해 RMS를 낸다.
       구 ULP 경로와 달리 창이 광고 주기 전체가 아니라 고정 320ms다 -
       정상상태 진동에서는 등가지만 3s 주기에서는 듀티 11%라 간헐 충격을
       놓칠 수 있다(sensor/adxl345.h 참조).
       실패하면 세 축 모두 0이 나가고, 드라이버가 60s 백오프로 버스를 보호한다. */
    uint16_t rms_x_mg = 0, rms_y_mg = 0, rms_z_mg = 0;
    uint32_t n = 0;
    bool accel_ok = adxl345_read_rms(&rms_x_mg, &rms_y_mg, &rms_z_mg, &n);
    if (accel_ok) {
        /* "센서가 실제로 표본을 냈다" 시점에만 kick - 구 ULP stall 감시를 대체한다. */
        wdt_guard_heartbeat(WDT_HB_ACCEL);
    }

    /* 온도는 AS6221이 자체 변환한 값을 그대로 읽는다. 아날로그 버전의
       창평균·eFuse INL 보정·Steinhart-Hart 변환은 전부 불필요해졌다
       (센서 자체 정확도 ±0.1°C, 분해능 1/128°C).
       읽기는 광고 사이클당 채널별 1회 — 버스 점유 ~1ms. */
    int16_t temp1 = AS6221_TEMP_INVALID_X100;
    int16_t temp2 = AS6221_TEMP_INVALID_X100;
    bool ok1 = as6221_read_x100(AS6221_CH_TH1, &temp1);
    bool ok2 = as6221_read_x100(AS6221_CH_TH2, &temp2);

    /* adv_manager에 넘길 "마지막 유효값". 부팅 후 한 번도 못 읽었으면 0을 유지해
       ΔT가 0으로 계산되게 한다(승격 없음). */
    static int16_t s_last_t1, s_last_t2;
    if (ok1) s_last_t1 = temp1;
    if (ok2) s_last_t2 = temp2;

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

    if (out) {
        uint16_t m = rms_x_mg;
        if (rms_y_mg > m) m = rms_y_mg;
        if (rms_z_mg > m) m = rms_z_mg;
        out->rms_max_mg = m;
        out->t1_x100    = s_last_t1;
        out->t2_x100    = s_last_t2;
    }

    /* 매 사이클 데이터 로그는 DEBUG — 운영 빌드에서 묵음. 진단 시
       esp_log_level_set("BLE_ADV", ESP_LOG_DEBUG)로 개방(전류↑ 유의). */
    ESP_LOGD(TAG, "RMS X=%u Y=%u Z=%u mg n=%u %s",
             rms_x_mg, rms_y_mg, rms_z_mg, (unsigned)n, accel_ok ? "ok" : "FAIL");
    ESP_LOGD(TAG, "T1=%.2fC(0x%02X %s) | T2=%.2fC(0x%02X %s)",
             temp1 / 100.0f, as6221_addr(AS6221_CH_TH1), ok1 ? "ok" : "FAIL",
             temp2 / 100.0f, as6221_addr(AS6221_CH_TH2), ok2 ? "ok" : "FAIL");
}

/* mfg_data → BLE advertising payload로 인코딩.
   이름 포함 여부는 knob(advm/name). 이름 제거 시 AD ~8B 단축 = TX 시간·전류 절감. */
static int encode_adv_fields(uint8_t *adv_data, uint8_t *adv_len)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    if (adv_manager_name_in_adv()) {
        const char *name = ble_svc_gap_device_name();
        fields.name = (uint8_t *)name;
        fields.name_len = (uint8_t)strlen(name);
        fields.name_is_complete = 1;
    }

    fields.mfg_data = mfg_data;
    fields.mfg_data_len = sizeof(mfg_data);

    return ble_hs_adv_set_fields(&fields, adv_data, adv_len, BLE_HS_ADV_MAX_SZ);
}

/* 현재 adv_manager 인터벌로 비연결·비스캔 광고 시작.
   itvl을 명시하지 않으면 NimBLE 기본 ~100ms로 나가 광고 에너지가 10배가 된다.
   disc_mode=NON: ADV_NONCONN_IND(비스캔) — 스캔요청 RX 윈도우 제거.
   (Flags AD의 GEN bit는 폰 스캐너 앱 표시 호환을 위해 유지.) */
static int adv_start_current(void)
{
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_NON;
    adv_params.itvl_min  = adv_manager_itvl_units();
    adv_params.itvl_max  = adv_manager_itvl_units();

    int rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER,
                               &adv_params, gap_event_cb, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "ADV started, itvl=%u units (%ums)",
                 adv_params.itvl_min, (unsigned)adv_manager_cycle_ms());
    }
    return rc;
}

/* pause/resume — nvs_safe_commit 전용. flash write와 TX 스파이크의 시간 분리용
   (BOD LVL7 ≈2.44V < 플래시 write 안전선 ~2.7V). s_adv_paused 동안 adv_cycle_task는
   인터벌 변경 재시작을 보류한다(itvl_changed 플래그는 소비하지 않고 남겨둬
   resume 후 다음 사이클에 반영). */
static volatile bool s_adv_paused;
static bool s_adv_ever_started;

int ble_adv_pause(void)
{
    s_adv_paused = true;
    if (!s_adv_ever_started) return 0;
    int rc = ble_gap_adv_stop();
    return (rc == BLE_HS_EALREADY) ? 0 : rc;
}

int ble_adv_resume(void)
{
    s_adv_paused = false;
    if (!s_adv_ever_started) return 0;
    int rc = adv_start_current();
    return (rc == BLE_HS_EALREADY) ? 0 : rc;
}

/* TWDT(8s)보다 짧은 조각으로 나눠 기다리며 feed — SAFE(10s) cadence에서도
   TWDT를 굶기지 않는다. 조각이 짧을수록 HP wake가 늘어나므로 여유 범위에서
   가장 크게 잡는다. */
#define ADV_WAIT_CHUNK_MS  4000

static void wait_cycle(uint32_t ms)
{
    while (ms > 0) {
        uint32_t chunk = ms > ADV_WAIT_CHUNK_MS ? ADV_WAIT_CHUNK_MS : ms;
        vTaskDelay(pdMS_TO_TICKS(chunk));
        wdt_guard_feed();
        ms -= chunk;
    }
}

void adv_cycle_task(void *arg)
{
    (void)arg;

    wdt_guard_task_subscribe();

    adv_meas_t meas;
    build_mfg_data(&meas);

    uint8_t adv_data[BLE_HS_ADV_MAX_SZ];
    uint8_t adv_len = 0;
    int rc = encode_adv_fields(adv_data, &adv_len);
    if (rc != 0) {
        ESP_LOGE(TAG, "encode_adv_fields failed: %d", rc);
        wdt_guard_reboot("adv encode failed at boot");
    }

    rc = ble_gap_adv_set_data(adv_data, adv_len);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_data failed: %d", rc);
        wdt_guard_reboot("adv set_data failed at boot");
    }

    rc = adv_start_current();
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        wdt_guard_reboot("adv start failed at boot");
    }
    s_adv_ever_started = true;
    wdt_guard_heartbeat(WDT_HB_ADV);
    wdt_guard_set_hb_stale(WDT_HB_ADV, adv_manager_cycle_ms() * 3 + 5000);
    /* ACCEL도 같은 cadence를 따라간다. 드라이버 백오프(60s)가 있어 하한이
       75s라 짧은 cadence에서는 그대로 75s가 쓰인다. */
    wdt_guard_set_hb_stale(WDT_HB_ACCEL, adv_manager_cycle_ms() * 3 + 5000);

    /* 매 사이클(상태머신 cadence)마다 mfg_data 갱신 + 상태 전이.
       인터벌이 바뀌면 adv stop→start로 재시작(광고 데이터는 유지됨). */
    while (1) {
        wait_cycle(adv_manager_cycle_ms());

        build_mfg_data(&meas);
        adv_manager_update(meas.rms_max_mg, meas.t1_x100, meas.t2_x100);

        if (!s_adv_paused && adv_manager_take_itvl_changed()) {
            ble_gap_adv_stop();
            rc = adv_start_current();
            if (rc != 0) {
                ESP_LOGE(TAG, "adv restart failed: %d", rc);
                wdt_guard_reboot("adv restart failed");
            }
            wdt_guard_set_hb_stale(WDT_HB_ADV, adv_manager_cycle_ms() * 3 + 5000);
            wdt_guard_set_hb_stale(WDT_HB_ACCEL, adv_manager_cycle_ms() * 3 + 5000);
        }

        rc = encode_adv_fields(adv_data, &adv_len);
        if (rc != 0) {
            ESP_LOGE(TAG, "encode_adv_fields failed: %d", rc);
            continue;
        }
        rc = ble_gap_adv_set_data(adv_data, adv_len);
        if (rc == 0) {
            wdt_guard_heartbeat(WDT_HB_ADV);   /* "실제 성공" 시점에만 */
        } else {
            ESP_LOGE(TAG, "ble_gap_adv_set_data failed: %d", rc);
        }
    }
}
