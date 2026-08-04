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
#include "sensor/adc_cal.h"
#include "ulp_shared.h"
#include "watchdog/wdt_guard.h"
#include "adv_manager.h"

static const char *TAG = "BLE_ADV";

extern volatile ulp_shared_t ulp_shared;

/* STM32 게이트웨이가 이 ESP를 구분하는 device ID. */
#define ESP_DEVICE_ID  0x01

/*
 * Manufacturer Specific Data (13바이트) — 게이트웨이/서버와의 계약.
 * 바이트 배치를 바꾸면 게이트웨이 파서와 temperature_log 스키마가 함께 깨진다.
 *   [0..1]   company ID (LE) = 0x1234
 *   [2..3]   temp1   (°C × 100, int16 LE)   GPIO3 NTC (TH1)
 *   [4..5]   temp2   (°C × 100, int16 LE)   GPIO4 NTC (TH2)
 *   [6..7]   rms_x   (mg, uint16 LE)
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

/* 최신 측정 스냅샷 — adv_manager 상태 전이 입력용 */
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

/* ULP 누적값을 읽고 0으로 리셋. 읽기/리셋 사이 ULP가 한 번 더 누적할 수 있으나
   200Hz 기준 최대 1샘플 손실이라 무시한다. */
static void read_and_reset_accum(uint32_t *sx, uint32_t *sy, uint32_t *sz,
                                 int32_t *dx, int32_t *dy, int32_t *dz,
                                 uint32_t *n)
{
    *n  = ulp_shared.sample_count;
    *sx = ulp_shared.sum_sq_x;
    *sy = ulp_shared.sum_sq_y;
    *sz = ulp_shared.sum_sq_z;
    *dx = ulp_shared.sum_dx_x;
    *dy = ulp_shared.sum_dx_y;
    *dz = ulp_shared.sum_dx_z;

    ulp_shared.sum_sq_x     = 0;
    ulp_shared.sum_sq_y     = 0;
    ulp_shared.sum_sq_z     = 0;
    ulp_shared.sum_dx_x     = 0;
    ulp_shared.sum_dx_y     = 0;
    ulp_shared.sum_dx_z     = 0;
    ulp_shared.sample_count = 0;
}

static void build_mfg_data(adv_meas_t *out)
{
    uint32_t sx, sy, sz, n;
    int32_t  dx, dy, dz;
    read_and_reset_accum(&sx, &sy, &sz, &dx, &dy, &dz, &n);

    uint16_t rms_x_mg = accel_rms_to_mg(sx, dx, n, ADXL335_SENS_X);
    uint16_t rms_y_mg = accel_rms_to_mg(sy, dy, n, ADXL335_SENS_Y);
    uint16_t rms_z_mg = accel_rms_to_mg(sz, dz, n, ADXL335_SENS_Z);

    /* NTC 창 평균을 ×16 고정소수로 계산해 sub-LSB 정보를 보존한다. 정수
       나눗셈(sum/n)은 소수부를 버리는데, 200~2000샘플 평균의 표준오차가
       0.3~0.7 LSB라 1 LSB 절삭이 그 정보를 통째로 삼킨다.
       ntc_count==0(부팅 직후 경계)이면 최신 순시값으로 폴백.
       오버플로: 최대 2000샘플 × 4095 × 16 = 1.31e8 < 2^32. */
    uint32_t ntc_n = ulp_shared.ntc_count;
    float raw1_f = ntc_n
        ? (float)((ulp_shared.sum_ntc1 * 16U) / ntc_n) / 16.0f
        : (float)ulp_shared.last_raw_ntc1;
    float raw2_f = ntc_n
        ? (float)((ulp_shared.sum_ntc2 * 16U) / ntc_n) / 16.0f
        : (float)ulp_shared.last_raw_ntc2;
    ulp_shared.sum_ntc1  = 0;
    ulp_shared.sum_ntc2  = 0;
    ulp_shared.ntc_count = 0;

    /* avg raw → adc_cal_raw_frac_to_mv (per-chip eFuse curve fitting) → 실전압
                → mv_to_resistance (분압 역산) → R → Steinhart-Hart → 온도.
       ULP는 raw만 누적하므로 per-chip 곡선 보정은 여기(메인 CPU)에서 적용한다.
       float 오버로드를 쓰는 이유는 위 창평균의 소수부를 온도까지 살리기 위함.
       adc_cal 미가용(eFuse 미소성)이면 내부에서 선형 폴백. */
    float v_ntc1 = adc_cal_raw_frac_to_mv(raw1_f);
    float v_ntc2 = adc_cal_raw_frac_to_mv(raw2_f);
    float r_ntc1 = mv_to_resistance(v_ntc1);
    float r_ntc2 = mv_to_resistance(v_ntc2);
    int16_t temp1 = resistance_to_temp_steinhart_x100(r_ntc1);
    int16_t temp2 = resistance_to_temp_steinhart_x100(r_ntc2);

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
        out->t1_x100    = temp1;
        out->t2_x100    = temp2;
    }

    /* 매 사이클 데이터 로그는 DEBUG — 운영 빌드에서 묵음. 진단 시
       esp_log_level_set("BLE_ADV", ESP_LOG_DEBUG)로 개방(전류↑ 유의). */
    ESP_LOGD(TAG, "RMS X=%u Y=%u Z=%u mg fs=%u ntc_n=%u",
             rms_x_mg, rms_y_mg, rms_z_mg, (unsigned)n, (unsigned)ntc_n);
    ESP_LOGD(TAG, "T1=%.2fC R=%.0f v=%.1fmV raw=%.2f | T2=%.2fC R=%.0f v=%.1fmV raw=%.2f cal=%d",
             temp1 / 100.0f, r_ntc1, v_ntc1, raw1_f,
             temp2 / 100.0f, r_ntc2, v_ntc2, raw2_f,
             (int)adc_cal_is_enabled());
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
