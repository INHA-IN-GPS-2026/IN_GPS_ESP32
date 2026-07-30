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

/* STM32 게이트웨이가 이 ESP를 구분하는 device ID (esp_test 고정값). */
#define ESP_DEVICE_ID  0x01

/*
 * Manufacturer Specific Data (13바이트)
 *   [0..1]   company ID (LE) = 0x1234
 *   [2..3]   temp1   (°C × 100, int16 LE)   GPIO3 NTC (TH1)
 *   [4..5]   temp2   (°C × 100, int16 LE)   GPIO4 NTC (TH2)
 *   [6..7]   rms_x   (mg, uint16 LE)
 *   [8..9]   rms_y   (mg, uint16 LE)
 *   [10..11] rms_z   (mg, uint16 LE)
 *   [12]     device ID (esp_test 고정값)
 *
 * ★★★ 임시(2026-07-16, 사용자 요청으로 원복 예정) ★★★
 *   게이트웨이 MQTT 매핑 손대기 번거로워서, 지금은 위 [6..11]에 실제로
 *   rms_x/y/z 대신 avg_raw1/avg_raw2(NTC raw)를 실어보냄. 노트북 없이
 *   BLE 스캐너로 캘리브 raw값 바로 확인하려는 목적. 게이트웨이는 이 6바이트를
 *   여전히 "RMS"로 오해하고 MQTT로 흘려보내지만 이 기간 동안은 무시.
 *   [6..7]   raw1_int  (avg_raw1 정수부, uint16 LE)
 *   [8..9]   raw2_int  (avg_raw2 정수부, uint16 LE)
 *   [10]     raw1_frac (avg_raw1 소수부×100, 0~99, uint8)
 *   [11]     raw2_frac (avg_raw2 소수부×100, 0~99, uint8)
 *   되돌릴 때: build_mfg_data() 안의 "★임시" 블록 삭제하고 그 위에 주석 처리된
 *   원래 rms_x/y/z 대입 블록 주석만 해제하면 됨.
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

/* ULP 누적값을 읽고 0으로 리셋 (최대 1샘플 race 손실은 무시).
   분산 계산용 dx 합(dx/dy/dz)도 함께 읽어 리셋. */
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

    /* NTC 창 평균: 지난 사이클 동안 ULP가 누적한 raw를 평균내 온도로 변환.
       ntc_count==0(부팅 직후 경계)이면 최신 순시값으로 폴백. 최대 1샘플 race 손실은 무시.
       순시 1샘플 대신 창 평균 → 온도 지터 √N 감소, 추가 지연 없음.
       (STATIONARY 3s 창이면 N~600 → 노이즈는 오히려 더 줄어든다.) */
    /* ★창 평균을 ×16 고정소수로 계산해 sub-LSB 정보를 보존한다.
       정수 나눗셈(sum/n)은 평균이 힘들게 얻은 소수부를 통째로 버린다 —
       200~2000샘플 평균의 표준오차가 0.3~0.7 LSB인데 1 LSB로 절삭하면
       S-curve 잔차를 볼 수 없다.
       오버플로: 최대 2000샘플 × 4095 × 16 = 1.31e8 < 2^32. 안전. */
    uint32_t ntc_n = ulp_shared.ntc_count;
    float raw1_f = ntc_n
        ? (float)((ulp_shared.sum_ntc1 * 16U) / ntc_n) / 16.0f
        : (float)ulp_shared.last_raw_ntc1;
    float raw2_f = ntc_n
        ? (float)((ulp_shared.sum_ntc2 * 16U) / ntc_n) / 16.0f
        : (float)ulp_shared.last_raw_ntc2;
    uint16_t avg_ntc1 = (uint16_t)(raw1_f + 0.5f);
    uint16_t avg_ntc2 = (uint16_t)(raw2_f + 0.5f);
    ulp_shared.sum_ntc1  = 0;
    ulp_shared.sum_ntc2  = 0;
    ulp_shared.ntc_count = 0;

    /* INL 보정 파이프라인:
         avg raw → adc_cal_raw_to_mv (per-chip eFuse curve fitting) → 실전압(mV)
                 → mv_to_resistance (분압 역산) → R → Steinhart-Hart → 온도.
       ULP는 raw만 누적하므로 per-chip 곡선 보정은 여기(메인 CPU)에서 적용한다.
       adc_cal 미가용(eFuse 미소성)이면 내부에서 선형 폴백(INL 미보정)으로 동작. */
    float v_ntc1 = adc_cal_raw_to_mv(avg_ntc1);
    float v_ntc2 = adc_cal_raw_to_mv(avg_ntc2);
    float r_ntc1 = mv_to_resistance(v_ntc1);
    float r_ntc2 = mv_to_resistance(v_ntc2);
    int16_t temp1 = resistance_to_temp_steinhart_x100(r_ntc1);
    int16_t temp2 = resistance_to_temp_steinhart_x100(r_ntc2);

    mfg_data[2]  = (uint8_t)((uint16_t)temp1 & 0xFF);
    mfg_data[3]  = (uint8_t)((uint16_t)temp1 >> 8);
    mfg_data[4]  = (uint8_t)((uint16_t)temp2 & 0xFF);
    mfg_data[5]  = (uint8_t)((uint16_t)temp2 >> 8);

    /* --- 원래 rms_x/y/z 대입 (원복 시 이 주석만 해제) ---
    mfg_data[6]  = (uint8_t)(rms_x_mg & 0xFF);
    mfg_data[7]  = (uint8_t)(rms_x_mg >> 8);
    mfg_data[8]  = (uint8_t)(rms_y_mg & 0xFF);
    mfg_data[9]  = (uint8_t)(rms_y_mg >> 8);
    mfg_data[10] = (uint8_t)(rms_z_mg & 0xFF);
    mfg_data[11] = (uint8_t)(rms_z_mg >> 8);
    */

    /* ★임시(2026-07-16): [6..11]을 avg_raw1/avg_raw2로 갈아끼움.
       BLE 스캐너에서 "RMS X/Y" 자리에 raw 정수부, "RMS Z" 자리에 두 소수부가
       보인다. 소수부는 ×100(0~99)이라 분해능 0.01 LSB — 창평균(200~2000샘플)의
       표준오차 0.3~0.7 LSB보다 충분히 곱다. 원복 방법은 위 헤더 주석 참고. */
    uint16_t raw1_int  = (uint16_t)raw1_f;
    uint16_t raw2_int  = (uint16_t)raw2_f;
    uint8_t  raw1_frac = (uint8_t)((raw1_f - (float)raw1_int) * 100.0f);
    uint8_t  raw2_frac = (uint8_t)((raw2_f - (float)raw2_int) * 100.0f);
    if (raw1_frac > 99) raw1_frac = 99;
    if (raw2_frac > 99) raw2_frac = 99;
    mfg_data[6]  = (uint8_t)(raw1_int & 0xFF);
    mfg_data[7]  = (uint8_t)(raw1_int >> 8);
    mfg_data[8]  = (uint8_t)(raw2_int & 0xFF);
    mfg_data[9]  = (uint8_t)(raw2_int >> 8);
    mfg_data[10] = raw1_frac;
    mfg_data[11] = raw2_frac;
    (void)rms_x_mg; (void)rms_y_mg; (void)rms_z_mg;

    if (out) {
        uint16_t m = rms_x_mg;
        if (rms_y_mg > m) m = rms_y_mg;
        if (rms_z_mg > m) m = rms_z_mg;
        out->rms_max_mg = m;
        out->t1_x100    = temp1;
        out->t2_x100    = temp2;
    }

    /* 매 사이클 데이터 로그는 DEBUG로 — 운영 빌드(log level NONE~INFO)에서 묵음.
       진단 시 esp_log_level_set("BLE_ADV", ESP_LOG_DEBUG)로 개방(전류↑ 유의). */
    ESP_LOGD(TAG, "ADV RMS X=%u Y=%u Z=%u mg fs=%u ntc_n=%u id=%u",
             rms_x_mg, rms_y_mg, rms_z_mg, (unsigned)n, (unsigned)ntc_n, ESP_DEVICE_ID);
    ESP_LOGD(TAG, "T1=%.2fC R=%.0f v=%.1fmV | T2=%.2fC R=%.0f v=%.1fmV cal=%d",
             temp1 / 100.0f, r_ntc1, v_ntc1, temp2 / 100.0f, r_ntc2, v_ntc2,
             (int)adc_cal_is_enabled());
    /* ★임시(2026-07-16): UART가 연결돼 있을 땐 BLE 스캐너로 읽은 값과
       대조할 수 있게 raw를 그대로 한 줄 더 찍는다. 원복 시 같이 삭제. */
    ESP_LOGD(TAG, "EXP raw1=%.2f raw2=%.2f (n=%u)",
             raw1_f, raw2_f, (unsigned)ntc_n);
}

/* mfg_data → BLE advertising payload (adv_data 버퍼)로 인코딩.
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
   ★Analog 1.0.0 핵심 수정: 기존 코드는 itvl 미지정 → NimBLE 기본 ~100ms로
   초당 10회 송출되고 있었다(광고 에너지 10배). 이제 itvl_min=itvl_max를
   상태머신이 준 값(1s/3s/10s)으로 명시한다.
   disc_mode=NON: ADV_NONCONN_IND(비스캔) — 스캔요청 RX 윈도우 제거.
   (Flags AD의 GEN bit는 유지 — 폰 스캐너 앱 표시 호환.) */
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

/* TWDT(8s)보다 짧은 조각으로 나눠 기다리며 feed — STATIONARY(3s)/SAFE(10s)
   cadence에서도 TWDT를 굶기지 않는다. */
static void wait_cycle(uint32_t ms)
{
    while (ms > 0) {
        uint32_t chunk = ms > 2000 ? 2000 : ms;
        vTaskDelay(pdMS_TO_TICKS(chunk));
        wdt_guard_feed();
        ms -= chunk;
    }
}

void adv_cycle_task(void *arg)
{
    (void)arg;

    wdt_guard_task_subscribe();   /* TWDT 등록 (L2) */

    /* 1) 처음 1회: 첫 데이터 채워 광고 시작 */
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
    wdt_guard_heartbeat(WDT_HB_ADV);
    wdt_guard_set_hb_stale(WDT_HB_ADV,
                           adv_manager_cycle_ms() * 3 + 5000);

    /* 2) 이후: 매 사이클(상태머신 cadence)마다 mfg_data 갱신 + 상태 전이.
       인터벌이 바뀌면 adv stop→start로 재시작(광고 데이터는 유지됨). */
    while (1) {
        wait_cycle(adv_manager_cycle_ms());

        build_mfg_data(&meas);
        adv_manager_update(meas.rms_max_mg, meas.t1_x100, meas.t2_x100);

        if (adv_manager_take_itvl_changed()) {
            ble_gap_adv_stop();
            rc = adv_start_current();
            if (rc != 0) {
                ESP_LOGE(TAG, "adv restart failed: %d", rc);
                wdt_guard_reboot("adv restart failed");
            }
            wdt_guard_set_hb_stale(WDT_HB_ADV,
                                   adv_manager_cycle_ms() * 3 + 5000);
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
