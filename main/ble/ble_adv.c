#include "ble_adv.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_hs_adv.h"
#include "services/gap/ble_svc_gap.h"

#include "sensor/sensor.h"
#include "ulp_shared.h"
#include "watchdog/wdt_guard.h"
#include "watchdog/wdt_test.h"

static const char *TAG = "BLE_ADV";

/* 딥슬립 버스트 광고 완료 신호용. ble_adv_init()에서 생성, gap_event_cb의
   ADV_COMPLETE에서 give, ble_adv_wait_burst_done()에서 take. */
static SemaphoreHandle_t s_adv_done_sem = NULL;

/* NTC 링버퍼 드레인 위치(2026-07-16, trimmed mean용). ring1/ring2는 매
   ULP 사이클마다 같은 인덱스로 함께 적재되므로 tail 하나를 공유해도 됨. */
static uint32_t s_ntc_tail = 0;

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

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    if (event->type == BLE_GAP_EVENT_ADV_COMPLETE) {
        ESP_LOGI(TAG, "ADV complete, reason=%d", event->adv_complete.reason);
        if (s_adv_done_sem != NULL) {
            xSemaphoreGive(s_adv_done_sem);
        }
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

static int cmp_i16(const void *a, const void *b)
{
    int16_t x = *(const int16_t *)a;
    int16_t y = *(const int16_t *)b;
    return (int)x - (int)y;
}

/* ★07-16: NTC 링버퍼를 드레인해 채널별 trimmed mean을 한 번에 계산.
   ring1/ring2는 같은 head를 공유(매 사이클 동시 적재)하므로 tail도 하나로
   같이 관리. 오버런(미드레인 덮어쓰기) 시 ADXL 캡처와 동일하게 최신
   NTC_RING_LEN 구간으로 점프. count=0이면 last_raw로 폴백. */
static void ntc_trimmed_mean_pair(uint32_t head, uint32_t *tail,
                                   float *out1, float *out2, uint32_t *out_n,
                                   int16_t fallback1, int16_t fallback2)
{
    if ((uint32_t)(head - *tail) > NTC_RING_LEN) {
        *tail = head - NTC_RING_LEN;
    }
    int n = (int)(head - *tail);
    if (n <= 0) {
        *out1  = (float)fallback1;
        *out2  = (float)fallback2;
        *out_n = 0;
        return;
    }
    if (n > NTC_RING_LEN) {
        n = NTC_RING_LEN;   /* 안전 클램프 */
    }

    static int16_t tmp1[NTC_RING_LEN];
    static int16_t tmp2[NTC_RING_LEN];
    for (int i = 0; i < n; i++) {
        uint32_t idx = (*tail + (uint32_t)i) & NTC_RING_MASK;
        tmp1[i] = ulp_shared.ntc_ring1[idx];
        tmp2[i] = ulp_shared.ntc_ring2[idx];
    }
    *tail += (uint32_t)n;

    qsort(tmp1, (size_t)n, sizeof(tmp1[0]), cmp_i16);
    qsort(tmp2, (size_t)n, sizeof(tmp2[0]), cmp_i16);

    int trim = (int)((float)n * NTC_TRIM_FRAC);
    int lo = trim;
    int hi = n - trim;
    if (hi <= lo) {   /* n이 너무 작으면 trim 생략 */
        lo = 0;
        hi = n;
    }

    double s1 = 0.0, s2 = 0.0;
    for (int i = lo; i < hi; i++) {
        s1 += tmp1[i];
        s2 += tmp2[i];
    }
    int cnt = hi - lo;
    *out1  = (float)(s1 / cnt);
    *out2  = (float)(s2 / cnt);
    *out_n = (uint32_t)n;
}

static void build_mfg_data(void)
{
    uint32_t sx, sy, sz, n;
    read_and_reset_accum(&sx, &sy, &sz, &n);

    uint16_t rms_x_mg = accel_rms_to_mg(sx, n, ADXL335_SENS_X);
    uint16_t rms_y_mg = accel_rms_to_mg(sy, n, ADXL335_SENS_Y);
    uint16_t rms_z_mg = accel_rms_to_mg(sz, n, ADXL335_SENS_Z);

    /* ★07-16: NTC 오버샘플링을 sum/count 단순평균 대신 링버퍼 trimmed
       mean으로 교체. VCC dip(~112ms 주기·~1.3ms 폭, BLE TX 추정) 같은 짧은
       이상치가 1초 180여 샘플 중 1~2개만 오염시키므로, 상하위
       NTC_TRIM_FRAC(5%)씩 잘라내면 순수 소프트웨어로 이 이상치를 걸러낼
       수 있다(지속적 VCC 변동 자체는 못 잡음 — 그건 VCC센스 채널 필요,
       미구현). sum_ntc1/2·ntc_count는 오버플로 방지용으로 계속 리셋만
       하고 평균 계산엔 더 이상 안 씀. */
    uint32_t ring_head = ulp_shared.ntc_ring_head;
    float avg_raw1, avg_raw2;
    uint32_t ncnt = 0;
    ntc_trimmed_mean_pair(ring_head, &s_ntc_tail, &avg_raw1, &avg_raw2, &ncnt,
                          ulp_shared.last_raw_ntc1, ulp_shared.last_raw_ntc2);

    ulp_shared.sum_ntc1  = 0;
    ulp_shared.sum_ntc2  = 0;
    ulp_shared.ntc_count = 0;

    int16_t temp1 = raw_to_temp_x100(avg_raw1, 0);
    int16_t temp2 = raw_to_temp_x100(avg_raw2, 1);

    mfg_data[2]  = (uint8_t)((uint16_t)temp1 & 0xFF);
    mfg_data[3]  = (uint8_t)((uint16_t)temp1 >> 8);
    mfg_data[4]  = (uint8_t)((uint16_t)temp2 & 0xFF);
    mfg_data[5]  = (uint8_t)((uint16_t)temp2 >> 8);

    /* --- 원래(운영) 코드: 되돌릴 때 이 블록 주석 해제하고 밑의 ★임시 블록 삭제 ---
    mfg_data[6]  = (uint8_t)(rms_x_mg & 0xFF);
    mfg_data[7]  = (uint8_t)(rms_x_mg >> 8);
    mfg_data[8]  = (uint8_t)(rms_y_mg & 0xFF);
    mfg_data[9]  = (uint8_t)(rms_y_mg >> 8);
    mfg_data[10] = (uint8_t)(rms_z_mg & 0xFF);
    mfg_data[11] = (uint8_t)(rms_z_mg >> 8);
    ------------------------------------------------------------------------- */

    /* ★임시(2026-07-16): X/Y 자리에 avg_raw1/2 정수부, Z 자리에 두 소수부를
       각각 한 바이트씩(×100, 0~99) 압축해서 실음. rms_x/y/z_mg는 위에서 이미
       계산은 해뒀으니(로그용으로 계속 사용) 여기선 그냥 안 씀. */
    uint16_t raw1_int  = (uint16_t)avg_raw1;
    uint16_t raw2_int  = (uint16_t)avg_raw2;
    uint8_t  raw1_frac = (uint8_t)roundf((avg_raw1 - (float)raw1_int) * 100.0f);
    uint8_t  raw2_frac = (uint8_t)roundf((avg_raw2 - (float)raw2_int) * 100.0f);
    if (raw1_frac > 99) { raw1_frac = 99; }
    if (raw2_frac > 99) { raw2_frac = 99; }

    mfg_data[6]  = (uint8_t)(raw1_int & 0xFF);
    mfg_data[7]  = (uint8_t)(raw1_int >> 8);
    mfg_data[8]  = (uint8_t)(raw2_int & 0xFF);
    mfg_data[9]  = (uint8_t)(raw2_int >> 8);
    mfg_data[10] = raw1_frac;
    mfg_data[11] = raw2_frac;

    /* ★2026-07-18: [[NTC_RAW_DIAG]] 3초 주기 floating 재발 확인 끝(미재현 확인,
       재캘리브 불필요 결론) — ESP_LOGI→ESP_LOGD로 재하향. 브라운아웃 완화
       목적의 매초 UART 전류 절감 복구. 필요하면 이 블록 위 주석 패턴대로
       다시 임시 복귀 가능. */
    ESP_LOGD(TAG, "ADV  RMS X=%u Y=%u Z=%u mg device_id = %u",
             rms_x_mg, rms_y_mg, rms_z_mg, ESP_DEVICE_ID);
    ESP_LOGD(TAG, "therm T1=%.2f T2=%.2f C (avg_raw1=%.1f avg_raw2=%.1f n=%u | last %d,%d)",
             temp1 / 100.0f, temp2 / 100.0f,
             avg_raw1, avg_raw2, (unsigned)ncnt,
             (int)ulp_shared.last_raw_ntc1, (int)ulp_shared.last_raw_ntc2);
    ESP_LOGD(TAG, "raw: x=%d y=%d z=%d  zero=(%d,%d,%d)  dx=%d dy=%d dz=%d",
             ulp_shared.last_raw_x, ulp_shared.last_raw_y, ulp_shared.last_raw_z,
             ulp_shared.zero_x, ulp_shared.zero_y, ulp_shared.zero_z,
             ulp_shared.last_raw_x - ulp_shared.zero_x,
             ulp_shared.last_raw_y - ulp_shared.zero_y,
             ulp_shared.last_raw_z - ulp_shared.zero_z);
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

    /* ★2026-07-18: 이 태스크를 Task WDT에 등록. 루프(1s 주기)가 8s 이상
       멈추면(NimBLE 호출 블록 등) TWDT panic → 재부팅. 또한 광고 갱신이
       "성공"할 때마다 앱 heartbeat를 kick — 태스크는 도는데 BLE 스택이
       계속 실패하는 상태는 모니터가 15s 후 재부팅으로 복구. */
    wdt_guard_task_subscribe();

    /* 1) 처음 1회: 첫 데이터 채워 광고 시작 */
    build_mfg_data();

    uint8_t adv_data[BLE_HS_ADV_MAX_SZ];
    uint8_t adv_len = 0;
    int rc = encode_adv_fields(adv_data, &adv_len);
    if (rc != 0) {
        /* ★07-18: 예전엔 vTaskDelete로 조용히 죽어 디바이스가 "ULP만 도는
           벽돌"이 됐음 — 초기 광고 셋업 실패는 즉시 재부팅으로 복구. */
        ESP_LOGE(TAG, "encode_adv_fields failed: %d", rc);
        wdt_guard_reboot("initial encode_adv_fields failed");
    }

    rc = ble_gap_adv_set_data(adv_data, adv_len);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_data failed: %d", rc);
        wdt_guard_reboot("initial ble_gap_adv_set_data failed");
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
        wdt_guard_reboot("initial ble_gap_adv_start failed");
    }
    ESP_LOGI(TAG, "ADV started (continuous)");
    wdt_guard_heartbeat(WDT_HB_ADV);

    /* 2) 이후: 1초마다 mfg_data 갱신 후 ble_gap_adv_set_data로 교체.
       광고는 멈추지 않으므로 스캐너에서 항상 보임. */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        wdt_guard_feed();   /* TWDT: 루프 생존 신고 */
        wdt_test_tick();    /* 장애 주입 테스트 스위치(WDT_TEST_MODE, 운영=0 no-op) */

        build_mfg_data();
        rc = encode_adv_fields(adv_data, &adv_len);
        if (rc != 0) {
            /* 일시 실패는 다음 초에 재시도. heartbeat는 kick하지 않음 —
               15s 연속 실패면 모니터가 재부팅으로 복구(wdt_guard). */
            ESP_LOGE(TAG, "encode_adv_fields failed: %d", rc);
            continue;
        }
        rc = ble_gap_adv_set_data(adv_data, adv_len);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_adv_set_data failed: %d", rc);
        } else if (!wdt_test_suppress_heartbeat()) {
            wdt_guard_heartbeat(WDT_HB_ADV);   /* 갱신 성공 시에만 kick */
        }
    }
}

void ble_adv_init(void)
{
    if (s_adv_done_sem == NULL) {
        s_adv_done_sem = xSemaphoreCreateBinary();
    } else {
        /* 재사용 시 이전 give가 남아있지 않도록 비움 */
        xSemaphoreTake(s_adv_done_sem, 0);
    }
}

void adv_burst_start(uint32_t duration_ms)
{
    /* ULP가 측정창 동안 쌓은 값을 여기서 1회 읽고 리셋 — 딥슬립 사이클당 정확히
       한 번만 build되므로 상시광고 버전(1초마다 재빌드)과 달리 값이 안 바뀜. */
    build_mfg_data();

    uint8_t adv_data[BLE_HS_ADV_MAX_SZ];
    uint8_t adv_len = 0;
    int rc = encode_adv_fields(adv_data, &adv_len);
    if (rc != 0) {
        ESP_LOGE(TAG, "encode_adv_fields failed: %d", rc);
        goto fail_unblock;
    }

    rc = ble_gap_adv_set_data(adv_data, adv_len);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_data failed: %d", rc);
        goto fail_unblock;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    /* duration_ms 뒤 NimBLE이 스스로 광고를 멈추고 ADV_COMPLETE를 쏨(FOREVER
       대신 명시적 지속시간 지정) — 별도 타이머/태스크 없이 버스트 구현. */
    rc = ble_gap_adv_start(g_own_addr_type, NULL, (int32_t)duration_ms,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        goto fail_unblock;
    }
    ESP_LOGI(TAG, "ADV burst started (%u ms)", (unsigned)duration_ms);
    return;

fail_unblock:
    /* 광고 시작 자체가 실패해도 app_main이 세마포어 대기에서 영원히 안 걸리게
       바로 신호를 준다(그래도 이 사이클은 데이터 송신 실패). */
    if (s_adv_done_sem != NULL) {
        xSemaphoreGive(s_adv_done_sem);
    }
}

bool ble_adv_wait_burst_done(uint32_t timeout_ms)
{
    if (s_adv_done_sem == NULL) {
        return false;
    }
    return xSemaphoreTake(s_adv_done_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}
