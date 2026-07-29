/* ble/adv_manager.c — 적응형 광고 정책 상태머신. 설계: Docs/INGPS_1yr_firmware_design.md */
#include "adv_manager.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_bt.h"
#include "nvs.h"

#include "watchdog/wdt_guard.h"

static const char *TAG = "ADV_MGR";

/* === knob 기본값 ====================================================== */
#define ADVM_DEF_POLICY     1        /* 1=ADAPTIVE */
#define ADVM_DEF_MV_MS      1000
#define ADVM_DEF_ST_MS      3000
#define ADVM_DEF_QUIET_MS   300000   /* 5분 */
#define ADVM_DEF_MOT_MG     25       /* ADXL335 zero-보정 후 RMS 노이즈 플로어 위 */
#define ADVM_DEF_DT_X100    50       /* 0.5°C */
#define ADVM_SAFE_MS        10000
#define ADVM_ITVL_MAX_UNITS 0x4000   /* BLE 스펙 상한 10.24s */

static struct {
    uint8_t  policy;
    uint16_t mv_ms, st_ms;
    uint32_t quiet_ms;
    uint16_t mot_mg, dt_x100;
    uint8_t  name_in_adv;
    int8_t   tx_dbm;
} s_cfg;

static advm_state_t s_state;
static uint32_t s_cycle_ms;
static bool     s_itvl_changed;
static int64_t  s_last_active_us;
static int16_t  s_ref_t1, s_ref_t2;
static bool     s_ref_valid;

static uint32_t state_cycle_ms(advm_state_t st)
{
    switch (st) {
    case ADVM_SAFE:       return ADVM_SAFE_MS;
    case ADVM_STATIONARY: return s_cfg.st_ms;
    default:              return s_cfg.mv_ms;
    }
}

static void enter_state(advm_state_t st, const char *why)
{
    if (st == s_state) return;
    s_state = st;
    uint32_t new_cycle = state_cycle_ms(st);
    if (new_cycle != s_cycle_ms) {
        s_cycle_ms = new_cycle;
        s_itvl_changed = true;
    }
    ESP_LOGI(TAG, "state -> %d (%s), cycle=%ums", (int)st, why, (unsigned)s_cycle_ms);
}

void adv_manager_init(void)
{
    s_cfg.policy      = ADVM_DEF_POLICY;
    s_cfg.mv_ms       = ADVM_DEF_MV_MS;
    s_cfg.st_ms       = ADVM_DEF_ST_MS;
    s_cfg.quiet_ms    = ADVM_DEF_QUIET_MS;
    s_cfg.mot_mg      = ADVM_DEF_MOT_MG;
    s_cfg.dt_x100     = ADVM_DEF_DT_X100;
    s_cfg.name_in_adv = 1;
    s_cfg.tx_dbm      = 0;

    nvs_handle_t h;
    if (nvs_open("advm", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8 (h, "policy",   &s_cfg.policy);
        nvs_get_u16(h, "mv_ms",    &s_cfg.mv_ms);
        nvs_get_u16(h, "st_ms",    &s_cfg.st_ms);
        nvs_get_u32(h, "quiet_ms", &s_cfg.quiet_ms);
        nvs_get_u16(h, "mot_mg",   &s_cfg.mot_mg);
        nvs_get_u16(h, "dt_x100",  &s_cfg.dt_x100);
        nvs_get_u8 (h, "name",     &s_cfg.name_in_adv);
        nvs_get_i8 (h, "tx_dbm",   &s_cfg.tx_dbm);
        nvs_close(h);
    }
    /* 방어: 인터벌 하한 100ms(스펙 20ms지만 전력상 의미 없음), 상한 10.24s */
    if (s_cfg.mv_ms < 100)  s_cfg.mv_ms = 100;
    if (s_cfg.st_ms < s_cfg.mv_ms) s_cfg.st_ms = s_cfg.mv_ms;

    if (wdt_guard_safe_mode()) {
        s_state = ADVM_SAFE;
    } else if (s_cfg.policy == 0) {
        s_state = ADVM_FIXED;
    } else {
        s_state = ADVM_MOVING;   /* 부팅 직후엔 활성으로 시작(실시간성 우선) */
    }
    s_cycle_ms      = state_cycle_ms(s_state);
    s_itvl_changed  = false;
    s_last_active_us = esp_timer_get_time();
    s_ref_valid     = false;

    ESP_LOGI(TAG, "policy=%u state=%d mv=%u st=%u quiet=%u mot=%umg dT=%u name=%u tx=%ddBm",
             s_cfg.policy, (int)s_state, s_cfg.mv_ms, s_cfg.st_ms,
             (unsigned)s_cfg.quiet_ms, s_cfg.mot_mg, s_cfg.dt_x100,
             s_cfg.name_in_adv, (int)s_cfg.tx_dbm);
}

advm_state_t adv_manager_update(uint16_t rms_max_mg, int16_t t1_x100, int16_t t2_x100)
{
    if (s_state == ADVM_SAFE || s_state == ADVM_FIXED) {
        return s_state;   /* 고정 cadence — 전이 없음 */
    }

    int64_t now = esp_timer_get_time();
    bool motion = (rms_max_mg >= s_cfg.mot_mg);

    int32_t d1 = s_ref_valid ? (int32_t)t1_x100 - s_ref_t1 : 0;
    int32_t d2 = s_ref_valid ? (int32_t)t2_x100 - s_ref_t2 : 0;
    if (d1 < 0) d1 = -d1;
    if (d2 < 0) d2 = -d2;
    bool temp_event = s_ref_valid && ((uint32_t)d1 >= s_cfg.dt_x100 ||
                                      (uint32_t)d2 >= s_cfg.dt_x100);

    if (s_state == ADVM_MOVING) {
        /* 활성 중엔 기준온도를 계속 따라감(느린 드리프트는 이벤트 아님) */
        s_ref_t1 = t1_x100;
        s_ref_t2 = t2_x100;
        s_ref_valid = true;
        if (motion) {
            s_last_active_us = now;
        } else if (now - s_last_active_us >= (int64_t)s_cfg.quiet_ms * 1000) {
            enter_state(ADVM_STATIONARY, "quiet timeout");   /* 기준온도 동결 */
        }
    } else { /* ADVM_STATIONARY */
        if (motion || temp_event) {
            s_last_active_us = now;
            enter_state(ADVM_MOVING, motion ? "motion" : "deltaT");
        }
    }
    return s_state;
}

uint16_t adv_manager_itvl_units(void)
{
    uint32_t units = (s_cycle_ms * 8u) / 5u;   /* ms → 0.625ms 단위 */
    if (units > ADVM_ITVL_MAX_UNITS) units = ADVM_ITVL_MAX_UNITS;
    return (uint16_t)units;
}

uint32_t adv_manager_cycle_ms(void)  { return s_cycle_ms; }

bool adv_manager_take_itvl_changed(void)
{
    bool c = s_itvl_changed;
    s_itvl_changed = false;
    return c;
}

bool adv_manager_name_in_adv(void)   { return s_cfg.name_in_adv != 0; }

void adv_manager_apply_tx_power(void)
{
    /* 0dBm 기본: WBA52 감도(-96dBm@1M) 기준 30m 링크버짓 96dB.
       실측 30m RSSI < -80dBm이면 NVS advm/tx_dbm을 +3/+6으로 상향(설계문서 4절). */
    esp_power_level_t lvl;
    int8_t d = s_cfg.tx_dbm;
    if      (d <= -12) lvl = ESP_PWR_LVL_N12;
    else if (d <= -9)  lvl = ESP_PWR_LVL_N9;
    else if (d <= -6)  lvl = ESP_PWR_LVL_N6;
    else if (d <= -3)  lvl = ESP_PWR_LVL_N3;
    else if (d <= 0)   lvl = ESP_PWR_LVL_N0;
    else if (d <= 3)   lvl = ESP_PWR_LVL_P3;
    else if (d <= 6)   lvl = ESP_PWR_LVL_P6;
    else               lvl = ESP_PWR_LVL_P9;
    esp_err_t e1 = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, lvl);
    esp_err_t e2 = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, lvl);
    ESP_LOGI(TAG, "TX power %ddBm -> lvl=%d (adv=%d, def=%d)",
             (int)d, (int)lvl, (int)e1, (int)e2);
}

void adv_manager_enter_storage(void)
{
    ESP_LOGW(TAG, "STORAGE: adv off, deep sleep (~7uA). Wake = power cycle only.");
    /* 웨이크 소스 미설정 → 사실상 전원 재인가 전까지 딥슬립 유지 */
    esp_deep_sleep_start();
}
