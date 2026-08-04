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
#define ADVM_ITVL_MAX_UNITS 0x4000   /* BLE 스펙 상한 10.24s (0.625ms 단위) */
#define ADVM_ITVL_MAX_MS    10240

/* ★임시 실험용 — 광고 주기를 상태·NVS와 무관하게 고정한다(ms).
   0 = 정상 적응형 상태머신(MOVING 1s / STATIONARY 3s).
   값을 주면 policy를 FIXED로 강제해 모션·ΔT 전이를 막고 그 주기로 고정한다.
   NVS knob보다 뒤에 적용되므로 저장값이 있어도 확실히 덮어쓴다.
   ⚠ 상한 ADVM_ITVL_MAX_MS(10240) 이하여야 한다. 실험 후 0으로 되돌릴 것. */
#define ADVM_FORCE_CYCLE_MS 0

#ifndef ADVM_DEF_TX_DBM
#define ADVM_DEF_TX_DBM     9
#endif

#ifndef ADVM_MAX_TX_DBM
#define ADVM_MAX_TX_DBM     20
#endif
#define ADVM_MIN_TX_DBM     (-24)

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
    s_cfg.tx_dbm      = ADVM_DEF_TX_DBM;

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
    /* 하한 100ms(스펙은 20ms지만 전력상 의미 없음), 상한 10.24s(BLE 광고 스펙).
       상한을 여기서 걸지 않으면 cadence만 커지고 광고 인터벌은 itvl_units()에서
       10.24s로 잘려, 같은 페이로드가 여러 번 반복 송출되고 ADV stale 임계도
       함께 늘어난다. */
    if (s_cfg.mv_ms < 100)  s_cfg.mv_ms = 100;
    if (s_cfg.mv_ms > ADVM_ITVL_MAX_MS) s_cfg.mv_ms = ADVM_ITVL_MAX_MS;
    if (s_cfg.st_ms < s_cfg.mv_ms) s_cfg.st_ms = s_cfg.mv_ms;
    if (s_cfg.st_ms > ADVM_ITVL_MAX_MS) s_cfg.st_ms = ADVM_ITVL_MAX_MS;
    if (s_cfg.tx_dbm > ADVM_MAX_TX_DBM) s_cfg.tx_dbm = ADVM_MAX_TX_DBM;
    if (s_cfg.tx_dbm < ADVM_MIN_TX_DBM) s_cfg.tx_dbm = ADVM_MIN_TX_DBM;

#if ADVM_FORCE_CYCLE_MS
    /* NVS knob·기본값을 모두 덮어쓰고 고정 cadence로 강제한다(클램프 뒤에 적용). */
    s_cfg.policy = 0;                      /* FIXED — 모션/ΔT 전이 없음 */
    s_cfg.mv_ms  = ADVM_FORCE_CYCLE_MS;
    s_cfg.st_ms  = ADVM_FORCE_CYCLE_MS;
    ESP_LOGW(TAG, "FORCED adv cycle = %dms (실험 빌드, 상태 전이 비활성)",
             ADVM_FORCE_CYCLE_MS);
#endif

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

/* dBm ↔ esp_power_level_t 사다리 (오름차순).
   ESP32-S3는 esp32c3 계열 공용 esp_bt.h를 쓰며 N24..P20을 3dB 간격으로 제공.
   (sdkconfig의 CONFIG_BT_CTRL_DFT_TX_POWER_LEVEL_EFF=11 == P9와 일치.) */
typedef struct { int8_t dbm; esp_power_level_t lvl; } advm_pwr_step_t;
static const advm_pwr_step_t s_pwr_ladder[] = {
    { -24, ESP_PWR_LVL_N24 }, { -21, ESP_PWR_LVL_N21 },
    { -18, ESP_PWR_LVL_N18 }, { -15, ESP_PWR_LVL_N15 },
    { -12, ESP_PWR_LVL_N12 }, {  -9, ESP_PWR_LVL_N9  },
    {  -6, ESP_PWR_LVL_N6  }, {  -3, ESP_PWR_LVL_N3  },
    {   0, ESP_PWR_LVL_N0  }, {   3, ESP_PWR_LVL_P3  },
    {   6, ESP_PWR_LVL_P6  }, {   9, ESP_PWR_LVL_P9  },
    {  12, ESP_PWR_LVL_P12 }, {  15, ESP_PWR_LVL_P15 },
    {  18, ESP_PWR_LVL_P18 }, {  20, ESP_PWR_LVL_P20 },
};
#define ADVM_PWR_STEPS (sizeof(s_pwr_ladder)/sizeof(s_pwr_ladder[0]))

void adv_manager_apply_tx_power(void)
{
    /* 기본 +9dBm(= 기존 컨트롤러 기본값 유지). 커버리지가 모자라면 NVS
       advm/tx_dbm 또는 ADVM_DEF_TX_DBM으로 +12/+15/+18/+20까지 올릴 수 있다.
       ⚠ 올리기 전에 링크버짓부터 확인할 것: 30m 목표에서 초과손실이 20dB를
       넘으면 안테나/정합 문제이지 파워 문제가 아니며, 파워로는 못 메운다.
       ⚠ +9 초과는 TX 피크전류 급증 → LS14500 brownout 위험(BOD 설정 확인)과
       평균전류 상승(1년 목표 잠식)을 동반한다. */
    int8_t d = s_cfg.tx_dbm;

    /* 요청값 이하 중 가장 높은 단계 선택 */
    int idx = 0;
    for (int i = 0; i < (int)ADVM_PWR_STEPS; i++) {
        if (s_pwr_ladder[i].dbm <= d) idx = i;
    }

    /* 컨트롤러가 해당 단계를 거부하면(타깃/IDF에 따라 상위 단계 미지원 가능)
       한 단계씩 낮추며 재시도 — 무설정으로 남는 것보다 낫다. */
    esp_err_t err = ESP_FAIL;
    while (idx >= 0) {
        esp_power_level_t lvl = s_pwr_ladder[idx].lvl;
        err = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, lvl);
        if (err == ESP_OK) {
            esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, lvl);
            break;
        }
        ESP_LOGW(TAG, "TX power %ddBm rejected (err=%d), stepping down",
                 (int)s_pwr_ladder[idx].dbm, (int)err);
        idx--;
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TX power set failed entirely — controller default in use");
        return;
    }

    int8_t applied = s_pwr_ladder[idx].dbm;
    ESP_LOGI(TAG, "TX power: requested %ddBm, applied %ddBm (lvl=%d, readback=%d)",
             (int)d, (int)applied, (int)s_pwr_ladder[idx].lvl,
             (int)esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV));
    if (applied > 9) {
        ESP_LOGW(TAG, "TX >+9dBm: check BOD/brownout margin (LS14500 high-ESR)"
                      " and re-verify battery budget");
    }
}

void adv_manager_enter_storage(void)
{
    ESP_LOGW(TAG, "STORAGE: adv off, deep sleep (~7uA). Wake = power cycle only.");
    /* 웨이크 소스 미설정 → 사실상 전원 재인가 전까지 딥슬립 유지 */
    esp_deep_sleep_start();
}
