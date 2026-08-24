#include "sensor/as6221.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "nvs.h"

#include "sensor/i2c_bus.h"

static const char *TAG = "AS6221";

/* === 데이터시트 상수 (DS000751 v5-00) ================================= */

/* 인덱스(포인터) 레지스터 값 — 하위 2비트만 유효 */
#define AS6221_REG_TVAL     0x00   /* RO, 온도 (16-bit, MSB first) */
#define AS6221_REG_CONFIG   0x01
#define AS6221_REG_TLOW     0x02
#define AS6221_REG_THIGH    0x03

/* CONFIG 비트: 5=AL(RO) 6=CR0 7=CR1 8=SM 9=IM 10=POL 11=CF0 12=CF1 14=RO(1) 15=SS */
#define AS6221_CFG_RO_ONES  0x4020u   /* 읽으면 항상 1로 오는 RO 비트(14, 5) */
#define AS6221_CFG_CR_SHIFT 6

/* 변환율 CR[1:0]: 0=0.25/s 1=1/s 2=4/s(POR 기본) 3=8/s.
   1/s를 고르는 이유: 광고 cadence가 MOVING 1s / STATIONARY 3s라, 4s 주기(CR=0)면
   adv_manager의 ΔT 승격이 최대 4초 늦어진다. 반대로 4/s(기본)는 전류만 더 쓴다.
   데이터시트 기준 4 conv/s에서 typ 6µA이고 그보다 낮은 변환율은 더 적다. */
#ifndef AS6221_CONV_RATE_CR
#define AS6221_CONV_RATE_CR  1
#endif

/* SM=0(연속변환), IM=0(comparator), POL=0, CF=00, SS=0.
   ALERT을 쓰지 않으므로 TLOW/THIGH는 POR 기본값(75/80°C) 그대로 둔다. */
#define AS6221_CFG_VALUE \
    (AS6221_CFG_RO_ONES | ((AS6221_CONV_RATE_CR & 0x3u) << AS6221_CFG_CR_SHIFT))

/* 주소 후보 범위. ADD0/ALERT-ADD1 배선에 따라 0x44~0x4B 중 하나가 된다.
   ALERT을 VDD로 풀업해 쓰는 구성(우리 보드처럼 ALERT_TH1/2가 MCU로 나온 경우)은
   0x48~0x4B 그룹이다. 실제 배선은 J6 바깥 모듈에 있어 스키매틱만으로는 모른다. */
#define AS6221_ADDR_MIN     0x44
#define AS6221_ADDR_MAX     0x4B

#ifndef AS6221_DEF_ADDR_TH1
#define AS6221_DEF_ADDR_TH1  0x48
#endif
#ifndef AS6221_DEF_ADDR_TH2
#define AS6221_DEF_ADDR_TH2  0x49
#endif

/* === 버스 파라미터 ==================================================== */

/* rev 4.0부터 버스 속도/타임아웃은 sensor/i2c_bus.h가 단일 출처다 —
   ADXL345/BQ35100과 같은 버스를 공유하므로 드라이버마다 다른 값을 쓰면
   i2c_master_bus_add_device()가 클럭을 마지막 값으로 덮어쓴다.
     INGPS_I2C_SCL_HZ = 100000  (외부 풀업 10k라 400kHz 불가)
     INGPS_I2C_TMO_MS = 50 */

/* 연속 실패 임계 및 재시도 유예. 센서가 물리적으로 빠졌을 때 매 사이클
   100ms를 버리지 않도록 잠시 접근을 끊는다. */
#define AS6221_FAIL_LIMIT    3
#define AS6221_RETRY_AFTER_US  (60 * 1000000LL)

/* === 상태 ============================================================= */

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t  addr;
    bool     present;
    uint8_t  fails;
    int64_t  retry_at_us;
} as6221_ch_t;

static as6221_ch_t s_ch[AS6221_CH_COUNT];

/* === 내부 헬퍼 ======================================================== */

/* 인덱스 레지스터를 쓰고 16-bit 워드를 읽는다(MSB first).
   AS6221은 인덱스 레지스터를 write로 먼저 지정해야 하므로 write-then-read.
   인덱스는 마지막 값을 유지하지만, 매번 명시해 TVAL 이외 레지스터를 읽은 뒤에도
   안전하도록 한다. */
static esp_err_t reg_read16(as6221_ch_t *c, uint8_t reg, uint16_t *out)
{
    uint8_t rx[2];
    esp_err_t err = i2c_master_transmit_receive(c->dev, &reg, 1, rx, 2,
                                                INGPS_I2C_TMO_MS);
    if (err == ESP_OK) {
        *out = ((uint16_t)rx[0] << 8) | rx[1];
    }
    return err;
}

static esp_err_t reg_write16(as6221_ch_t *c, uint8_t reg, uint16_t val)
{
    uint8_t tx[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2c_master_transmit(c->dev, tx, sizeof(tx), INGPS_I2C_TMO_MS);
}

/* raw(1 LSB = 1/128 °C, 2의 보수) → °C×100.
   x100 = raw × 100 / 128 = raw × 25 / 32. |raw| ≤ 16000(=125°C)이므로
   raw×25 ≤ 4.0e5 로 int32 안전 — float 없이 반올림까지 처리한다.
   검산: 25.00°C = 0x0C80 = 3200 → (3200×25+16)/32 = 2500. */
static int16_t raw_to_x100(uint16_t raw)
{
    int32_t v = (int16_t)raw;                    /* 2의 보수 → 부호 확장 */
    int32_t n = v * 25 + (v >= 0 ? 16 : -16);
    return (int16_t)(n / 32);
}

/* 채널에 주소를 붙이고 CONFIG를 기록한다. 실패하면 핸들을 되돌린다. */
static bool ch_attach(as6221_ch_t *c, uint8_t addr)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = INGPS_I2C_SCL_HZ,
    };
    if (i2c_master_bus_add_device(ingps_i2c_bus(), &dev_cfg, &c->dev) != ESP_OK) {
        c->dev = NULL;
        return false;
    }
    c->addr = addr;

    esp_err_t err = reg_write16(c, AS6221_REG_CONFIG, AS6221_CFG_VALUE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "0x%02X config write failed: %s", addr, esp_err_to_name(err));
        i2c_master_bus_rm_device(c->dev);
        c->dev  = NULL;
        c->addr = 0;
        return false;
    }

    /* 기록이 실제로 먹었는지 되읽어 확인한다. 주소만 ACK하는 다른 부품이
       물려 있어도 여기서 걸러진다 — "붙긴 했는데 값이 이상하다"보다 낫다. */
    uint16_t cfg = 0;
    if (reg_read16(c, AS6221_REG_CONFIG, &cfg) != ESP_OK ||
        (cfg & ~AS6221_CFG_RO_ONES) != (AS6221_CFG_VALUE & ~AS6221_CFG_RO_ONES)) {
        ESP_LOGW(TAG, "0x%02X config readback mismatch (got 0x%04X, want 0x%04X)"
                      " -> AS6221이 아닐 수 있음", addr, cfg, AS6221_CFG_VALUE);
        i2c_master_bus_rm_device(c->dev);
        c->dev  = NULL;
        c->addr = 0;
        return false;
    }

    c->present     = true;
    c->fails       = 0;
    c->retry_at_us = 0;
    ESP_LOGI(TAG, "ch%d attached at 0x%02X (cfg=0x%04X, CR=%d)",
             (int)(c - s_ch), addr, cfg, AS6221_CONV_RATE_CR);
    return true;
}

static void load_addr_knobs(uint8_t *a1, uint8_t *a2)
{
    *a1 = AS6221_DEF_ADDR_TH1;
    *a2 = AS6221_DEF_ADDR_TH2;

    nvs_handle_t h;
    if (nvs_open("as6221", NVS_READONLY, &h) != ESP_OK) return;
    nvs_get_u8(h, "addr1", a1);
    nvs_get_u8(h, "addr2", a2);
    nvs_close(h);
}

/* === 공개 API ========================================================= */

bool as6221_init(void)
{
    memset(s_ch, 0, sizeof(s_ch));

    /* rev 4.0: 버스는 sensor/i2c_bus.c가 이미 만들어 뒀다(GPIO8/9 단일 버스).
       여기서 i2c_new_master_bus()를 다시 부르면 ESP_ERR_INVALID_STATE로 실패한다. */
    if (ingps_i2c_bus() == NULL) {
        ESP_LOGE(TAG, "I2C 버스 없음 - ingps_i2c_bus_init()을 먼저 호출할 것");
        return false;
    }

    uint8_t want1, want2;
    load_addr_knobs(&want1, &want2);

    /* 0x44~0x4B 전수 스캔. 실패해도 로그가 남아야 다음 판단이 가능하다 —
       "온도가 전부 NULL"일 때 센서가 없는 건지 주소가 틀린 건지 여기서 갈린다. */
    uint8_t found[AS6221_ADDR_MAX - AS6221_ADDR_MIN + 1];
    int n_found = 0;
    for (uint8_t a = AS6221_ADDR_MIN; a <= AS6221_ADDR_MAX; a++) {
        if (i2c_master_probe(ingps_i2c_bus(), a, INGPS_I2C_TMO_MS) == ESP_OK) {
            found[n_found++] = a;
        }
    }
    if (n_found == 0) {
        ESP_LOGE(TAG, "scan 0x%02X~0x%02X: 응답 없음 - 센서 미연결/전원/풀업 확인."
                      " ADXL345(0x53)가 붙었다면 버스 자체는 살아 있는 것이다",
                 AS6221_ADDR_MIN, AS6221_ADDR_MAX);
        return false;
    }
    for (int i = 0; i < n_found; i++) {
        ESP_LOGI(TAG, "scan: 0x%02X responded", found[i]);
    }

    /* 1순위: 원하는 주소가 스캔에 잡혔으면 그걸로 붙인다. */
    const uint8_t want[AS6221_CH_COUNT] = { want1, want2 };
    bool taken[AS6221_ADDR_MAX - AS6221_ADDR_MIN + 1] = { false };

    for (int ch = 0; ch < AS6221_CH_COUNT; ch++) {
        for (int i = 0; i < n_found; i++) {
            if (taken[i] || found[i] != want[ch]) continue;
            if (ch_attach(&s_ch[ch], found[i])) taken[i] = true;
            break;
        }
    }

    /* 2순위: 못 붙은 채널에 남은 응답 주소를 순서대로 배정한다.
       배선을 모르는 첫 브링업에서 조용히 죽지 않게 하려는 폴백이다. */
    for (int ch = 0; ch < AS6221_CH_COUNT; ch++) {
        if (s_ch[ch].present) continue;
        for (int i = 0; i < n_found; i++) {
            if (taken[i]) continue;
            if (ch_attach(&s_ch[ch], found[i])) {
                taken[i] = true;
                ESP_LOGW(TAG, "ch%d: 기대 주소 0x%02X 미응답 -> 0x%02X로 폴백 배정."
                              " 확정되면 NVS as6221/addr%d 또는 AS6221_DEF_ADDR_TH%d를"
                              " 고칠 것", ch, want[ch], found[i], ch + 1, ch + 1);
                break;
            }
        }
    }

    int n_ok = 0;
    for (int ch = 0; ch < AS6221_CH_COUNT; ch++) if (s_ch[ch].present) n_ok++;
    if (n_ok < AS6221_CH_COUNT) {
        ESP_LOGW(TAG, "%d/%d 채널만 확보 — 나머지는 temp=NULL로 광고된다",
                 n_ok, AS6221_CH_COUNT);
    }
    return n_ok > 0;
}

bool as6221_read_x100(int ch, int16_t *out_x100)
{
    if (out_x100) *out_x100 = AS6221_TEMP_INVALID_X100;
    if (ch < 0 || ch >= AS6221_CH_COUNT) return false;

    as6221_ch_t *c = &s_ch[ch];
    if (c->dev == NULL) return false;

    /* 백오프 중이면 버스를 건드리지 않는다. */
    if (!c->present && esp_timer_get_time() < c->retry_at_us) return false;

    uint16_t raw = 0;
    esp_err_t err = reg_read16(c, AS6221_REG_TVAL, &raw);
    if (err != ESP_OK) {
        if (c->fails < AS6221_FAIL_LIMIT) c->fails++;
        if (c->fails >= AS6221_FAIL_LIMIT && c->present) {
            c->present     = false;
            c->retry_at_us = esp_timer_get_time() + AS6221_RETRY_AFTER_US;
            ESP_LOGW(TAG, "ch%d (0x%02X) %d회 연속 실패(%s) -> %llds 후 재시도",
                     ch, c->addr, AS6221_FAIL_LIMIT, esp_err_to_name(err),
                     (long long)(AS6221_RETRY_AFTER_US / 1000000));
        } else if (!c->present) {
            /* 백오프 만료 후 재시도도 실패 — 다음 창까지 다시 미룬다. */
            c->retry_at_us = esp_timer_get_time() + AS6221_RETRY_AFTER_US;
        }
        return false;
    }

    if (!c->present) {
        ESP_LOGI(TAG, "ch%d (0x%02X) 복구", ch, c->addr);
    }
    c->present = true;
    c->fails   = 0;

    if (out_x100) *out_x100 = raw_to_x100(raw);
    return true;
}

uint8_t as6221_addr(int ch)
{
    if (ch < 0 || ch >= AS6221_CH_COUNT) return 0;
    return s_ch[ch].addr;
}

bool as6221_present(int ch)
{
    if (ch < 0 || ch >= AS6221_CH_COUNT) return false;
    return s_ch[ch].present;
}
