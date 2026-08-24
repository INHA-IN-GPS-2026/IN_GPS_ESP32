#include "sensor/adxl345.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "sensor/i2c_bus.h"
#include "sensor/sensor.h"

static const char *TAG = "ADXL345";

/* === 데이터시트 상수 (Rev. E) ========================================== */

/* SDO/ALT_ADDRESS가 GND이므로 0x53 고정(회로도 확정). VDD였다면 0x1D. */
#define ADXL345_I2C_ADDR    0x53
#define ADXL345_DEVID_VAL   0xE5

#define REG_DEVID           0x00
#define REG_BW_RATE         0x2C
#define REG_POWER_CTL       0x2D
#define REG_INT_ENABLE      0x2E
#define REG_DATA_FORMAT     0x31
#define REG_DATAX0          0x32   /* 0x32~0x37, 자동 증가 */
#define REG_FIFO_CTL        0x38
#define REG_FIFO_STATUS     0x39

/* BW_RATE: bit4 = LOW_POWER, bits[3:0] = rate code.
   0x0A = 100Hz ODR(대역 50Hz). LOW_POWER는 rate code 0x07~0x0C에서만 유효하다. */
#define BW_LOW_POWER        0x10
#define BW_RATE_100HZ       0x0A
#define ADXL345_BW_RATE_VAL (BW_LOW_POWER | BW_RATE_100HZ)

/* POWER_CTL bit3 = Measure. 나머지(링크/자동슬립/웨이크업)는 0. */
#define PWR_MEASURE         0x08
#define PWR_STANDBY         0x00

/* DATA_FORMAT: bit3 FULL_RES, bits[1:0] range(11 = 16g).
   FULL_RES + 16g를 고르는 이유: FULL_RES에서는 레인지와 무관하게 3.9mg/LSB가
   유지되므로, 레인지를 최대로 두면 분해능 손해 없이 클리핑만 사라진다.
   설비 진동에서 순간 충격이 2g를 넘어도 포화되지 않는다. */
#define FMT_FULL_RES        0x08
#define FMT_RANGE_16G       0x03
#define ADXL345_FMT_VAL     (FMT_FULL_RES | FMT_RANGE_16G)

/* FIFO_CTL: bits[7:6] 모드(10 = Stream), bits[4:0] 워터마크 샘플 수.
   Stream은 가득 차면 가장 오래된 샘플을 밀어내므로, 드레인 시점에 항상
   "가장 최근 32샘플(=320ms)"이 들어 있다. 워터마크는 INT 미접속이라 무의미하나
   레지스터 기본값을 명시해 둔다. */
#define FIFO_MODE_STREAM    0x80
#define ADXL345_FIFO_VAL    (FIFO_MODE_STREAM | 0x1F)

/* FIFO_STATUS bits[5:0] = 현재 엔트리 수(0~32). bit7은 FIFO_TRIG. */
#define FIFO_ENTRIES_MASK   0x3F
#define ADXL345_FIFO_DEPTH  32

/* 연속 실패 임계 및 재시도 유예 — as6221.c와 같은 정책. */
#define ADXL345_FAIL_LIMIT      3
#define ADXL345_RETRY_AFTER_US  (60 * 1000000LL)

/* === 상태 ============================================================= */

static i2c_master_dev_handle_t s_dev;
static bool    s_present;
static uint8_t s_fails;
static int64_t s_retry_at_us;

/* === 내부 헬퍼 ======================================================== */

static esp_err_t reg_write8(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };
    return i2c_master_transmit(s_dev, tx, sizeof(tx), INGPS_I2C_TMO_MS);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len,
                                       INGPS_I2C_TMO_MS);
}

/* === 공개 API ========================================================= */

bool adxl345_init(void)
{
    s_dev         = NULL;
    s_present     = false;
    s_fails       = 0;
    s_retry_at_us = 0;

    i2c_master_bus_handle_t bus = ingps_i2c_bus();
    if (bus == NULL) {
        ESP_LOGE(TAG, "I2C 버스 없음 - ingps_i2c_bus_init()을 먼저 호출할 것");
        return false;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ADXL345_I2C_ADDR,
        .scl_speed_hz    = INGPS_I2C_SCL_HZ,
    };
    if (i2c_master_bus_add_device(bus, &dev_cfg, &s_dev) != ESP_OK) {
        ESP_LOGE(TAG, "device add 실패 (0x%02X)", ADXL345_I2C_ADDR);
        s_dev = NULL;
        return false;
    }

    /* DEVID 확인. 주소만 ACK하는 다른 부품이 물려 있어도 여기서 걸러진다 -
       AS6221 스캔 범위(0x44~0x4B)와 겹치지 않지만, 배선 실수로 엉뚱한 부품이
       0x53에 앉는 경우를 "값이 이상하다"보다 먼저 잡는 게 낫다. */
    uint8_t devid = 0;
    esp_err_t err = reg_read(REG_DEVID, &devid, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "0x%02X 무응답(%s) - 미실장/전원/풀업 확인",
                 ADXL345_I2C_ADDR, esp_err_to_name(err));
        goto fail;
    }
    if (devid != ADXL345_DEVID_VAL) {
        ESP_LOGE(TAG, "DEVID 불일치: got 0x%02X, want 0x%02X - ADXL345가 아니다",
                 devid, ADXL345_DEVID_VAL);
        goto fail;
    }

    /* 데이터시트 권고: 설정은 standby에서. measure 중 FIFO 모드를 바꾸면
       엔트리가 정의되지 않은 상태로 남을 수 있다. */
    if (reg_write8(REG_POWER_CTL,    PWR_STANDBY)         != ESP_OK) goto fail;
    if (reg_write8(REG_INT_ENABLE,   0x00)                != ESP_OK) goto fail;
    if (reg_write8(REG_BW_RATE,      ADXL345_BW_RATE_VAL) != ESP_OK) goto fail;
    if (reg_write8(REG_DATA_FORMAT,  ADXL345_FMT_VAL)     != ESP_OK) goto fail;
    if (reg_write8(REG_FIFO_CTL,     ADXL345_FIFO_VAL)    != ESP_OK) goto fail;

    /* 기록이 실제로 먹었는지 되읽어 확인. I2C가 ACK만 하고 내부적으로
       무시하는 상황(전원 마진 부족 등)을 부팅 시점에 잡는다. */
    uint8_t rb_bw = 0, rb_fmt = 0, rb_fifo = 0;
    if (reg_read(REG_BW_RATE, &rb_bw, 1) != ESP_OK ||
        reg_read(REG_DATA_FORMAT, &rb_fmt, 1) != ESP_OK ||
        reg_read(REG_FIFO_CTL, &rb_fifo, 1) != ESP_OK) {
        ESP_LOGE(TAG, "설정 readback 실패");
        goto fail;
    }
    if (rb_bw != ADXL345_BW_RATE_VAL || rb_fmt != ADXL345_FMT_VAL ||
        rb_fifo != ADXL345_FIFO_VAL) {
        ESP_LOGE(TAG, "설정 readback 불일치: BW=0x%02X/0x%02X FMT=0x%02X/0x%02X"
                      " FIFO=0x%02X/0x%02X",
                 rb_bw, ADXL345_BW_RATE_VAL, rb_fmt, ADXL345_FMT_VAL,
                 rb_fifo, ADXL345_FIFO_VAL);
        goto fail;
    }

    if (reg_write8(REG_POWER_CTL, PWR_MEASURE) != ESP_OK) goto fail;

    s_present = true;
    ESP_LOGI(TAG, "0x%02X attached: 100Hz low-power, FULL_RES 16g,"
                  " FIFO stream 32 (창 320ms)", ADXL345_I2C_ADDR);
    return true;

fail:
    i2c_master_bus_rm_device(s_dev);
    s_dev = NULL;
    return false;
}

bool adxl345_read_rms(uint16_t *rms_x, uint16_t *rms_y, uint16_t *rms_z,
                      uint32_t *out_n)
{
    if (rms_x) *rms_x = 0;
    if (rms_y) *rms_y = 0;
    if (rms_z) *rms_z = 0;
    if (out_n) *out_n = 0;

    if (s_dev == NULL) return false;

    /* 백오프 중이면 버스를 건드리지 않는다 - 부품이 빠졌을 때 매 사이클
       타임아웃을 무는 것을 막는다(as6221.c와 같은 정책). */
    if (!s_present && esp_timer_get_time() < s_retry_at_us) return false;

    uint8_t status = 0;
    esp_err_t err = reg_read(REG_FIFO_STATUS, &status, 1);
    if (err != ESP_OK) goto io_fail;

    uint32_t entries = status & FIFO_ENTRIES_MASK;
    if (entries > ADXL345_FIFO_DEPTH) entries = ADXL345_FIFO_DEPTH;

    uint32_t sum_sq[3] = { 0, 0, 0 };
    int32_t  sum[3]    = { 0, 0, 0 };
    uint32_t n         = 0;

    for (uint32_t i = 0; i < entries; i++) {
        uint8_t d[6];
        /* DATAX0~DATAZ1 6바이트를 한 번에. 이 읽기가 FIFO 엔트리를 1개 pop한다.
           데이터시트는 연속 FIFO 읽기 사이에 5us 이상을 요구하는데, 100kHz에서
           한 트랜잭션이 ~700us라 자연히 충족된다 - 별도 지연이 필요 없다. */
        if (reg_read(REG_DATAX0, d, sizeof(d)) != ESP_OK) {
            err = ESP_FAIL;
            break;
        }
        /* 리틀엔디안 16-bit 2의 보수. FULL_RES 16g에서 유효 13비트지만
           부호 확장된 int16으로 그대로 쓰면 된다. */
        int16_t v[3] = {
            (int16_t)(((uint16_t)d[1] << 8) | d[0]),
            (int16_t)(((uint16_t)d[3] << 8) | d[2]),
            (int16_t)(((uint16_t)d[5] << 8) | d[4]),
        };
        for (int a = 0; a < 3; a++) {
            sum[a]    += v[a];
            sum_sq[a] += (uint32_t)((int32_t)v[a] * (int32_t)v[a]);
        }
        n++;
    }

    if (n == 0) {
        /* 트랜잭션은 성공했는데 FIFO가 비어 있다 = measure 모드가 풀렸거나
           ODR이 0이다. IO 실패와 같은 취급으로 백오프시킨다. */
        if (err == ESP_OK) err = ESP_ERR_INVALID_STATE;
        goto io_fail;
    }

    if (!s_present) {
        ESP_LOGI(TAG, "복구 (n=%u)", (unsigned)n);
    }
    s_present = true;
    s_fails   = 0;

    if (rms_x) *rms_x = accel_rms_to_mg(sum_sq[0], sum[0], n, ADXL345_SENS_FULL_RES);
    if (rms_y) *rms_y = accel_rms_to_mg(sum_sq[1], sum[1], n, ADXL345_SENS_FULL_RES);
    if (rms_z) *rms_z = accel_rms_to_mg(sum_sq[2], sum[2], n, ADXL345_SENS_FULL_RES);
    if (out_n) *out_n = n;
    return true;

io_fail:
    if (s_fails < ADXL345_FAIL_LIMIT) s_fails++;
    if (s_fails >= ADXL345_FAIL_LIMIT && s_present) {
        s_present     = false;
        s_retry_at_us = esp_timer_get_time() + ADXL345_RETRY_AFTER_US;
        ESP_LOGW(TAG, "%d회 연속 실패(%s) -> %llds 후 재시도. RMS는 0으로 광고된다",
                 ADXL345_FAIL_LIMIT, esp_err_to_name(err),
                 (long long)(ADXL345_RETRY_AFTER_US / 1000000));
    } else if (!s_present) {
        s_retry_at_us = esp_timer_get_time() + ADXL345_RETRY_AFTER_US;
    }
    return false;
}

bool adxl345_present(void)
{
    return s_present;
}

bool adxl345_test_force_standby(void)
{
    if (s_dev == NULL) return false;
    esp_err_t err = reg_write8(REG_POWER_CTL, PWR_STANDBY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "force standby 실패: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGW(TAG, "standby 강제 진입 - 이후 FIFO는 비어 있다(진단 전용)");
    return true;
}
