#include "sensor/i2c_bus.h"

#include "esp_log.h"

#include "board_pins.h"

static const char *TAG = "I2C_BUS";

static i2c_master_bus_handle_t s_bus;

bool ingps_i2c_bus_init(void)
{
    if (s_bus != NULL) {
        return true;                  /* 멱등 */
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = PIN_I2C_SDA,
        .scl_io_num        = PIN_I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        /* rev 4.0은 외부 풀업(R11/R12 10kΩ)이 있으므로 내부 풀업을 켜지 않는다.
           병렬로 물리면 유효 풀업이 ~8.2kΩ으로 낮아져 유휴 누설만 늘고,
           상승시간 개선폭은 미미하다. board_pins.h 주석 참조. */
        .flags.enable_internal_pullup = false,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus create failed (SDA=%d SCL=%d): %s",
                 PIN_I2C_SDA, PIN_I2C_SCL, esp_err_to_name(err));
        s_bus = NULL;
        return false;
    }

    /* PM 락 주의 — INGPS_PM_DEBUG 덤프에서 "I2C_0 / NO_LIGHT_SLEEP"을 보고
       놀라지 말 것. ESP32-S3에서 I2C_CLK_SRC_DEFAULT는 XTAL이고, 이 경우
       드라이버가 만드는 락은 ESP_PM_NO_LIGHT_SLEEP이다(i2c_common.c).
       다만 락은 트랜잭션 진입에서 acquire, 종료에서 release 되므로
       (i2c_master.c) 광고 사이클당 수 ms만 잡힌다 — led_strip/RMT처럼 계속
       쥐고 있어 light sleep을 통째로 막는 문제와는 다르다.
       ⚠ 단 rev 4.0에서는 ADXL345 FIFO 드레인(32샘플 × 6B ≈ 20ms @100kHz)이
         가장 긴 트랜잭션이다. 덤프에서 count>0으로 굳어 있다면 그때는 진짜 누수다. */
    ESP_LOGI(TAG, "bus up: SDA=%d SCL=%d @%dHz (외부 풀업 10k)",
             PIN_I2C_SDA, PIN_I2C_SCL, INGPS_I2C_SCL_HZ);
    return true;
}

i2c_master_bus_handle_t ingps_i2c_bus(void)
{
    return s_bus;
}
