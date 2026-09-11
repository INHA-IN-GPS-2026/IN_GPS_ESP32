#include "soft_i2c.h"
#include "../../board_pins.h"
#include "ulp_riscv_gpio.h"
#include "ulp_riscv_utils.h"

/* External pull-ups. Output latch always LOW; enable sinks, disable releases. */
static void delay(void) { ulp_riscv_delay_cycles(90); }
static void low(unsigned pin) { ulp_riscv_gpio_output_enable(pin); }
static void release(unsigned pin) { ulp_riscv_gpio_output_disable(pin); }
static bool high(unsigned pin) { return ulp_riscv_gpio_get_level(pin); }
static bool clock_high(void)
{
    release(PIN_I2C_SCL);
    for (unsigned i = 0; i < 100; ++i) {
        delay();
        if (high(PIN_I2C_SCL)) return true;
    }
    return false; /* bounded clock stretching, about 0.5 ms plus overhead */
}
static bool stop(void)
{
    low(PIN_I2C_SDA); delay();
    bool ok = clock_high();
    release(PIN_I2C_SDA); delay();
    release(PIN_I2C_SCL);
    return ok && high(PIN_I2C_SDA);
}
static bool start(void)
{
    release(PIN_I2C_SDA);
    if (!clock_high() || !high(PIN_I2C_SDA)) return false;
    low(PIN_I2C_SDA); delay(); low(PIN_I2C_SCL); delay();
    return true;
}
static bool put(uint8_t v)
{
    for (unsigned i = 0; i < 8; ++i, v <<= 1) {
        if (v & 0x80) release(PIN_I2C_SDA); else low(PIN_I2C_SDA);
        delay();
        if (!clock_high()) return false;
        low(PIN_I2C_SCL); delay();
    }
    release(PIN_I2C_SDA); delay();
    if (!clock_high()) return false;
    bool ack = !high(PIN_I2C_SDA);
    low(PIN_I2C_SCL); delay();
    return ack;
}
static bool get(uint8_t *v, bool last)
{
    *v = 0; release(PIN_I2C_SDA);
    for (unsigned i = 0; i < 8; ++i) {
        delay();
        if (!clock_high()) return false;
        *v = (*v << 1) | high(PIN_I2C_SDA);
        low(PIN_I2C_SCL); delay();
    }
    if (!last) low(PIN_I2C_SDA); /* final byte NACK */
    delay();
    if (!clock_high()) return false;
    low(PIN_I2C_SCL); delay(); release(PIN_I2C_SDA);
    return true;
}
void soft_i2c_init(void)
{
    for (unsigned p = PIN_I2C_SDA; p <= PIN_I2C_SCL; ++p) {
        ulp_riscv_gpio_init(p);
        release(p);
        ulp_riscv_gpio_output_level(p, 0);
        ulp_riscv_gpio_set_output_mode(p, RTCIO_MODE_OUTPUT_OD);
        ulp_riscv_gpio_input_enable(p);
        ulp_riscv_gpio_pullup_disable(p);
        ulp_riscv_gpio_pulldown_disable(p);
    }
}
bool soft_i2c_recover(void)
{
    release(PIN_I2C_SDA);
    for (unsigned i = 0; i < 9 && !high(PIN_I2C_SDA); ++i) {
        low(PIN_I2C_SCL); delay();
        if (!clock_high()) { release(PIN_I2C_SDA); return false; }
    }
    return stop();
}
bool soft_i2c_read(uint8_t a, uint8_t r, uint8_t *p, unsigned n)
{
    bool ok = start() && put(a << 1) && put(r) && start() && put((a << 1) | 1);
    for (unsigned i = 0; ok && i < n; ++i) ok = get(p + i, i + 1 == n);
    bool stopped = stop();
    return ok && stopped;
}
bool soft_i2c_write(uint8_t a, uint8_t r, const uint8_t *p, unsigned n)
{
    bool ok = start() && put(a << 1) && put(r);
    for (unsigned i = 0; ok && i < n; ++i) ok = put(p[i]);
    bool stopped = stop();
    return ok && stopped;
}
