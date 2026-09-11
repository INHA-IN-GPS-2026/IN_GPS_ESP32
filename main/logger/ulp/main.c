#include "../log_format.h"
#include "soft_i2c.h"
#include "ulp_riscv_utils.h"
#include "soc/rtc_cntl_reg.h"
#include "ulp_riscv_register_ops.h"

volatile log_shared_t shared;
static uint32_t started, last_run, next_temp, retry[3];
static uint32_t next_end = 1, bank, used, flags, n;
static int32_t sums[3];
static uint64_t squares[3];
static int16_t temperatures[2] = {LOG_INVALID_TEMP, LOG_INVALID_TEMP};
static uint32_t temp_valid, accel_ok;

#if CONFIG_INGPS_LOGGER_DIAGNOSTICS
/* Independent debug symbols: not part of the sensor wire format.
   Snapshots are observational, not an atomic cross-core transaction. */
volatile uint32_t debug_entries, debug_stage, debug_tick, debug_elapsed;
volatile uint32_t debug_accel_ok, debug_temp_valid, debug_flags;
#define TRACE(stage) (debug_stage = (stage))
#else
#define TRACE(stage) ((void)0)
#endif

static void publish(void)
{
#ifndef INGPS_LOGGER_HOST_TEST
    /* S3 ULP traps on the FENCE emitted here (stage 8, init=0).
       RTC shared accesses are volatile, as in IDF's ULP lock implementation.
       Keep compiler ordering without emitting a hardware FENCE instruction.
       HP-side Xtensa memw and ready-bank ownership remain in place. */
    __asm__ volatile("" ::: "memory");
#endif
}
static uint32_t ticks(void)
{
    /* Same latch sequence as IDF esp32s3 rtc_cntl_ll_get_rtc_time().
       HP does not access RTC time registers during an acquisition. */
    SET_PERI_REG_MASK(RTC_CNTL_TIME_UPDATE_REG, RTC_CNTL_TIME_UPDATE);
    return READ_PERI_REG(RTC_CNTL_TIME0_REG);
}
static bool write8(uint8_t r, uint8_t v) { return soft_i2c_write(0x53, r, &v, 1); }
static bool setup_accel(void)
{
    uint8_t id, bw, fmt, fifo;
    return soft_i2c_read(0x53, 0, &id, 1) && id == 0xe5 &&
        write8(0x2d, 0) && write8(0x2e, 0) && write8(0x2c, 0x1a) &&
        write8(0x31, 0x0b) && write8(0x38, 0x9f) &&
        soft_i2c_read(0x53, 0x2c, &bw, 1) && bw == 0x1a &&
        soft_i2c_read(0x53, 0x31, &fmt, 1) && fmt == 0x0b &&
        soft_i2c_read(0x53, 0x38, &fifo, 1) && fifo == 0x9f && write8(0x2d, 8);
}
static bool setup_temp(unsigned ch)
{
    uint8_t config[2] = {0x40, 0x60}, readback[2];
    return soft_i2c_write(shared.th_addr[ch], 1, config, 2) &&
        soft_i2c_read(shared.th_addr[ch], 1, readback, 2) &&
        (readback[0] & 0xbf) == 0 && (readback[1] & 0xdf) == 0x40;
}
static uint32_t isqrt64(uint64_t a)
{
    uint64_t result = 0, bit = (uint64_t)1 << 62;
    while (bit > a) bit >>= 2;
    while (bit) {
        if (a >= result + bit) { a -= result + bit; result = (result >> 1) + bit; }
        else result >>= 1;
        bit >>= 2;
    }
    return (uint32_t)result;
}
static void append(bool missing)
{
    log_record_t r = {.index = next_end - 1, .end_s = next_end,
        .th1_x100 = temperatures[0], .th2_x100 = temperatures[1],
        .samples = n, .flags = flags | temp_valid};
    if (missing) {
        r.samples = 0; r.flags = LOG_MISSING | LOG_TIMING_GAP;
        r.th1_x100 = r.th2_x100 = LOG_INVALID_TEMP;
    } else if (n && !(flags & (LOG_FIFO_FULL | LOG_TIMING_GAP | LOG_ACCEL_ERROR))) {
        r.flags |= LOG_RMS_VALID;
        for (unsigned a = 0; a < 3; ++a) {
            /* ADXL345 full-resolution 256 counts/g, same scale as legacy.
               Keep signed sum squared and fractional precision until sqrt. */
            int64_t s = sums[a];
            uint64_t variance_n2 = squares[a] * n - (uint64_t)(s * s);
            uint64_t scaled = variance_n2 * 1000000ULL / ((uint64_t)n * n * 65536);
            uint32_t rms = isqrt64(scaled);
            r.rms_mg[a] = rms > UINT16_MAX ? UINT16_MAX : rms;
        }
    }
    r.crc16 = log_crc16(&r, 18);
    if (!shared.ready[bank]) {
        shared.records[bank][used++] = r;
        if (used == LOG_BANK_RECORDS || next_end == LOG_DURATION_S) {
            shared.count[bank] = used;
            publish(); shared.ready[bank] = 1;
            bank ^= 1; used = 0;
            ulp_riscv_wakeup_main_processor();
        }
    } else {
        shared.dropped++;
    }
    shared.produced = next_end++;
}
static void clear_window(void)
{
    n = 0; flags = 0;
    for (unsigned a = 0; a < 3; ++a) { sums[a] = 0; squares[a] = 0; }
}
int main(void)
{
#if CONFIG_INGPS_LOGGER_DIAGNOSTICS
    ++debug_entries;
#endif
    TRACE(1);
    if (shared.stop || shared.done || shared.magic != LOG_MAGIC || !shared.ticks_per_s)
        return 0;
    TRACE(2);
    soft_i2c_init();
    if (!shared.initialized) {
        TRACE(3);
        soft_i2c_recover();
        TRACE(4);
        accel_ok = setup_accel();
        for (unsigned ch = 0; ch < 2; ++ch) {
            TRACE(5 + ch);
            if (!setup_temp(ch)) retry[ch] = 60;
        }
        if (!accel_ok) retry[2] = 60;
        TRACE(7);
        started = last_run = ticks();
        TRACE(8);
        publish(); shared.initialized = 1;
    }
    TRACE(9);
    uint32_t now = ticks(), elapsed = (now - started) / shared.ticks_per_s;
#if CONFIG_INGPS_LOGGER_DIAGNOSTICS
    debug_tick = now; debug_elapsed = elapsed;
#endif
    if (now - last_run > shared.ticks_per_s / 4) flags |= LOG_TIMING_GAP;
    last_run = now;

    TRACE(10);
    if (!accel_ok && elapsed >= retry[2]) {
        soft_i2c_recover(); accel_ok = setup_accel(); retry[2] = elapsed + 60;
    }
    uint8_t status = 0, raw[6];
    if (accel_ok && soft_i2c_read(0x53, 0x39, &status, 1)) {
        unsigned entries = status & 0x3f;
        if (entries >= 32) flags |= LOG_FIFO_FULL;
        if (entries > 32) entries = 32;
        for (unsigned i = 0; i < entries; ++i) {
            if (!soft_i2c_read(0x53, 0x32, raw, 6)) {
                flags |= LOG_ACCEL_ERROR | LOG_I2C_ERROR;
                accel_ok = 0; retry[2] = elapsed + 60; break;
            }
            if (n >= 200) { flags |= LOG_TIMING_GAP; continue; }
            for (unsigned a = 0; a < 3; ++a) {
                int32_t v = (int16_t)((uint16_t)raw[2*a] | (uint16_t)raw[2*a+1] << 8);
                /* Values outside the documented full-resolution range mark
                   a bad bus/config; cap arithmetic inputs to prevent overflow. */
                if (v < -4096 || v > 4095) { flags |= LOG_ACCEL_ERROR; v = 0; }
                sums[a] += v; squares[a] += (uint32_t)(v * v);
            }
            ++n;
        }
    } else {
        flags |= LOG_ACCEL_ERROR | LOG_I2C_ERROR;
        if (accel_ok) { accel_ok = 0; retry[2] = elapsed + 60; }
    }
    TRACE(11);
    if (elapsed >= next_temp) {
        next_temp = elapsed + 1; temp_valid = 0;
        for (unsigned ch = 0; ch < 2; ++ch) {
            temperatures[ch] = LOG_INVALID_TEMP;
            if (retry[ch] && elapsed < retry[ch]) { flags |= LOG_TEMP_ERROR; continue; }
            if (retry[ch] && !setup_temp(ch)) {
                retry[ch] = elapsed + 60; flags |= LOG_TEMP_ERROR | LOG_I2C_ERROR; continue;
            }
            if (soft_i2c_read(shared.th_addr[ch], 0, raw, 2)) {
                int32_t v = (int16_t)((uint16_t)raw[0] << 8 | raw[1]);
                temperatures[ch] = (v * 25 + (v >= 0 ? 16 : -16)) / 32;
                temp_valid |= 1u << ch; retry[ch] = 0;
            } else {
                flags |= LOG_TEMP_ERROR | LOG_I2C_ERROR; retry[ch] = elapsed + 60;
            }
        }
    }
    /* Drain first so the final FIFO is included too. Windows are quantized to
       drain boundaries (not exact sample timestamps); retain the actual count. */
    TRACE(12);
    elapsed = (ticks() - started) / shared.ticks_per_s;
#if CONFIG_INGPS_LOGGER_DIAGNOSTICS
    debug_elapsed = elapsed; debug_accel_ok = accel_ok;
    debug_temp_valid = temp_valid; debug_flags = flags;
#endif
    while (next_end <= elapsed && next_end <= LOG_DURATION_S) {
        bool missing = next_end < elapsed;
        append(missing);
        if (!missing) clear_window();
    }
    if (next_end > LOG_DURATION_S) {
        TRACE(13);
        /* Standby is best effort; stored sensor flags describe acquisition. */
        (void)write8(0x2d, 0);
        publish(); shared.done = 1;
        ulp_riscv_wakeup_main_processor();
    }
    TRACE(14);
    publish(); shared.progress++;
    TRACE(15);
    return 0; /* halt; timer period starts after work finishes */
}
