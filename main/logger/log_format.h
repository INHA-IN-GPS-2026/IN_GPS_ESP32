#pragma once
#include <stdint.h>
#include <stddef.h>
#ifndef INGPS_LOGGER_HOST_TEST
#include "sdkconfig.h"
#endif

#define LOG_MAGIC 0x314c4749u /* IGL1 */
#define LOG_VERSION 1
#define LOG_INTERVAL_S 1u
#define LOG_DURATION_S (CONFIG_INGPS_LOGGER_DURATION_S * 1u)
#define LOG_CAPACITY (LOG_DURATION_S / LOG_INTERVAL_S)
_Static_assert(LOG_DURATION_S >= 300 && LOG_DURATION_S <= 1800, "supported batch duration");
#define LOG_BANK_RECORDS 30u
#define LOG_INVALID_TEMP INT16_MIN
enum {
    LOG_TH1_VALID = 1, LOG_TH2_VALID = 2, LOG_RMS_VALID = 4,
    LOG_I2C_ERROR = 8, LOG_FIFO_FULL = 16, LOG_TIMING_GAP = 32,
    LOG_TEMP_ERROR = 64, LOG_ACCEL_ERROR = 128, LOG_MISSING = 256
};
typedef struct {
    uint16_t index, end_s;
    int16_t th1_x100, th2_x100;
    uint16_t rms_mg[3], samples, flags, crc16;
} log_record_t;
_Static_assert(sizeof(log_record_t) == 20, "record must fit ATT MTU 23");
_Static_assert(offsetof(log_record_t, crc16) == 18, "wire layout");

typedef struct {
    uint32_t magic, ticks_per_s, th_addr[2]; /* HP initializes before starting ULP */
    uint32_t initialized, progress, done, produced, dropped, stop;
    uint32_t ready[2], count[2]; /* ULP publishes ready last; HP releases after flash */
    log_record_t records[2][LOG_BANK_RECORDS];
} log_shared_t;

/* CRC-16/CCITT-FALSE over first 18 bytes, init FFFF, poly 1021. */
static inline uint16_t log_crc16(const void *data, unsigned size)
{
    const uint8_t *p = data;
    uint16_t c = 0xffff;
    while (size--) {
        c ^= (uint16_t)*p++ << 8;
        for (unsigned b = 0; b < 8; ++b)
            c = (c & 0x8000) ? (c << 1) ^ 0x1021 : c << 1;
    }
    return c;
}
