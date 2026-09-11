/* Execute the actual ULP C collector with deterministic sensor/clock stubs.
   Hardware GPIO timing, fence instructions, and sleep are not simulated. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define INGPS_LOGGER_HOST_TEST 1
#define main ulp_entry
#include "../../main/logger/ulp/main.c"
#undef main

static uint32_t fake_tick, base_tick, sample_number, read_time, wakes;
static int test_fault, full_fifo;
static log_record_t received[LOG_CAPACITY];
static unsigned received_count;
uint32_t test_ticks(void) { return fake_tick; }
void ulp_riscv_wakeup_main_processor(void) { ++wakes; }
void soft_i2c_init(void) {}
bool soft_i2c_recover(void) { return !test_fault; }
bool soft_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, unsigned len)
{ (void)addr; (void)reg; (void)data; (void)len; return !test_fault; }
bool soft_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, unsigned len)
{
    if (test_fault) return false;
    memset(data, 0, len);
    if (addr == 0x53) {
        if (reg == 0) data[0] = 0xe5;
        else if (reg == 0x2c) data[0] = 0x1a;
        else if (reg == 0x31) data[0] = 0x0b;
        else if (reg == 0x38) data[0] = 0x9f;
        else if (reg == 0x39) {
            uint32_t dt = fake_tick - read_time;
            data[0] = full_fifo ? 32 : (dt / 10 > 32 ? 32 : dt / 10);
            read_time = fake_tick;
        } else if (reg == 0x32) {
            int16_t values[3] = {(sample_number & 1) ? 256 : -256,
                                (sample_number & 1) ? 128 : -128, 256};
            memcpy(data, values, 6); ++sample_number;
        }
    } else if (reg == 1) { data[0] = 0x40; data[1] = 0x60; }
    else {
        int16_t raw = addr == 0x48 ? -64 : 3200;
        data[0] = (uint16_t)raw >> 8; data[1] = raw;
    }
    return true;
}
static void consume(void)
{
    for (unsigned j = 0; j < 2; ++j) {
        if (!shared.ready[j]) continue;
        assert(shared.count[j] <= LOG_BANK_RECORDS);
        for (unsigned i = 0; i < shared.count[j]; ++i) {
            log_record_t r = shared.records[j][i];
            assert(r.crc16 == log_crc16(&r, 18));
            assert(r.index < LOG_CAPACITY && r.end_s == r.index + 1);
            received[received_count++] = r;
        }
        shared.ready[j] = 0;
    }
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *mode = argv[1];
    base_tick = !strcmp(mode, "wrap") ? UINT32_MAX - 500 : 0;
    fake_tick = read_time = base_tick;
    shared.magic = LOG_MAGIC; shared.ticks_per_s = 1000;
    shared.th_addr[0] = 0x48; shared.th_addr[1] = 0x49;
    for (uint32_t ms = 0; ms <= 1800000; ms += 200) {
        if (!strcmp(mode, "gap") && ms > 1000 && ms < 5000) continue;
        fake_tick = base_tick + ms;
        test_fault = !strcmp(mode, "fault") && ms >= 5000 && ms < 10000;
        full_fifo = !strcmp(mode, "full") && ms >= 5000 && ms < 6000;
        assert(ulp_entry() == 0);
        if (!strcmp(mode, "backpressure") && ms < 65000) continue;
        consume();
    }
    assert(shared.done && shared.produced == 1800);
    if (!strcmp(mode, "backpressure")) {
        assert(shared.dropped >= 5 && received_count + shared.dropped == 1800);
        assert(received[0].index == 0 && received[29].index == 29);
    } else {
        assert(received_count == 1800);
        for (unsigned i = 0; i < received_count; ++i) assert(received[i].index == i);
    }
    if (!strcmp(mode, "baseline") || !strcmp(mode, "wrap")) {
        for (unsigned i = 0; i < received_count; ++i) {
            assert(received[i].samples == 100);
            assert(received[i].th1_x100 == -50 && received[i].th2_x100 == 2500);
            assert(received[i].flags == 7);
            assert(received[i].rms_mg[0] == 1000 && received[i].rms_mg[1] == 500 && received[i].rms_mg[2] == 0);
        }
    } else if (!strcmp(mode, "gap")) {
        assert(received[1].flags & LOG_MISSING);
        assert(!(received[4].flags & LOG_RMS_VALID));
    } else if (!strcmp(mode, "fault")) {
        assert(received[6].flags & LOG_ACCEL_ERROR);
        assert(!(received[6].flags & LOG_RMS_VALID));
        assert(received[100].flags == 7);
    } else if (!strcmp(mode, "full")) {
        assert(received[5].flags & LOG_FIFO_FULL);
        assert(!(received[5].flags & LOG_RMS_VALID));
    }
    printf("PASS %s: produced=%u received=%u dropped=%u wakes=%u\n", mode,
        shared.produced, received_count, shared.dropped, wakes);
    return 0;
}
