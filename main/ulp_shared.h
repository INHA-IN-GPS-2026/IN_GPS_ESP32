// ulp_shared.h
#pragma once
#include <stdint.h>

#define ULP_MAGIC   0x56494221u  // "!BIV"
#define ULP_VERSION 2

/*
 * ADXL335 vibration accumulator (ULP RISC-V).
 *
 *  - ULP samples ADC X/Y/Z every 5 ms (200 Hz)
 *  - Accumulates (raw - zero)^2 per axis
 *  - Main CPU reads sum_sq / sample_count once per second,
 *    derives RMS in milli-g, then resets the accumulators.
 *
 * Race note: the main CPU may read & zero between two ULP
 * cycles, costing at most 1 sample/second. Acceptable at 200 Hz.
 */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t cal_phase;       /* 1: ULP accumulates raw into sum_raw_*, no sum_sq */
                              /* 0: normal — sum_sq accumulation */

    /* main -> ULP: DC zero offset (set dynamically after cal_phase ends) */
    int16_t  zero_x;
    int16_t  zero_y;
    int16_t  zero_z;
    int16_t  reserved1;

    /* ULP -> main: per-axis sum of squared deltas, reset every 1 s */
    uint32_t sum_sq_x;
    uint32_t sum_sq_y;
    uint32_t sum_sq_z;
    uint32_t sample_count;

    /* Calibration accumulators (used only during cal_phase=1) */
    uint32_t sum_raw_x;
    uint32_t sum_raw_y;
    uint32_t sum_raw_z;

    /* Last raw samples (diagnostics + NTC temperature source) */
    int16_t  last_raw_x;
    int16_t  last_raw_y;
    int16_t  last_raw_z;
    int16_t  reserved2;

    /* NTC thermistor raw values (slow-changing; only last sample used) */
    int16_t  last_raw_ntc1;   /* GPIO4 / ADC1_CH3 */
    int16_t  last_raw_ntc2;   /* GPIO5 / ADC1_CH4 */
    int16_t  reserved3;
    int16_t  reserved4;

    /* Lifetime sample counter (diagnostics) */
    uint32_t total_samples;
} ulp_shared_t;

extern volatile ulp_shared_t ulp_shared;
