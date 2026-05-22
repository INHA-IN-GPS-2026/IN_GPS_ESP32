// ulp_shared.h
#pragma once
#include <stdint.h>

#define ULP_MAGIC   0x56494221u  // "!BIV"
#define ULP_VERSION 2

// /*
//   ADXL335 진동 누적기 (ULP RISC-V)
 
//    - ULP가 5ms 주기(200Hz)로 ADC X/Y/Z 읽음
//    - (raw - zero)^2 누적
//   - 메인 CPU가 1초마다 sum_sq*/sample_count로 RMS 계산 후 0으로 리셋
 
//    레이스 노트:
//        메인 CPU가 sum/count 읽고 0 쓰는 사이에 ULP가 한 번 더
//  *      누적할 수 있음. 200Hz 기준 최대 1샘플 손실 → 무시 가능.
//  */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved0;

    /* main → ULP: DC 영점 (ulp_init에서 1회 기록) */
    int16_t  zero_x;
    int16_t  zero_y;
    int16_t  zero_z;
    int16_t  reserved1;

    /* ULP → main: 누적값. main이 1초마다 읽고 0으로 리셋 */
    uint32_t sum_sq_x;
    uint32_t sum_sq_y;
    uint32_t sum_sq_z;
    uint32_t sample_count;

    /* 진단용 마지막 raw 값 */
    int16_t  last_raw_x;
    int16_t  last_raw_y;
    int16_t  last_raw_z;
    int16_t  reserved2;

    /* 최신 NTC raw (BLE 광고 시 온도 변환에 사용) */
    int16_t  last_raw_ntc1;
    int16_t  last_raw_ntc2;

    /* 동적 zero 보정용 raw 누적 (3s × 200Hz × ~2000 → int16 overflow, uint32 필요) */
    uint32_t sum_raw_x;
    uint32_t sum_raw_y;
    uint32_t sum_raw_z;

    /* 1 = 부팅 직후 zero-cal 단계, 0 = 정상 sum_sq 누적 */
    uint8_t  cal_phase;
    uint8_t  reserved3;
    uint16_t reserved4;

    /* 전체 누적 샘플 수 (디버그) */
    uint32_t total_samples;
} ulp_shared_t;

extern volatile ulp_shared_t ulp_shared;
