// ulp_shared.h — ULP(RTC 도메인)와 메인 CPU가 공유하는 유일한 상태.
#pragma once
#include <stdint.h>

#define ULP_MAGIC   0x56494221u  // "!BIV"
/* v5: NTC ADC 채널(CH2/CH3) 제거. 온도는 AS6221(I2C)이 메인 CPU에서 읽는다.
   ULP는 ADXL335 3축만 담당한다. */
#define ULP_VERSION 5

/* ★전류 A/B 실험 전용 — 1이면 ULP/SARADC 서브시스템을 통째로 빼고 부팅한다.
   빠지는 것: ULP 기동, 194.7Hz SARADC 폴링, ADXL zero 캘리브(1s),
             RTC_PERIPH 도메인 강제 ON, 워치독의 ULP stall 감시.
   유지되는 것: BLE 광고·light sleep·워치독 나머지 계층 → 전력 경로 중
             "ULP 기여분"만 분리 측정할 수 있다.
   ⚠ ADXL335(~350µA)는 전원 게이팅 회로가 없어 이 스위치로 안 빠진다.
     즉 측정되는 델타는 ULP+SARADC+RTC_PERIPH 몫이다.
     (아날로그 버전의 NTC 분압 ~330µA는 AS6221 I2C 전환으로 사라졌다.)
   ⚠ 켜면 RMS가 전부 0으로 광고된다. 온도는 I2C 경로라 영향받지 않는다.
     절대 운영 빌드로 내보내지 말 것. */
#ifndef INGPS_ULP_ADC_OFF
#define INGPS_ULP_ADC_OFF 0
#endif

/* ULP가 매 사이클 각 ADXL 채널을 N회 읽어 평균낸다. 비상관(화이트) SAR 노이즈를
   √N 배 줄인다. 60Hz mains 같은 코히런트 피크는 한 사이클 내 평균으로 안 줄어들며
   접지/전원 분리로 따로 잡아야 한다.
   주의: 한 사이클(≈5ms) 안에 5채널의 모든 read가 끝나야 한다. N을 키우면 사이클이
   넘쳐 실효 샘플레이트가 떨어지므로, 의심되면 BLE_ADV DEBUG 로그의 fs= 값이
   기대치를 유지하는지 확인하고 미달 시 SHIFT를 낮출 것.
   ULP RISC-V엔 하드웨어 나눗셈이 없어 평균은 시프트로 처리 → N은 2의 거듭제곱. */
#ifndef ADXL_OVERSAMPLE_SHIFT
#define ADXL_OVERSAMPLE_SHIFT 3   /* N=8 → √8≈2.8× SAR 잡음↓ */
#endif
#define ADXL_OVERSAMPLE_N (1u << ADXL_OVERSAMPLE_SHIFT)

/* 레이스: 메인이 누적값을 읽고 0을 쓰는 사이에 ULP가 한 번 더 누적할 수 있다.
   200Hz 기준 최대 1샘플 손실 → 무시 가능. */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved0;

    /* main → ULP: DC 영점 (ulp_init에서 1회 기록) */
    int16_t  zero_x;
    int16_t  zero_y;
    int16_t  zero_z;
    int16_t  reserved1;

    /* ULP → main: 누적값. main이 사이클마다 읽고 0으로 리셋 */
    uint32_t sum_sq_x;
    uint32_t sum_sq_y;
    uint32_t sum_sq_z;
    uint32_t sample_count;

    /* ULP → main: dx(=raw-zero)의 단순 합. main이 분산 = sum_sq/n - (sum_dx/n)^2 로
       자세 독립 RMS를 계산한다 — 기기를 다른 면/기울기로 옮겨도 중력 DC가 RMS에
       새어들지 않는다. |dx|<=~2048, 윈도우 200샘플 → |sum_dx|<=~4e5 (int32 안전). */
    int32_t  sum_dx_x;
    int32_t  sum_dx_y;
    int32_t  sum_dx_z;

    /* 진단용 마지막 raw 값 */
    int16_t  last_raw_x;
    int16_t  last_raw_y;
    int16_t  last_raw_z;
    int16_t  reserved2;

    /* 동적 zero 보정용 raw 누적 (int16으로는 넘쳐 uint32 필요) */
    uint32_t sum_raw_x;
    uint32_t sum_raw_y;
    uint32_t sum_raw_z;

    /* 1 = 부팅 직후 zero-cal 단계, 0 = 정상 sum_sq 누적 */
    uint8_t  cal_phase;
    uint8_t  reserved3;
    uint16_t reserved4;

    /* 전체 누적 샘플 수 — wdt_guard가 ULP 생존 확인에 쓴다(전진하지 않으면 재부팅) */
    uint32_t total_samples;
} ulp_shared_t;

extern volatile ulp_shared_t ulp_shared;
