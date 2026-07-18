// ulp_shared.h
#pragma once
#include <stdint.h>

#define ULP_MAGIC   0x56494221u  // "!BIV"
#define ULP_VERSION 2

/* === ADXL raw 캡처(진단용) ===========================================
   1로 두면 ULP가 매 사이클 raw(X/Y/Z)를 링버퍼에 적재하고, 메인 CPU가
   부팅 직후 N초간 UART로 CSV(x,y,z @200Hz)를 흘린다. FFT 노이즈 분석용.
   운영 펌웨어로 복귀할 땐 0으로 내려 빌드 → 링버퍼/캡처 코드 모두 제외. */
#ifndef ADXL_RAW_CAPTURE
#define ADXL_RAW_CAPTURE 1
#endif

/* 링버퍼 길이(2의 거듭제곱). 256 → ~1.28s 버퍼(200Hz). 256*3*2B = 1536B. */
#define RAW_RING_LEN  256
#define RAW_RING_MASK (RAW_RING_LEN - 1)

/* === ADXL ADC 오버샘플링 =============================================
   ULP가 매 사이클 각 ADXL 채널을 N회 읽어 평균낸다. 비상관(화이트) SAR ADC
   노이즈를 √N 배 줄인다. N=8 → 약 2.8배 감소(측정 노이즈밀도 ~5x → ~1.8x
   데이터시트). 빠른 연속 read라 30Hz 진동(주기 33ms)엔 영향 없음.
   주의 1) 5ms(200Hz) 한 사이클 안에 모든 read가 끝나야 한다. 채널이 5개라
           N을 너무 키우면 사이클이 5ms를 넘겨 실효 샘플레이트가 떨어진다.
           오버런 의심되면 N을 줄이거나 app_main의 measured_fs 로그 확인.
        2) 60Hz mains 같은 코히런트 피크는 한 사이클 내 평균으론 안 줄어든다.
           그건 접지/전원 분리로 따로 잡아야 함.
   ULP RISC-V엔 하드웨어 나눗셈이 없으므로 평균은 시프트로 처리한다 →
   N은 반드시 2의 거듭제곱(SHIFT로 지정). SHIFT=3 → N=8, SHIFT=4 → N=16. */
#ifndef ADXL_OVERSAMPLE_SHIFT
#define ADXL_OVERSAMPLE_SHIFT 0
#endif
#define ADXL_OVERSAMPLE_N (1u << ADXL_OVERSAMPLE_SHIFT)

/* === NTC 채널전환 정착(settle) 리드 수 ================================
   07-08 스코프 실측: CH2 NTC 노드가 ULP ADC 샘플 시점마다(~170Hz) -673mV
   폭락 → SAR S/H가 고임피던스(5~10kΩ) NTC 노드에서 전하를 끌어가는 charge
   injection이 원인으로 진단됨(하드웨어 100nF 캡이 근본 수정이나 미실장).
   그때 결론: "무HW 레버 = ULP ADC 샘플/획득시간 늘리면 딥 감소".
   기존엔 채널 전환 후 dummy read 1회만 버리고 바로 본 샘플을 썼는데(잔류
   전압 크로스토크만 제거, 획득시간 자체는 그대로) → dummy read 횟수를
   늘려 고임피던스 노드가 더 오래 정착할 시간을 준다(소프트웨어만으로
   acquisition time을 사실상 늘리는 효과). 2~3회로 시작해 raw 로그로
   딥이 줄었는지(σ, avg_raw 안정성) 확인 후 조정 권장. 5ms(200Hz) 예산
   안에서 안전한 범위(ADC 1회 read가 수십 μs 수준). */
#ifndef NTC_SETTLE_READS
#define NTC_SETTLE_READS 3
#endif

/* === NTC raw 캡처 + trimmed mean (2026-07-16) ==========================
   vcc40.csv 실측: ~112ms 주기, ~1.3ms 폭짜리 VCC dip 확인(BLE TX 전류
   버스트로 추정). ESP32 ADC 기준전압이 VCC에 안 묶여있어(내부 밴드갭)
   ratiometric 분압으로도 이 dip이 raw에 그대로 새어들어옴 — dip이 뜬
   순간 raw_ntc가 수백 count 튈 수 있음(sensor.c VCC 민감도 계산 참고).
   duty cycle이 낮아(~1.2%) 1초 180여 샘플 중 1~2개만 오염되므로, ULP가
   raw를 sum만 누적하던 방식 대신 링버퍼에 개별 샘플을 남기고 main이
   정렬 후 상하위 trim%씩 잘라낸 trimmed mean을 쓰면 이 이상치를 순수
   소프트웨어로 제거할 수 있다. (VCC가 초 단위 이상 지속적으로 밀리는
   경우는 이 방법으로 못 잡음 — 그건 별도 VCC센스 채널이 필요, 미구현.) */
#ifndef NTC_RAW_CAPTURE
#define NTC_RAW_CAPTURE 1
#endif
#define NTC_RING_LEN  256
#define NTC_RING_MASK (NTC_RING_LEN - 1)

/* 트림 비율(편측). 0.05 = 상하위 5%씩(합 10%) 잘라내고 평균.
   1초 183~184샘플 기준 상하위 각 ~9개 제외 — dip 오염 샘플(최대
   1~2개/초 추정)을 넉넉히 커버하면서도 표본을 과하게 버리진 않는다. */
#ifndef NTC_TRIM_FRAC
#define NTC_TRIM_FRAC 0.05f
#endif

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

    /* 최신 NTC raw (진단/로그용) */
    int16_t  last_raw_ntc1;
    int16_t  last_raw_ntc2;

    /* NTC 오버샘플링 누적: ULP가 200Hz로 raw를 더하고 main이 1초마다
       sum/count로 평균 → 비상관 노이즈 √N 저감. main이 읽고 0 리셋.
       ★07-16: 평균 자체는 아래 ntc_ring 기반 trimmed mean으로 대체됨 —
       이 sum/count는 오버플로 방지용으로 계속 리셋만 되고 실제 평균
       계산엔 더 이상 안 쓰임(호환/폴백용으로 필드는 유지). */
    uint32_t sum_ntc1;
    uint32_t sum_ntc2;
    uint32_t ntc_count;

#if NTC_RAW_CAPTURE
    /* === NTC raw 스트리밍 링버퍼 (단일 생산자=ULP / 단일 소비자=main) ===
       ADXL_RAW_CAPTURE 링버퍼와 동일 패턴. ULP가 ntc_ring_head를 증가시키며
       적재, main이 자체 tail로 드레인 후 trimmed mean 계산. */
    uint32_t ntc_ring_head;
    int16_t  ntc_ring1[NTC_RING_LEN];
    int16_t  ntc_ring2[NTC_RING_LEN];
#endif

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

#if ADXL_RAW_CAPTURE
    /* === raw 스트리밍 링버퍼 (단일 생산자=ULP / 단일 소비자=main) ===
       ULP가 ring_head를 증가시키며 적재, main이 자체 tail로 드레인.
       오버런(미드레인 덮어쓰기)은 진단용이라 허용(200Hz면 main이 충분히 따라옴). */
    uint32_t ring_head;          /* ULP가 증가시키는 쓰기 인덱스(자유 진행) */
    int16_t  ring_x[RAW_RING_LEN];
    int16_t  ring_y[RAW_RING_LEN];
    int16_t  ring_z[RAW_RING_LEN];
#endif
} ulp_shared_t;

extern volatile ulp_shared_t ulp_shared;
