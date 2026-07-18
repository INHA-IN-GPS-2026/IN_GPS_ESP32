#include <stdint.h>

#include "ulp_riscv.h"
#include "ulp_riscv_adc_ulp_core.h"
#include "ulp_shared.h"

/* ULP-side symbol is "shared"; main side sees it as "ulp_shared". */
volatile ulp_shared_t shared;

/* 한 채널을 오버샘플링해 평균 raw를 반환.
   채널 전환 직후 첫 read는 SAR S/H cap 잔류전압이라 dummy로 버리고,
   이후 2^shift 회 평균. 비상관 SAR 노이즈를 √(2^shift) 배 줄인다.
   ULP RISC-V엔 하드웨어 나눗셈이 없으므로 평균은 >>shift 로 처리. */
static uint16_t adc_read_avg(int ch, int shift)
{
    (void)ulp_riscv_adc_read_channel(ADC_UNIT_1, ch);  /* dummy: 채널 전환 정착 */
    uint32_t acc = 0;
    int n = 1 << shift;
    for (int i = 0; i < n; i++) {
        acc += (uint32_t)ulp_riscv_adc_read_channel(ADC_UNIT_1, ch);
    }
    return (uint16_t)(acc >> shift);
}

int main(void)
{
    if (shared.magic != ULP_MAGIC) {
        shared.magic         = ULP_MAGIC;
        shared.version       = ULP_VERSION;
        shared.sum_sq_x      = 0;
        shared.sum_sq_y      = 0;
        shared.sum_sq_z      = 0;
        shared.sample_count  = 0;
        shared.total_samples = 0;
#if ADXL_RAW_CAPTURE
        shared.ring_head     = 0;
#endif
#if NTC_RAW_CAPTURE
        shared.ntc_ring_head = 0;
#endif
    }

    /* 채널 전환 직후엔 SAR 내부 sample-and-hold cap이 이전 채널 잔류전압을 유지하므로
       첫 read는 dummy로 버리고 두 번째 read만 사용. NTC(소스 임피던스 5~10kΩ) 측정의
       채널 간 누설을 막고 ADXL 측정 정밀도도 함께 개선.
       ★07-15: dummy 1회→NTC_SETTLE_READS회로 확장. 07-08 스코프에서 CH2 노드가
       채널전환 크로스토크와 별개로 ULP 자체 샘플 시점마다 -673mV 딥을 보였는데(고
       임피던스 노드 charge injection), 이건 dummy 1회로는 못 잡고 정착 리드를
       늘려야 완화된다는 게 그때 결론(무HW 레버). 마지막 read만 실사용치로 채택. */
    /* HW 핀맵(ESP32-S3-WROOM-1 스키매틱): 아날로그가 GPIO3~7로 한 칸씩 내려감.
       ADC1_CHx = GPIO(x+1) → TH1=GPIO3=CH2, TH2=GPIO4=CH3. */
    for (int i = 0; i < NTC_SETTLE_READS - 1; i++) {
        (void)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_2);
    }
    shared.last_raw_ntc1 = (int16_t)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_2);
    for (int i = 0; i < NTC_SETTLE_READS - 1; i++) {
        (void)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_3);
    }
    shared.last_raw_ntc2 = (int16_t)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_3);

    /* NTC 오버샘플링 누적(덧셈만; 나눗셈은 main). 매 200Hz 사이클마다 1샘플 적재
       → main이 1초마다 sum/count로 평균 → 비상관 노이즈 √N 저감(≈√200).
       ★07-16: main은 이제 이 sum/count 대신 아래 ring 기반 trimmed mean을
       쓰지만, 오버플로 방지용으로 sum/count 누적 자체는 계속 둔다(main이
       주기적으로 읽고 0 리셋). */
    shared.sum_ntc1 += (uint32_t)shared.last_raw_ntc1;
    shared.sum_ntc2 += (uint32_t)shared.last_raw_ntc2;
    shared.ntc_count++;

#if NTC_RAW_CAPTURE
    /* NTC raw 개별 샘플 스트리밍: main이 링버퍼를 정렬 후 상하위 trim%
       잘라낸 trimmed mean으로 VCC dip 등 짧은 이상치를 걸러낼 수 있게 함. */
    {
        uint32_t h = shared.ntc_ring_head & NTC_RING_MASK;
        shared.ntc_ring1[h] = shared.last_raw_ntc1;
        shared.ntc_ring2[h] = shared.last_raw_ntc2;
        shared.ntc_ring_head++;
    }
#endif

    /* ADXL335: CH4=GPIO5 (X_OUT), CH5=GPIO6 (Y_OUT), CH6=GPIO7 (Z_OUT).
       GPIO8(CH7)은 SDA_OUT(I2C)이므로 절대 ADC로 읽지 말 것.
       채널당 2^SHIFT회 오버샘플링 평균으로 SAR 노이즈 √N 감소. */
    int16_t rx = (int16_t)adc_read_avg(ADC_CHANNEL_4, ADXL_OVERSAMPLE_SHIFT);
    int16_t ry = (int16_t)adc_read_avg(ADC_CHANNEL_5, ADXL_OVERSAMPLE_SHIFT);
    int16_t rz = (int16_t)adc_read_avg(ADC_CHANNEL_6, ADXL_OVERSAMPLE_SHIFT);

    shared.last_raw_x = rx;
    shared.last_raw_y = ry;
    shared.last_raw_z = rz;

#if ADXL_RAW_CAPTURE
    /* raw 스트리밍: 매 사이클 링버퍼에 적재(자유 진행 head). main이 드레인. */
    {
        uint32_t h = shared.ring_head & RAW_RING_MASK;
        shared.ring_x[h] = rx;
        shared.ring_y[h] = ry;
        shared.ring_z[h] = rz;
        shared.ring_head++;
    }
#endif

    if (shared.cal_phase) {
        /* Calibration: 부팅 후 N초간 raw 평균을 모음. sum_sq는 건너뜀. */
        shared.sum_raw_x += (uint32_t)rx;
        shared.sum_raw_y += (uint32_t)ry;
        shared.sum_raw_z += (uint32_t)rz;
    } else {
        int32_t dx = (int32_t)rx - (int32_t)shared.zero_x;
        int32_t dy = (int32_t)ry - (int32_t)shared.zero_y;
        int32_t dz = (int32_t)rz - (int32_t)shared.zero_z;

        /* 12-bit ADC: |dx| <= ~2048, dx*dx <= 4.2M (int32 safe).
           1 s window (200 samples) <= 8.4e8 < 2^32 (uint32 safe). */
        shared.sum_sq_x += (uint32_t)(dx * dx);
        shared.sum_sq_y += (uint32_t)(dy * dy);
        shared.sum_sq_z += (uint32_t)(dz * dz);
    }
    shared.sample_count++;
    shared.total_samples++;

    ulp_riscv_halt();
    return 0;
}
