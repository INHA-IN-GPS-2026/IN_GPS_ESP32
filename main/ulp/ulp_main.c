#include <stdint.h>

#include "ulp_riscv.h"
#include "ulp_riscv_adc_ulp_core.h"
#include "ulp_shared.h"

/* ULP-side symbol is "shared"; main side sees it as "ulp_shared". */
volatile ulp_shared_t shared;

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
    }

    /* 채널 전환 직후엔 SAR 내부 sample-and-hold cap이 이전 채널 잔류전압을 유지하므로
       첫 read는 dummy로 버리고 두 번째 read만 사용. NTC(소스 임피던스 5~10kΩ) 측정의
       채널 간 누설을 막고 ADXL 측정 정밀도도 함께 개선. */
    (void)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_3);
    shared.last_raw_ntc1 = (int16_t)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_3);
    (void)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_4);
    shared.last_raw_ntc2 = (int16_t)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_4);

    /* ADXL335: CH5=GPIO6 (X), CH6=GPIO7 (Y), CH8=GPIO9 (Z, GPIO8 회피) */
    (void)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_5);
    int16_t rx = (int16_t)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_5);
    (void)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_6);
    int16_t ry = (int16_t)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_6);
    (void)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_8);
    int16_t rz = (int16_t)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_8);

    shared.last_raw_x = rx;
    shared.last_raw_y = ry;
    shared.last_raw_z = rz;

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
