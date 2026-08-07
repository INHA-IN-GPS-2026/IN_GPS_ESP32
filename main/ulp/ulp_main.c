#include <stdint.h>

#include "ulp_riscv.h"
#include "ulp_riscv_adc_ulp_core.h"
#include "ulp_shared.h"

/* ULP-side symbol is "shared"; main side sees it as "ulp_shared". */
volatile ulp_shared_t shared;

/* 한 채널을 오버샘플링해 평균 raw를 반환. 채널 전환 직후 첫 read는 SAR S/H cap의
   이전 채널 잔류전압이라 dummy로 버린다. ULP RISC-V엔 하드웨어 나눗셈이 없으므로
   평균은 >>shift 로 처리한다. */
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
    /* magic만이 아니라 version도 본다. RTC_SLOW_MEM은 리셋을 살아남으므로,
       구조체 레이아웃이 바뀐 펌웨어를 덮어씌우면 magic이 그대로 남아 초기화가
       통째로 건너뛰어지고 옛 오프셋의 값이 새 필드로 읽힌다. NTC 필드 3개
       (12B)를 걷어낸 v5에서 실제로 발생할 수 있는 조합이라 여기서 막는다. */
    if (shared.magic != ULP_MAGIC || shared.version != ULP_VERSION) {
        shared.magic         = ULP_MAGIC;
        shared.version       = ULP_VERSION;
        shared.sum_sq_x      = 0;
        shared.sum_sq_y      = 0;
        shared.sum_sq_z      = 0;
        shared.sum_dx_x      = 0;
        shared.sum_dx_y      = 0;
        shared.sum_dx_z      = 0;
        shared.sample_count  = 0;
        shared.total_samples = 0;
        shared.sum_raw_x     = 0;
        shared.sum_raw_y     = 0;
        shared.sum_raw_z     = 0;
        shared.zero_x        = 0;
        shared.zero_y        = 0;
        shared.zero_z        = 0;
        shared.cal_phase     = 0;
    }

    /* HW 핀맵(Docs/Schemetic/I2C_init_ver.pdf, ESP32-S3: ADC1_CHx = GPIO(x+1)):
       ADXL335 CH4=GPIO5(X), CH5=GPIO6(Y), CH6=GPIO7(Z).

       이 리비전에서 ULP가 읽는 아날로그 채널은 이 3개가 전부다.
       구 NTC 채널(CH2=GPIO3, CH3=GPIO4)은 AS6221 I2C 전환으로 제거됐고,
       두 핀에는 이제 어떤 네트도 붙지 않는다 — 다시 읽지 말 것.
       GPIO8(CH7)은 SDA_OUT(BQ35100 I2C)이므로 마찬가지로 ADC 금지. */
    int16_t rx = (int16_t)adc_read_avg(ADC_CHANNEL_4, ADXL_OVERSAMPLE_SHIFT);
    int16_t ry = (int16_t)adc_read_avg(ADC_CHANNEL_5, ADXL_OVERSAMPLE_SHIFT);
    int16_t rz = (int16_t)adc_read_avg(ADC_CHANNEL_6, ADXL_OVERSAMPLE_SHIFT);

    shared.last_raw_x = rx;
    shared.last_raw_y = ry;
    shared.last_raw_z = rz;

    if (shared.cal_phase) {
        /* 부팅 후 N초간 raw 평균을 모으는 단계. sum_sq는 건너뛴다. */
        shared.sum_raw_x += (uint32_t)rx;
        shared.sum_raw_y += (uint32_t)ry;
        shared.sum_raw_z += (uint32_t)rz;
    } else {
        int32_t dx = (int32_t)rx - (int32_t)shared.zero_x;
        int32_t dy = (int32_t)ry - (int32_t)shared.zero_y;
        int32_t dz = (int32_t)rz - (int32_t)shared.zero_z;

        /* 12-bit ADC: |dx| <= ~2048, dx*dx <= 4.2M (int32 safe).
           1s 윈도우(200샘플) <= 8.4e8 < 2^32 (uint32 safe). */
        shared.sum_sq_x += (uint32_t)(dx * dx);
        shared.sum_sq_y += (uint32_t)(dy * dy);
        shared.sum_sq_z += (uint32_t)(dz * dz);

        shared.sum_dx_x += dx;
        shared.sum_dx_y += dy;
        shared.sum_dx_z += dz;
    }
    shared.sample_count++;
    shared.total_samples++;

    ulp_riscv_halt();
    return 0;
}
