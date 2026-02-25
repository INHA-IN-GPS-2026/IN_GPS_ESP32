#include <stdint.h>

#include "ulp_riscv.h"
#include "ulp_riscv_adc_ulp_core.h"
#include "ulp_shared.h"

volatile ulp_shared_t shared; // main에서는 ulp_shared 로 보임

int main(void)
{
    if (shared.magic != ULP_MAGIC) {
        shared.magic = ULP_MAGIC;
        shared.version = ULP_VERSION;
        shared.rpt.sample_counter = 0;
        shared.rpt.last_raw[0] = 0;
    }

    uint32_t raw = ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_3);
    shared.rpt.last_raw[0] = (int16_t)raw;
    shared.rpt.sample_counter++;

    ulp_riscv_halt();
    return 0;
}