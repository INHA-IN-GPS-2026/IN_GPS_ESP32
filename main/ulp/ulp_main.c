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

    shared.rpt.last_raw[0]  = (int16_t)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_3); // GPIO4
    shared.rpt.last_raw[1]  = (int16_t)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_4); // GPIO5
    shared.rpt.last_raw[2]  = (int16_t)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_5); // GPIO6
    shared.rpt.extra_raw[0] = (int16_t)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_6); // GPIO7
    shared.rpt.extra_raw[1] = (int16_t)ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_7); // GPIO8
    shared.rpt.sample_counter++;

    ulp_riscv_halt();
    return 0;
}