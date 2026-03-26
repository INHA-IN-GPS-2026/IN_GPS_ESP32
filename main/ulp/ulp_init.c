#include "ulp_init.h"

#include "esp_err.h"
#include "esp_sleep.h"
#include "esp_adc/adc_oneshot.h"
#include "soc/sens_struct.h"

#include "ulp_riscv.h"
#include "ulp_adc.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

void start_ulp_adc_gpio4(void)
{
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    ulp_adc_cfg_t adc_config = {
        .adc_n    = ADC_UNIT_1,
        .channel  = ADC_CHANNEL_3,      // GPIO4 = ADC1_CH3
        .atten    = ADC_ATTEN_DB_12,
        .width    = ADC_BITWIDTH_12,
        .ulp_mode = ADC_ULP_MODE_RISCV,
    };
    ESP_ERROR_CHECK(ulp_adc_init(&adc_config));

    // CH4~CH7(GPIO5~8): ADC_ATTEN_DB_12 = 3
    uint32_t atten = SENS.sar_atten1;
    for (int ch = ADC_CHANNEL_4; ch <= ADC_CHANNEL_7; ch++) {
        atten = (atten & ~(0x3U << (ch * 2))) | (3U << (ch * 2));
    }
    SENS.sar_atten1 = atten;

    ESP_ERROR_CHECK(ulp_riscv_load_binary(
        ulp_main_bin_start,
        (size_t)(ulp_main_bin_end - ulp_main_bin_start)
    ));

    ESP_ERROR_CHECK(ulp_set_wakeup_period(0, 200000)); // 200ms
    ESP_ERROR_CHECK(ulp_riscv_run());
}
