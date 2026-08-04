#include "ulp_init.h"

#include "esp_err.h"
#include "esp_sleep.h"
#include "esp_adc/adc_oneshot.h"
#include "soc/sens_struct.h"

#include "ulp_riscv.h"
#include "ulp_adc.h"

#include "ulp_shared.h"
#include "sensor/sensor.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

/* main에서 보이는 ULP 공유 메모리 (ulp_main.c의 `shared` 심볼) */
extern volatile ulp_shared_t ulp_shared;

/* ULP 기동 주기(µs). 5000(200Hz)이 아니라 5137인 것은 의도적이다 — 정확히
   5000µs 정주기가 스위칭 레귤레이터 주파수와 코히런트하게 물려 공통 온도
   오프셋(~0.6°C)을 만드는지 확인하려고 격자에서 어긋낸 값이다(194.7Hz).
   ※ 2026-07-14 도입 후 결론 미확정. 확정 시 이 주석을 근거로 갱신하거나
     5000으로 원복할 것. */
#define ULP_WAKEUP_PERIOD_US  5137

void start_ulp_adc_measurement(bool do_zero_cal, int16_t zero_x, int16_t zero_y, int16_t zero_z)
{
    /* light sleep 중에도 ADC 패드가 유효해야 ULP가 샘플링을 이어갈 수 있으므로
       RTC 페리페럴 도메인을 항상 켜 둔다. */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    /* HW 핀맵: TH1=GPIO3=CH2, TH2=GPIO4=CH3, ADXL X/Y/Z=GPIO5/6/7=CH4/5/6. */
    ulp_adc_cfg_t adc_config = {
        .adc_n    = ADC_UNIT_1,
        .channel  = ADC_CHANNEL_2,
        .atten    = ADC_ATTEN_DB_12,
        .width    = ADC_BITWIDTH_12,
        .ulp_mode = ADC_ULP_MODE_RISCV,
    };
    ESP_ERROR_CHECK(ulp_adc_init(&adc_config));

    /* ulp_adc_init은 CH2만 설정하므로 나머지 채널(CH3~CH6)의 감쇠를
       레지스터로 직접 DB_12(필드값 3)에 맞춘다. */
    uint32_t atten = SENS.sar_atten1;
    for (int ch = ADC_CHANNEL_3; ch <= ADC_CHANNEL_6; ch++) {
        atten = (atten & ~(0x3U << (ch * 2))) | (3U << (ch * 2));
    }
    SENS.sar_atten1 = atten;

    ESP_ERROR_CHECK(ulp_riscv_load_binary(
        ulp_main_bin_start,
        (size_t)(ulp_main_bin_end - ulp_main_bin_start)
    ));

    if (do_zero_cal) {
        /* 동적 zero 보정 모드로 시작 — 부팅 후 첫 N초간 raw 평균을 측정한 뒤
           app_main에서 zero를 박고 정상 모드(cal_phase=0)로 전환. */
        ulp_shared.zero_x    = 0;
        ulp_shared.zero_y    = 0;
        ulp_shared.zero_z    = 0;
        ulp_shared.sum_raw_x = 0;
        ulp_shared.sum_raw_y = 0;
        ulp_shared.sum_raw_z = 0;
        ulp_shared.cal_phase = 1;
    } else {
        /* fast resume: 알려진 zero를 즉시 적용하고 바로 정상 측정 시작. */
        ulp_shared.zero_x    = zero_x;
        ulp_shared.zero_y    = zero_y;
        ulp_shared.zero_z    = zero_z;
        ulp_shared.cal_phase = 0;
    }
    ulp_shared.sum_sq_x     = 0;
    ulp_shared.sum_sq_y     = 0;
    ulp_shared.sum_sq_z     = 0;
    ulp_shared.sample_count = 0;
    ulp_shared.sum_ntc1     = 0;
    ulp_shared.sum_ntc2     = 0;
    ulp_shared.ntc_count    = 0;

    ESP_ERROR_CHECK(ulp_set_wakeup_period(0, ULP_WAKEUP_PERIOD_US));
    ESP_ERROR_CHECK(ulp_riscv_run());
}
