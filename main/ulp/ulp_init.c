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

void start_ulp_adc_measurement(bool do_zero_cal, int16_t zero_x, int16_t zero_y, int16_t zero_z)
{
    /* 진동 누적 동안에도 RTC 도메인은 항상 켜 둠(현재는 app_main에서 부팅 시
       1회만 호출 — 상시가동 구조, [[ingps-deepsleep-redesign]] 참고). */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    /* HW 핀맵(스키매틱): TH1=GPIO3=CH2, TH2=GPIO4=CH3, ADXL X/Y/Z=GPIO5/6/7=CH4/5/6.
       CH2 (GPIO3 / NTC1) → ULP RISC-V ADC unit 초기화. */
    ulp_adc_cfg_t adc_config = {
        .adc_n    = ADC_UNIT_1,
        .channel  = ADC_CHANNEL_2,
        .atten    = ADC_ATTEN_DB_12,
        .width    = ADC_BITWIDTH_12,
        .ulp_mode = ADC_ULP_MODE_RISCV,
    };
    ESP_ERROR_CHECK(ulp_adc_init(&adc_config));

    /* CH3 (NTC2), CH4~6 (ADXL X/Y/Z) attenuation DB_12 (필드값=3) */
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
        /* (현재 미사용 경로 — 딥슬립 재설계용으로 남겨둠) 알려진 zero를 즉시
           적용하고 cal_phase=0으로 바로 정상 측정 시작. */
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
#if NTC_RAW_CAPTURE
    ulp_shared.ntc_ring_head = 0;
#endif

    /* 5ms = 200Hz 샘플링
       ★진단용 임시 변경(2026-07-14, 유지): 5000us 정주기가 스위칭 레귤레이터
       주파수와 코히런트하게 물려 35도 공통오프셋(~0.6도)을 만드는지 검증 중.
       원복하려면 5000으로 되돌릴 것. */
    ESP_ERROR_CHECK(ulp_set_wakeup_period(0, 5137));
    ESP_ERROR_CHECK(ulp_riscv_run());
}
