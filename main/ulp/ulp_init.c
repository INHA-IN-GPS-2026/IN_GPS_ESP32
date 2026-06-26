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

void start_ulp_adc_gpio4(void)
{
    /* 진동 누적 동안에도 RTC 도메인은 항상 켜 둠 */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    /* HW 핀맵(스키매틱): TH1=GPIO3=CH2, TH2=GPIO4=CH3, ADXL X/Y/Z=GPIO5/6/7=CH4/5/6.
       CH2 (GPIO3 / NTC1) → ULP RISC-V ADC unit 초기화 */
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

    /* 동적 zero 보정 모드로 시작 — 부팅 후 첫 N초간 raw 평균을 측정한 뒤
       app_main에서 zero를 박고 정상 모드(cal_phase=0)로 전환. sensor.h의
       하드코딩 ZERO 매크로는 더 이상 사용 안 함. */

    ulp_shared.zero_x      = 0;
    ulp_shared.zero_y      = 0;
    ulp_shared.zero_z      = 0;
    ulp_shared.sum_sq_x    = 0;
    ulp_shared.sum_sq_y    = 0;
    ulp_shared.sum_sq_z    = 0;
    ulp_shared.sample_count = 0;

    /* 동적 zero 캘리브레이션 활성화: cal_phase=1로 켜야 ULP가 첫 N초간 sum_raw를
       누적한다. 이걸 안 켜면(기존 버그) zero가 0으로 남아 dx=raw가 되고, 정지
       상태에서도 중력 1g(특히 Z축)가 RMS에 통째로 들어가 진동값이 최대로 뜬다. */
    ulp_shared.sum_raw_x   = 0;
    ulp_shared.sum_raw_y   = 0;
    ulp_shared.sum_raw_z   = 0;
    ulp_shared.cal_phase   = 1;

    /* 5ms = 200Hz 샘플링 */
    ESP_ERROR_CHECK(ulp_set_wakeup_period(0, 5000));
    ESP_ERROR_CHECK(ulp_riscv_run());
}
