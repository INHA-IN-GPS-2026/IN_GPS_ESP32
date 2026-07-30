#include "sensor/adc_cal.h"
#include "sensor/sensor.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "ADC_CAL";

static adc_cali_handle_t s_handle = NULL;

bool adc_cal_init(void)
{
    if (s_handle != NULL) {
        return true;
    }

    /* ULP ADC 설정과 반드시 일치: unit1 / 12dB atten / 12-bit.
       curve fitting은 채널이 아니라 (unit, atten) 단위로 특성화되므로
       .chan은 지정하지 않는다(구조체에 필드가 있어도 0으로 무시됨). */
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    esp_err_t err = adc_cali_create_scheme_curve_fitting(&cfg, &s_handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "curve fitting cali ENABLED (per-chip eFuse, INL corrected)");
        return true;
    }

    s_handle = NULL;
    /* ESP_ERR_NOT_SUPPORTED = eFuse 캘리브 비트 미소성. 정품 Espressif 모듈은
       공장 소성되어 있으므로 보통 발생하지 않음. 발생 시 선형 폴백으로 동작. */
    ESP_LOGW(TAG, "curve fitting cali unavailable (%s) -> LINEAR fallback (INL uncorrected)",
             esp_err_to_name(err));
    return false;
}

bool adc_cal_is_enabled(void)
{
    return s_handle != NULL;
}

float adc_cal_raw_to_mv(uint16_t raw)
{
    if (s_handle != NULL) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(s_handle, (int)raw, &mv) == ESP_OK) {
            return (float)mv;
        }
    }
    /* 폴백: 선형 환산(INL 미보정). */
    return (float)raw / ADC_MAX_RAW * (float)ADC_REF_VOLTAGE_MV;
}

/* 보간 반폭(코드). 상세 트레이드오프는 adc_cal.h 주석 참고.
   1로 두면 IDF 원본 곡선을 거의 그대로(1mV 계단 포함) 본다. */
#ifndef ADC_CAL_INTERP_SPAN
#define ADC_CAL_INTERP_SPAN 16
#endif

float adc_cal_raw_frac_to_mv(float raw)
{
    if (raw < 0.0f)          raw = 0.0f;
    if (raw > ADC_MAX_RAW)   raw = ADC_MAX_RAW;

    if (s_handle == NULL) {
        return raw / ADC_MAX_RAW * (float)ADC_REF_VOLTAGE_MV;
    }

    /* raw를 사이에 두는 두 점을 잡되, 양 끝단에서는 구간을 안쪽으로 민다. */
    int lo = (int)raw - ADC_CAL_INTERP_SPAN;
    int hi = (int)raw + ADC_CAL_INTERP_SPAN;
    const int max_raw = (int)ADC_MAX_RAW;
    if (lo < 0)        { lo = 0;       hi = 2 * ADC_CAL_INTERP_SPAN; }
    if (hi > max_raw)  { hi = max_raw; lo = hi - 2 * ADC_CAL_INTERP_SPAN; }
    if (lo < 0)        { lo = 0; }
    if (hi <= lo)      { return adc_cal_raw_to_mv((uint16_t)(raw + 0.5f)); }

    int mv_lo = 0, mv_hi = 0;
    if (adc_cali_raw_to_voltage(s_handle, lo, &mv_lo) != ESP_OK ||
        adc_cali_raw_to_voltage(s_handle, hi, &mv_hi) != ESP_OK) {
        return adc_cal_raw_to_mv((uint16_t)(raw + 0.5f));
    }

    return (float)mv_lo
         + (raw - (float)lo) * (float)(mv_hi - mv_lo) / (float)(hi - lo);
}
