#include "sensor.h"

#include <math.h>

uint16_t accel_rms_to_mg(uint32_t sum_sq, int32_t sum_dx, uint32_t n, float sens)
{
    if (n == 0 || sens <= 0.0f) {
        return 0;
    }
    /* 분산 = E[x^2] - (E[x])^2. 윈도우 평균(DC)을 빼므로 자세/기울기로 생긴
       중력 DC가 RMS에 새어들지 않는다. float 반올림으로 음수가 나오면 0으로 클램프. */
    float mean    = (float)sum_dx / (float)n;
    float mean_sq = (float)sum_sq / (float)n;
    float var     = mean_sq - mean * mean;
    if (var < 0.0f) var = 0.0f;
    float rms_cnt = sqrtf(var);
    float rms_mg  = rms_cnt / sens * 1000.0f;
    if (rms_mg < 0.0f)     rms_mg = 0.0f;
    if (rms_mg > 65535.0f) rms_mg = 65535.0f;
    return (uint16_t)rms_mg;
}

float mv_to_resistance(float v_adc_mv)
{
    if (v_adc_mv <= 0.0f) v_adc_mv = 1.0f;

    float r_ntc = THERMISTOR_R_PULLDOWN * ((float)ADC_REF_VOLTAGE_MV - v_adc_mv) / v_adc_mv;
    if (r_ntc <= 0.0f) r_ntc = 1.0f;
    return r_ntc;
}

int16_t resistance_to_temp_steinhart_x100(float r_ntc)
{
    if (r_ntc <= 0.0f) r_ntc = 1.0f;
    float ln_r  = logf(r_ntc);
    float inv_t = STEINHART_A + STEINHART_B * ln_r + STEINHART_C * ln_r * ln_r * ln_r;
    if (inv_t <= 0.0f) inv_t = 1e-6f;
    float temp_k = 1.0f / inv_t;
    return (int16_t)((temp_k - 273.15f) * 100.0f);
}
