#include "sensor.h"

#include <math.h>

uint16_t accel_rms_to_mg(uint32_t sum_sq, uint32_t n, float sens)
{
    if (n == 0 || sens <= 0.0f) {
        return 0;
    }
    float rms_cnt = sqrtf((float)sum_sq / (float)n);
    float rms_mg  = rms_cnt / sens * 1000.0f;
    if (rms_mg < 0.0f)     rms_mg = 0.0f;
    if (rms_mg > 65535.0f) rms_mg = 65535.0f;
    return (uint16_t)rms_mg;
}

int16_t raw_to_temp_x100(uint16_t raw)
{
    float v_adc = (float)raw / ADC_MAX_RAW * (float)ADC_REF_VOLTAGE_MV;
    if (v_adc <= 0.0f) v_adc = 1.0f;

    float r_ntc = THERMISTOR_R_PULLDOWN * ((float)ADC_REF_VOLTAGE_MV - v_adc) / v_adc;
    if (r_ntc <= 0.0f) r_ntc = 1.0f;

    float temp_k = 1.0f / (1.0f / THERMISTOR_T0 +
                           logf(r_ntc / THERMISTOR_R0) / THERMISTOR_BETA);
    float temp_c = temp_k - 273.15f;
    return (int16_t)(temp_c * 100.0f);
}
