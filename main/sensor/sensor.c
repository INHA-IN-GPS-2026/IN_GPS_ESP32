#include "sensor.h"

#include <math.h>

int16_t raw_to_temp_x100(uint16_t raw){
    float v_adc = (float)raw / ADC_MAX_RAW * (float)ADC_REF_VOLTAGE_MV;
    if (v_adc <= 0.0f) v_adc = 1.0f;

    float r_ntc = THERMISTOR_R_PULLDOWN * ((float)ADC_REF_VOLTAGE_MV - v_adc) / v_adc;
    if (r_ntc <= 0.0f) r_ntc = 1.0f;

    float temp_k = 1.0f / (1.0f / THERMISTOR_T0 +
                            logf(r_ntc / THERMISTOR_R0) / THERMISTOR_BETA);
    float temp_c = temp_k - 273.15f;
    return (int16_t)(temp_c * 100.0f);
}

void raw_to_accel_angles(int16_t rx, int16_t ry, int16_t rz,
                         int16_t *ax100, int16_t *ay100, int16_t *az100){
    float gx = ((float)rx - ADXL335_ZERO_X) / ADXL335_SENS_X;
    float gy = ((float)ry - ADXL335_ZERO_Y) / ADXL335_SENS_Y;
    float gz = -((float)rz - ADXL335_ZERO_Z) / ADXL335_SENS_Z;

    /* Roll  (X축 주위): atan2(gy, gz) → -180° ~ +180° 전체 범위 */
    float angle_x = atan2f(gy, gz) * (180.0f / (float)M_PI);
    /* Pitch (Y축 주위): atan2(-gx, √(gy²+gz²)) → -90° ~ +90° */
    float angle_y = atan2f(-gx, sqrtf(gy*gy + gz*gz)) * (180.0f / (float)M_PI);
    /* Tilt  (Z 수직 기울기): -90° ~ +90° */
    float angle_z = atan2f(gz, sqrtf(gx*gx + gy*gy)) * (180.0f / (float)M_PI);

    *ax100 = (int16_t)(angle_x * 100.0f);
    *ay100 = (int16_t)(angle_y * 100.0f);
    *az100 = (int16_t)(angle_z * 100.0f);
}

// ADXL335 data rms convert (200hz/sample)
//sudo -code
// float rms_data[] = sqrt((x^2+y^2+z^2)) if 200 else 0
// 팀장님 에게 전달 받은 내용 Data rms로 압축해서 표현 x축 진동 y축 진동 z축 진동데이터로
// 변환 후 raw 값 전달

