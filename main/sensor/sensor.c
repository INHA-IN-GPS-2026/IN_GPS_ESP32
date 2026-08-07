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
