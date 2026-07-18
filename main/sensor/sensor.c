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

/* ★2026-07-16 저항치환 실험 반영 (Docs/06_저항치환_전자부검증_프로토콜.md,
   resistor_substitution_analysis_20260716.png / 동 CSV에 원데이터·계수):
   [1] ADC 전달함수 실측: raw는 이상값 4095·Rp/(Rp+R) 대비 저전압부 약 −5.4%
       (ADC 기준이 내부 밴드갭이라 FS_eff≈3.48V ≠ Vcc → 비-ratiometric),
       고전압부는 S-curve(+40mV@2.94V, 국소게인 k가 2.87V에서 1.0 관통 0.986까지).
       → 채널별 3차 다항 RAW_CORR로 "실측 raw → 이상 raw" 역변환 후 R 환산.
       ※기존 "ratiometric이라 공급 면역(ADI 권장)" 주석은 틀린 것으로 판명(07-13
         지적, 07-16 실측 확정). Vcc는 약분되지 않는다. 이 보정은 이 보드·이
         전원상태(Vcc≈3.3 가정) 전용 — 전원구성 바뀌면 저항치환 재실측 필요.
         근본책은 여유 ADC의 VCC 센스 채널(계산상 ratiometric 복원), 미구현.
   [2] 보정 유효구간: raw 1941~3701(≈25~91℃ 등가, 요구사항 25~85 커버).
       구간 밖은 3차 외삽이라 신뢰 낮음(단선/단락 fault 검출은 별도 TODO).
   [3] S-H 계수: 07-15 1점 앵커(채널별 A)는 "다른 전원상태의 유물"로 판명되어
       폐기(교차검증 +1.5~1.8℃ 이탈). 데이터시트 8307 R/T표 3점 LSQ(30/55/85,
       두 채널 공통, C=0)로 복귀 — 전자부 보정+이 곡선만으로 PV50 T1 재현
       잔차 −0.2℃(pure Beta 기준)~0.0℃(표 LSQ 기준) 확인됨.
   [4] 남은 미제: PV50에서 T2만 약 −2℃ 상당 이탈 → 전자부 채널차는 실측상
       무죄(k차 ≤0.7%p)이므로 T2 서미스터측(부품/배선/장착) 문제 — 채널 스왑으로
       규명 예정. 50℃ 검증에서 T2가 ~48℃로 나오면 이 가설 확인되는 것. */
static const float RAW_CORR[2][4] = {  /* raw_ideal = ((a3·r + a2)·r + a1)·r + a0 */
    { -5.1369488e-08f, 3.1414055e-04f,  4.0651410e-01f,  448.99270f },  /* [0]=temp1(CH2/GPIO3) */
    { -7.5454282e-08f, 5.3674786e-04f, -2.5842393e-01f, 1089.49845f },  /* [1]=temp2(CH3/GPIO4) */
};
static const float SH_A[2] = { 7.0900845e-04f, 7.0900845e-04f };
static const float SH_B[2] = { 2.8695407e-04f, 2.8695407e-04f };
static const float SH_C[2] = { 0.0f, 0.0f };

int16_t raw_to_temp_x100(float raw, int ch)
{
    if (ch < 0 || ch > 1) ch = 0;
    if (raw < 1.0f)               raw = 1.0f;
    if (raw > ADC_MAX_RAW - 1.0f) raw = ADC_MAX_RAW - 1.0f;

    /* 0) 전자부 보정(2026-07-16 저항치환 실측): 실측 raw → 이상 raw.
          밴드갭 FS 스케일(−5.4%)과 고전압 S-curve를 한꺼번에 되돌린다. */
    const float *a = RAW_CORR[ch];
    raw = ((a[0] * raw + a[1]) * raw + a[2]) * raw + a[3];
    if (raw < 1.0f)               raw = 1.0f;
    if (raw > ADC_MAX_RAW - 1.0f) raw = ADC_MAX_RAW - 1.0f;

    /* 1) raw→R:  R = Rp·(FS − raw)/raw.
          ※주의: 이 식이 공급전압에 면역이라는 과거 주석은 오류였음(ADC 기준 =
          내부 밴드갑, Vcc 약분 안 됨). 위 RAW_CORR가 현 전원상태 기준으로 이를
          보정하며, Vcc 변동 자체는 여전히 미보정(VCC 센스 채널 TODO). */
    float r_ntc = THERMISTOR_R_PULLDOWN * (ADC_MAX_RAW - raw) / raw;
    if (r_ntc < 1.0f) r_ntc = 1.0f;

    /* 2) Steinhart-Hart: 1/T[K] = A + B·ln(R) + C·ln(R)^3 */
    float l = logf(r_ntc);
    float inv_t = SH_A[ch] + SH_B[ch] * l + SH_C[ch] * l * l * l;
    if (inv_t < 1e-6f) inv_t = 1e-6f;   /* div-by-0 가드 */
    float temp_c = (1.0f / inv_t) - 273.15f;

    return (int16_t)(temp_c * 100.0f);
}
