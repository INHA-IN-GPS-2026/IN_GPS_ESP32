# 04. end-to-end Steinhart-Hart 캘리브레이션 — 방법 정리

작성일: 2026-07-08
목적: 부품공차 + ADC 스케일 + S/H 딥 + 자기발열을 **원인 구분 없이 한 번에 흡수**해
써미스터가 기준온도를 정확히 추종하게 만드는 채널별 캘리브레이션(=실험 설계 M5) 정리.
연계: `03_추종검증_5단계_실험설계_및_목표잔차.md`(§3·§5-2), `02_데이터시트_비교_및_채널오차.md`(채널차)

> 📌 **역할(07-08 정리): 이건 "개발/검증용 금본위"지 양산 방법이 아니다.** 채널별 개체
> 캘리브라 **대당 수조 작업이 필요해 양산 불가**. 양산은 **데이터시트 S-H + eFuse adc_cali +
> 100nF 캡 + 정밀저항**(→ 03 §5-2). end-to-end는 "이론상 최대 정확도"를 재서 양산 접근이
> 거기에 얼마나 근접하는지 비교하는 **기준**으로 쓴다. 단, **Thermistor 간 공차를 실제로
> 줄이는 유일한 방법은 이 채널별 캘리브**(또는 더 좁은 부품)라는 점은 기억할 것 —
> adc_cali·공통 S-H로는 공차가 안 줄어든다.

---

## 1. 핵심 개념 — "end-to-end"의 의미

계수를 **어디서 뽑느냐**가 일반 S-H와 다르다.

| | 일반(데이터시트) S-H | **end-to-end S-H** |
|---|---|---|
| 계수 출처 | 데이터시트 R-T 표 | **ESP 자기 raw(=R_app) ↔ 기준온도** |
| 잡는 것 | R→T 변환식만 | 전 신호경로 |
| ADC 오차 | **남음** (틀린 R을 정확히 변환) | **흡수** (틀린 R을 참온도에 맞춤) |

end-to-end는 **NTC→분압→ADC(스케일+딥)→펌웨어 R_app→온도** 전체를 관통해 피팅하므로,
"오차가 낀 R_app을 입력으로 쓰고 출력을 참온도에 억지로 맞춰" 그 사이 모든 오차가 계수에 빨려든다.

## 2. 수식

```
1/T = A + B·ln(R) + C·(ln R)³        (T = 켈빈)
```
- `R` = 펌웨어가 raw에서 계산한 **겉보기저항 R_app** (오차 포함)
- A·B·C = 구할 3계수 — **채널별로 따로**(temp1 ≠ temp2)

## 3. 계수 산출 (3점 → 3×3 연립)

3개 온도에서 각각 `(R_app, T_ref)` 측정. `Lᵢ=ln(R_appᵢ)`, `yᵢ=1/T_refᵢ`:

```
| 1  L₁  L₁³ | | A |   | y₁ |
| 1  L₂  L₂³ | | B | = | y₂ |
| 1  L₃  L₃³ | | C |   | y₃ |
```
풀면 A·B·C. **temp1·temp2 각각** 자기 R_app로 따로 푼다.

## 4. 절차

1. **펌웨어 B 고정**(세 점 동안 불변; 현재 3492). raw 직접 로깅이면 최선, temp만이면 그 B로 역산해 R_app 복원.
2. **35/55/75℃** 각각: 열평형(§5-1 게이팅) → N샘플 평균 → 그때 **PV 실측값** 기록.
3. 채널별 `(R_app, T_ref)` 3쌍 확보.
4. **채널별 3×3 풀어 A·B·C**.
5. 배포: `T = 1/(A + B·ln R_app + C·(ln R_app)³) − 273.15`, temp1/temp2 각 계수로.

## 5. 왜 모든 오차를 흡수하나

3점을 참온도에 **정확히 통과**시키므로, 그 점들의 부품공차·ADC스케일·S/H딥·자기발열·분압오차가
전부 계수로 들어간다(원인 구분 불필요). 점 사이는 보간 — 이 오차들이 온도에 완만해 보간잔차 ~0.1℃.

## 6. 데이터 입력 — CSV로 충분, UART 불필요

- DB에서 뽑은 **2초 CSV(temp1/temp2 + created_at)** 면 됨.
- raw는 `R_app = R0·exp(B_fw·(1/T−1/298.15))`로 temp에서 역산(펌웨어 B_fw 알면 정확 복원).
- temp의 0.01℃ 반올림 = 약 **0.5 LSB** 불확실성(무시 수준, 앞서 확인).
- UART `raw ntc1=` 직접 로깅은 **선택**(반올림 회피, 아주 약간 더 깔끔).

## 7. 주의사항

- **개체별**: ESP+써미스터 조합마다 계수 따로.
- **캘리 시점 고정**: ADC 스케일·딥이 칩온도/노후/전원으로 드리프트 시 계수 낡음(반복성 관건).
- **열매질 정합**: 물 캘리 → 공기 배포면 자기발열 차이. 배포 매질서 캘리하거나 보정.
- **기준 한계**: 캘리 정확도 ≤ 기준계(수조 ±0.1℃ + 균일도). PT100 옆 1cm·교반 필수.
- **범위**: 캘리 범위(35~75℃) 안 유효. 0℃ 쓰려면 빙수 점 추가.
- **점 수**: 3점=딱 통과. 4점↑=최소자승으로 강건(딥 온도의존 보간 개선).

## 8. 분석 스크립트 스켈레톤 (오프라인, CSV 입력)

```python
import numpy as np, math, pandas as pd
K=273.15; B_FW=3492.0; R0=10000.0
def temp_to_Rapp(Tc, B=B_FW): return R0*math.exp(B*(1/(Tc+K)-1/298.15))
def mean_reading(csv, t0, t1, col):     # 열평형 구간 평균
    df=pd.read_csv(csv, parse_dates=['created_at'])
    seg=df[(df.created_at>=t0)&(df.created_at<=t1)]
    return seg[col].mean()
def sh_fit(Rs, Ts):                     # 3점 → A,B,C
    L=np.log(Rs); M=np.column_stack([np.ones(len(Rs)),L,L**3])
    y=[1/(t+K) for t in Ts]
    return np.linalg.solve(M,y) if len(Rs)==3 else np.linalg.lstsq(M,y,rcond=None)[0]
# points: (csv, 시작, 끝, 기준PV)
points=[("p35.csv","2026-07-08 18:19","2026-07-08 18:25",35.0),
        ("p55.csv","...","...",55.0), ("p75.csv","...","...",75.0)]
for col in ['temp1','temp2']:
    Rs=[temp_to_Rapp(mean_reading(c,a,b,col)) for (c,a,b,_) in points]
    Ts=[T for (_,_,_,T) in points]
    A,B,C=sh_fit(Rs,Ts)
    res=[1/(A+B*math.log(R)+C*math.log(R)**3)-K-T for R,T in zip(Rs,Ts)]
    print(f"{col}: A={A:.6e} B={B:.6e} C={C:.6e}  잔차{[round(r,3) for r in res]}")
```

## 9. 런타임 코드 스켈레톤 (`sensor.c`, ULP 불변)

```c
/* 캘리브에서 산출한 채널별 계수 (0=temp1, 1=temp2) */
static const float SH_A[2] = { A_t1, A_t2 };
static const float SH_B[2] = { B_t1, B_t2 };
static const float SH_C[2] = { C_t1, C_t2 };

int16_t rapp_to_temp_x100(uint16_t raw, int ch)
{
    float v = (float)raw / 4095.0f * 3300.0f;      /* raw→겉보기전압 */
    float R = 10000.0f * (3300.0f - v) / v;         /* 겉보기저항 R_app */
    float L = logf(R);
    float Tk = 1.0f / (SH_A[ch] + SH_B[ch]*L + SH_C[ch]*L*L*L);
    return (int16_t)((Tk - 273.15f) * 100.0f);
}
```
- 런타임 비용: `logf` 1회 + 곱셈 몇 회(무시 수준). ULP 그대로.
- 계수 6개(채널당 3개)는 코드 상수 또는 NVS 저장(개체별 값).

## 10. 우리 진행

- 점 **#1 확보**: PV 35.0℃ → temp1 33.96 / temp2 34.47 (B=3492, 열평형 7분 평균).
- **#2(55℃)·#3(75℃)** 대기 → 3점 완성 시 §8 스크립트로 채널별 A·B·C + 잔차 산출 → §9 코드에 반영.
- 목표: temp1·temp2 둘 다 기준 대비 잔차 ≤ ±0.5℃(§03 §4).
