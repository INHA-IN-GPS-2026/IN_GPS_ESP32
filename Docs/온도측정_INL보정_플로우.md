# 온도 측정 파이프라인 — ADC INL 보정 + Steinhart-Hart (재설계 2026-07-21)

## 1. 왜 이 재설계인가 — 지배 오차는 ADC INL

온도채널 오차를 분해하면 지배항은 센서도 노이즈도 아니라 **ESP32-S3 ADC의 비선형(INL, S-curve)** 이다. 저항치환/항온수조 실험(7 레벨, code 1940~3558)으로 실측한 오차예산:

| 오차원 | 크기 | 온도 환산(worst) | 성격 |
|---|---|---|---|
| ADC 노이즈 σ | 0.5 ~ 0.9 LSB | ~0.02 ℃ | 랜덤, 창평균으로 √N 저감 |
| 채널 mismatch (T1−T2) | 0.4 ~ 7.6 LSB | ~0.1 ~ 0.4 ℃ | **common-mode라 두 센서 차분에선 상쇄** |
| **ADC 비선형 (INL)** | **최대 ~35 LSB ≈ 28 mV** | **최대 ~1.7 ℃** | 코드 의존 결정론적 곡선, **지배항** |
| 센서 모델(S-H) | — | ~0.001 ℃ | 무시 가능 |

핵심 함정: "써미스터 두 개를 붙이면 거의 안 벌어진다"는 관찰은 INL이 작다는 증거가 **아니다**. 같은 온도 → 같은 코드 → 같은 INL이라 T1−T2에서 상쇄될 뿐이다. INL은 절대 기준(저항/전압)으로만 드러나며, 위 표처럼 노이즈의 40~70배로 지배한다. 그래서 0.1℃ 목표에서 손대야 할 곳은 **오직 INL 보정** 하나다.

## 2. 설계 원칙 두 가지

### (a) INL은 per-chip → 직접 곡선 만들지 말고 eFuse를 쓴다
INL은 칩마다 다르다(Vref 1000~1200mV, 개체편차 ~15%). 따라서 한 대에서 뽑은 곡선을 다른 기기에 구우면 틀린다. Espressif가 공장에서 **칩마다** 특성화해 eFuse에 구운 curve fitting 계수를 그대로 쓴다(`adc_cali`). 공짜로 per-chip 보정이 된다.

### (b) 센서 모델은 단일 Steinhart-Hart — 밴딩 불필요
S-H 3계수 하나로 15~75℃ 전 구간 잔차 **0.6 mC**(목표 0.1℃의 1/160). 구간별(15-35/35-55/55-75) 계수 스위칭은 이득 0이고 경계 불연속·chattering 리스크만 있어 **하지 않는다**. (밴딩은 Beta 모델의 한계였고, S-H 전환이 그 필요를 없앤다.)

## 3. 전체 데이터 플로우

```
[ULP RISC-V, 200Hz, 항상 가동 / deep sleep도 동일]
  GPIO3=CH2(TH1), GPIO4=CH3(TH2) raw 읽기
      │  (채널전환 dummy read → S/H 정착)
      ▼
  sum_ntc1/2 += raw ,  ntc_count++      ← raw만 누적 (adc_cali 호출 불가)
      │
──────┼──────────────────────────────  ulp_shared (RTC_SLOW_MEM, sleep 유지)
      ▼
[메인 CPU, 1초 주기, ble_adv.c: build_mfg_data()]
  avg_raw = sum_ntc / ntc_count                     (창평균 → 노이즈 √N 저감)
      ▼
  v_mV = adc_cal_raw_to_mv(avg_raw)                 ★ per-chip eFuse curve fitting = INL 보정
      ▼
  R = mv_to_resistance(v_mV)                        분압 역산: R = Rpd·(Vcc−Vadc)/Vadc
      ▼                                              (VCC→NTC→ADC→R_pulldown→GND, Rpd=10k, Vcc=3300mV)
  T×100 = resistance_to_temp_steinhart_x100(R)      1/T = A + B·lnR + C·(lnR)³
      ▼
  BLE mfg_data[2..5] ← temp1/temp2 (°C×100 int16 LE)  게이트웨이 포맷 하위호환
```

보정이 **메인 CPU의 raw→mV 단계**에 들어간다는 것이 요점이다. ULP는 raw만 쌓고, 그 칩의 eFuse 곡선을 메인이 적용하므로 per-chip으로 정확하다.

## 4. Deep / Light sleep 동작

| 모드 | 보정 계층 | 동작 |
|---|---|---|
| Awake / Light sleep | 살아있음 | 메인 CPU + cali 핸들이 RAM 유지 → curve fitting 그대로 적용 |
| **Deep sleep** | ULP는 raw만 | ULP가 `sum_ntc` 누적 → 깨어난 메인이 `adc_cal_raw_to_mv`로 일괄 보정 = **per-chip 정답** |

즉 이 파이프라인은 deep sleep에서도 그대로 유효하다. ULP에 고정소수점 다항식을 이식할 필요가 없다(그 방식은 실시간 in-sleep 판단이 필요할 때만). sleep 중 온도 임계 판단이 필요하면, 임계 온도를 메인에서 raw 코드로 역변환해 RTC 메모리에 넣고 ULP는 raw끼리 비교한다.

## 5. 코드 맵 (이번 변경)

| 파일 | 역할 |
|---|---|
| `sensor/adc_cal.h/.c` | **(신규)** eFuse curve fitting 래퍼. `adc_cal_init()`, `adc_cal_raw_to_mv()`. 미소성 시 선형 폴백 |
| `sensor/sensor.h/.c` | S-H 계수 갱신(전 구간 LSQ). `mv_to_resistance()` 신설(전압→저항), `raw_to_resistance()`는 선형 폴백으로 잔존 |
| `ble/ble_adv.c` | `build_mfg_data()`가 `adc_cal_raw_to_mv → mv_to_resistance → S-H` 경로 사용. 페이로드 온도를 **S-H(+INL보정)** 로 전환. 로그에 `v=..mV cal=0/1` 추가 |
| `app_main.c` | 부팅 시 `adc_cal_init()` 호출 |
| `main/CMakeLists.txt` | `sensor/adc_cal.c` 등록 |

## 6. 상수 / 계수

```
NTC: B57541G, 8307 특성, B25/100 = 3492 K, R25 = 10 kΩ
분압: VCC → NTC → ADC → R_pulldown(10k) → GND,  Vcc = 3300 mV
ADC: UNIT_1, ATTEN_DB_12, 12-bit  (ULP 설정과 일치해야 curve fitting 유효)

Steinhart-Hart (1/T[K] = A + B·lnR + C·(lnR)³, R25=10k, 15~75℃ LSQ):
  A = 8.4781767957e-04
  B = 2.6110639311e-04
  C = 1.2967307160e-07
  → 15~75℃ 잔차 피크 0.6 mC
```

## 7. 검증 & 남은 일

- **빌드/실측 검증**: `idf.py build flash monitor` 후 로그의 `therm T1/T2 ... v=..mV cal=1` 확인. `cal=1`이면 per-chip 보정 활성. 기준 온도계와 15/45/75℃ 비교해 ±0.1℃ 확인.
- **순수 INL 곡선 확정(선택)**: 현재 저항치환 7점은 divider 모델오차가 섞인 상한치. 정밀 전압원을 ADC 핀에 직접 50~100mV 간격 스윕하면 eFuse 곡선 잔차(순수 INL)를 검증할 수 있다.
- **채널 오차(≈0.19℃) 대응**: 0.1℃ 절대정확도가 빡세면 per-unit **1점 offset**(생산 시 지그)만 추가. 전체 per-unit 곡선은 <0.05℃ 목표가 아니면 불필요.
- **참고**: 온도 경로를 AS6221(I2C 디지털)로 전환하면 ADC 자체가 사라져 INL 문제가 통째로 소멸한다(`I2C AS6221 전환` 결정 참조). 이 파이프라인은 NTC+ADC 경로를 유지하는 경우의 최적안이다.
