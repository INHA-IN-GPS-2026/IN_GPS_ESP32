# ADXL Raw 캡처 → FFT 노이즈 분석 가이드

ESP32-S3가 ULP ADC로 보는 raw(X/Y/Z, counts)를 200Hz로 UART에 흘려서,
`adxl_noise_fft.py`로 PSD/FFT 분석하기 위한 절차.

## 1. 무엇이 추가됐나 (펌웨어)

- `main/ulp_shared.h`
  - `ADXL_RAW_CAPTURE` 플래그(기본 1 = 캡처 ON), `RAW_RING_LEN 256` 링버퍼.
  - 공유 구조체에 `ring_head` + `ring_x/y/z[256]` (단일 생산자 ULP / 단일 소비자 main).
- `main/ulp/ulp_main.c`
  - 매 5ms 사이클마다 rx/ry/rz를 링버퍼에 적재(플래그 가드).
- `main/app_main.c`
  - `adxl_raw_capture(60)`: 캘리브레이션 직후 60초간 링버퍼를 드레인해
    `x,y,z` CSV를 UART로 출력. 캡처 동안 로그레벨을 ERROR로 낮춰 CSV를 깨끗하게.
  - **BLE init 전에** 호출 → TX 노이즈 없는 "ADC 베이스라인"을 먼저 확보.

> 무손실성: 200Hz 페이스에서 main이 20ms마다 ~4샘플씩 드레인, 버퍼는 1.28s분.
> 정상 동작 시 오버런 0(호스트 시뮬레이션으로 consumed+overrun==produced 검증).

## 2. 빌드 & 플래시 (캡처 모드)

```bash
idf.py build
idf.py -p (PORT) flash
```

`ulp_shared.h`의 `ADXL_RAW_CAPTURE`가 1이면 캡처 모드로 빌드된다.

## 3. 캡처 (정지 상태 60초)

기기를 **수평·정지** 상태로 둔 채 보드를 리셋(EN). 캘리브레이션(10초) 후
아래 마커 사이로 CSV가 쏟아진다.

```
# ADXL_RAW_CAPTURE BEGIN cols=x,y,z units=counts fs=200
1923,1860,1965
...
# ADXL_RAW_CAPTURE END n=12000 elapsed_ms=60000 (fs=n*1000/elapsed_ms)
```

### 자동 저장 (권장)

```bash
python scripts/capture_serial.py --port COM5 --baud 115200 \
    --out data/esp32_still_01.csv
```
(BEGIN~END 구간의 숫자 3열만 골라 저장. `idf.py monitor`는 동시에 띄우지 말 것 —
포트 점유 충돌.)

### 수동 저장 (PuTTY 등)

PuTTY: Session > Logging > "All session output" → 파일 지정. 캡처 후
BEGIN/END 마커 바깥의 부팅 로그 줄만 지우면 된다.

## 4. 분석

```bash
python scripts/adxl_noise_fft.py data/esp32_still_01.csv --fs 200 --units counts \
    --sens-x 406.845 --sens-y 407.095 --sens-z 399.405
```

출력: 축별 노이즈밀도(µg/√Hz) vs ADXL335 데이터시트, 50/60Hz·BLE 피크,
대역 RMS. `output/figures/*_asd.png` + `*_report.txt` 저장.

**해석 포인트**
- ESP32 floor가 오실로스코프(아날로그) 측정 floor보다 몇 배 높은지 = ADC가
  얹는 노이즈량. (스코프는 데이터시트 1.0~1.6배로 이미 확인됨.)
- 화이트하게 높으면 → 멀티샘플링/오버샘플링 + ADC 입력 100nF (ESP-IDF ADC
  Calibration 문서 "Minimize Noise" 공식 권고).
- 60Hz·배수 피크 → 전원/접지 커플링. BLE 인터벌 피크 → TX 스파이크.

## 5. BLE 영향까지 보려면

`app_main.c`에서 `adxl_raw_capture(60)` 호출을 `nimble_port_freertos_init()`
뒤로 옮겨 다시 캡처 → 광고 ON/OFF 구간의 노이즈 변화를 같은 스크립트로 비교.

## 6. 운영 펌웨어로 복귀

`main/ulp_shared.h`에서:
```c
#define ADXL_RAW_CAPTURE 0
```
→ 링버퍼·캡처 코드가 전부 컴파일에서 빠지고 원래 sum_sq/RMS 경로만 남는다.
