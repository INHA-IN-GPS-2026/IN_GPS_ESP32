# Reference — IN-GPS Firmware (ESP32-S3 + ADXL335)

ADXL335 캘리브레이션·노이즈 분석 작업 중 근거로 확인한 공식 문서/데이터시트 모음.
모두 1차 출처(Espressif / Analog Devices)이며, 직접 확인용 링크다.

> 참고: 본 펌웨어는 **ESP-IDF 5.x** 기준. 아래 API 문서는 `latest`로 걸어뒀으니,
> 페이지 우측 상단 버전 드롭다운에서 실제 사용하는 5.x 버전으로 바꿔서 보면 된다.
> (확인일: 2026-06-21)

---

## 1. ESP32-S3 ADC (SAR ADC) — 노이즈의 핵심 경로

오실로스코프(아날로그)는 데이터시트 근처로 깨끗한데 ESP32 로그에서 노이즈가 더
끼는 원인이 이 SAR ADC 경로다. Vref 편차(1.0~1.2V), 12-bit 단발 변환 노이즈,
멀티샘플링 권장 등이 여기에 정리돼 있다.

- **ADC Calibration Driver (ESP32-S3)** ★ 노이즈 대책 직접 근거
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/adc/adc_calibration.html
  - eFuse 기반 HW/SW 캘리브레이션, raw→mV 변환. Vref 1100mV(1000~1200mV) 편차.
  - **"Minimize Noise" 섹션 원문**: "The ESP32-S3 ADC is sensitive to noise...
    connect a bypass capacitor (e.g., a 100 nF ceramic capacitor) to the ADC
    input pad... multisampling may also be used to further mitigate the effects
    of noise." → ADC 입력에 100nF + **멀티샘플링(오버샘플링)**이 공식 권고.

- **ADC Oneshot Mode Driver (ESP32-S3)**
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/adc/adc_oneshot.html
  - 단발 변환 API, atten 옵션, raw→전압 공식, ULP mode 설정.

- **ADC Continuous (DMA) Mode Driver (ESP32-S3)** — 고속/연속 샘플링 시 참고
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/adc/adc_continuous.html

## 2. ULP RISC-V 코프로세서 — 현재 200Hz 샘플러가 도는 곳

- **ULP RISC-V Coprocessor Programming (ESP32-S3)**
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/system/ulp-risc-v.html
  - ulp_riscv_run(), 웨이크업 타이머(RTC_SLOW_CLK ~90kHz), RTC 도메인 주변장치 접근.
  - 채널당 N회 읽어 평균하는 **오버샘플링**을 ULP 루프에 넣을 때 기준 문서.

- **ULP RISC-V ADC 예제 (GitHub, esp-idf)**
  https://github.com/espressif/esp-idf/tree/master/examples/system/ulp/ulp_riscv/adc
  - ULP에서 주기적으로 ADC 읽는 레퍼런스 구현.

## 3. ESP32-S3 하드웨어 1차 문서

- **ESP32-S3 Datasheet (PDF)**
  https://documentation.espressif.com/esp32-s3_datasheet_en.pdf
  - SAR ADC 전기적 특성, 핀/atten 사양.
  - 미러: https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf

- **ESP32-S3 Technical Reference Manual (PDF)**
  https://documentation.espressif.com/esp32-s3_technical_reference_manual_en.pdf
  - SAR ADC 아키텍처, RTC/DIG ADC 컨트롤러, ADC 필터/threshold 등 레지스터 레벨.

- **ESP32-S3 Hardware Design Guidelines (PDF)** — 전원/접지/디커플링
  https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/esp-hardware-design-guidelines-en-master-esp32s3.pdf
  - 아날로그 레일 분리, 디커플링, 레이아웃. ADXL Vs 노이즈/접지 커플링 대책 근거.

## 4. ADXL335 가속도 센서

- **ADXL335 제품 페이지 (Analog Devices)**
  https://www.analog.com/en/products/adxl335.html

- **ADXL335 Datasheet (PDF, Rev. B)**
  https://www.analog.com/media/en/technical-documentation/data-sheets/adxl335.pdf
  - 노이즈 밀도: X/Y ~150 µg/√Hz, Z ~300 µg/√Hz (typ) — 분석 스크립트의 비교 기준.
  - 출력 대역폭 = Cx/Cy/Cz 캡으로 설정(X/Y 0.5~1600Hz, Z 0.5~550Hz). **안티앨리어싱**.
  - 감도 ~300mV/g(@Vs=3V, 비례=ratiometric), 단일전원 1.8~3.6V.

---

## 작업 맥락 메모

- 분석 스크립트: `ml_validation/scripts/adxl_noise_fft.py`
  (counts/volts → µg/√Hz, Welch PSD, 50/60Hz·BLE 피크 자동검출, 데이터시트 비교)
- 결론(현재까지): 아날로그/센서는 데이터시트 1.0~1.6배로 정상권.
  초과 노이즈는 ESP32-S3 SAR ADC 경로에서 유입 → 대책 우선순위:
  (1) ULP 채널당 N회 평균(오버샘플링, √N 저감) → 위 2번 ULP 문서
  (2) ADXL Vs 디커플링/레일 분리 → 위 3번 HW 가이드라인
  (3) 출력 캡(Cx) 점검 + 샘플레이트 ≥ 2.5×BW(안티앨리어싱) → 위 4번 ADXL 데이터시트
