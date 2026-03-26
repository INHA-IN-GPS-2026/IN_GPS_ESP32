# IN-GPS (Firmware, ESP32-S3)

IN-GPS 임베디드 펌웨어 초기버전입니다.
현재 ESP32-S3(ULP ADC 수집) → BLE 광고 패킷 → Gateway → Broker 까지 진행되었습니다.

## Project Structure

```
main/
├── app_main.c              # 진입점, NimBLE 초기화
├── ble/
│   ├── ble_adv.h/.c        # BLE 광고 패킷 구성 및 사이클 관리
├── sensor/
│   ├── sensor.h/.c         # NTC 온도 변환, ADXL335 각도 변환
├── ulp/
│   ├── ulp_main.c          # ULP RISC-V 코어 (ADC 샘플링, 200ms 주기)
│   ├── ulp_init.h/.c       # ULP 초기화 및 바이너리 로드
├── ulp_shared.h            # ULP ↔ Main CPU 공유 메모리 구조체
```

## Features

- ULP RISC-V가 200ms 주기로 ADC 5채널(GPIO4~8) 샘플링 (메인 CPU 절전 가능)
- NTC 서미스터 2채널 온도 측정 (GPIO4, GPIO5)
- ADXL335 가속도센서 3축 각도 계산 (GPIO6, GPIO7, GPIO8)
- BLE Manufacturer Specific Data로 센서값 광고 (0.5s ON / 5s OFF 사이클)

## BLE Advertising Packet (Manufacturer Specific Data)

| Offset | 크기 | 내용 |
|--------|------|------|
| 0~1    | 2B   | Company ID `0x1234` (LE) |
| 2~3    | 2B   | temp1 × 100, int16 LE (GPIO4) |
| 4~5    | 2B   | temp2 × 100, int16 LE (GPIO5) |
| 6~7    | 2B   | angle_x × 100, int16 LE |
| 8~9    | 2B   | angle_y × 100, int16 LE |
| 10~11  | 2B   | angle_z × 100, int16 LE |
| 12     | 1B   | reason |

ex) `temp1 = 2550` → 25.50°C / `angle_x = 4523` → 45.23°

## Hardware

| 핀 | 용도 |
|----|------|
| GPIO4 (ADC1_CH3) | NTC 서미스터 #1 |
| GPIO5 (ADC1_CH4) | NTC 서미스터 #2 |
| GPIO6 (ADC1_CH5) | ADXL335 X축 |
| GPIO7 (ADC1_CH6) | ADXL335 Y축 |
| GPIO8 (ADC1_CH7) | ADXL335 Z축 |

## Requirements

- ESP-IDF: 5.x
- Target: ESP32-S3
- NimBLE (bt component)
- ULP RISC-V 지원 필요

## Sensor Calibration

[sensor/sensor.h](main/sensor/sensor.h) 상단의 define 값을 측정 환경에 맞게 수정:

1. 보드를 수평으로 놓고 `raw_x/y/z` 로그 기록 → `ADXL335_ZERO_*` 업데이트
2. 각 축을 수직으로 세워 `raw` 기록 → `ADXL335_SENS_*` 업데이트
3. 재빌드 및 플래시

## Build & Flash


# 빌드
idf.py build

# 플래시 및 모니터
idf.py -p (PORT) flash monitor
```

## Gateway 연동

- Gateway는 BLE 스캔 시 Device Name `IN_GPS` + Company ID `0x1234` 로 1차 필터링
- 이후 BLE Source MAC을 화이트리스트에 자동 등록하여 기기별 구분
