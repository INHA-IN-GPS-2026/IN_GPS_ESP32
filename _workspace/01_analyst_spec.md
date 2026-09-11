# ADR: ULP 30분 수집 / 직접 BLE GATT 다운로드

- 날짜: 2026-09-11. 사용자 요청에 따라 구현 진행.
- 기반: `81d0551`, `feature/ulp-30min-ble-logger`. 시작 시 작업 트리 clean.
- 원 설계: `C:/Users/YechanLee/_workspace/ingps_deepsleep_architecture_2026-09-06.md`.

## 수용 기준과 결정

1. 콜드 부팅 후 1,800초 동안 Bluetooth 초기화/광고/연결 없이 수집한다.
2. HP는 Deep Sleep, ULP RISC-V만 GPIO8/9 open-drain software I2C를 소유한다. HP는 30초마다 통신 없이 ULP 진행을 점검한다.
3. ADXL345 100 Hz FIFO를 약 150 ms 수면 + 처리시간 간격으로 비운다. AS6221 TH1/TH2는 약 1초마다 읽는다.
4. 사용자가 확정한 해상도는 1초, 1,800개다. 각 창의 DC 제거 축별 RMS, 마지막 온도, 실제 표본 수, 품질 flags를 기록한다. ULP 코드/stack과 30개씩 두 RTC 버퍼를 7,168 B 예약 영역에 배치하고, HP가 완성 버퍼를 약 30초마다 전용 Flash 파티션에 저장한다. 원시 100 Hz 데이터는 저장하지 않는다.
5. 30분 후 버퍼를 동결하고 connectable GATT peripheral로 전환한다. nRF Connect 또는 PC Bleak가 직접 읽는다. BLE Classic SPP/COM 포트 방식이 아니다.
6. 연결 해제/재연결/재다운로드는 데이터를 삭제하지 않는다. 다음 측정은 별도 START 명령으로 시작한다. 수신되지 않은 버퍼를 자동 순환 덮어쓰기하지 않는다.
7. 완료된 Flash 데이터는 reset 후에도 CRC 검증을 거쳐 다시 제공한다. 수집 도중 전원 단절/비정상 reset은 부분 회차를 버리고 새 30분 수집을 시작한다. 시간은 측정 시작 상대시간이며 UTC가 아니다. 오류를 정상 0으로 숨기지 않는다.

## 영향 레이어

| 레이어 | 변경 |
|---|---|
| ESP32 | 신규 logger 빌드 경로, ULP, RTC, GATT, defaults |
| PC | `C:/Users/YechanLee/ingps_ble_logger`에 직접 수신/CSV 도구 |
| STM32 | 불필요: 신규 광고에 기존 manufacturer payload를 사용하지 않음 |
| 서버/DB/MQTT | 불필요: 데이터 경로에서 제외 |
| 기존 Android/웹 | 불필요: nRF Connect 사용 |

## 변경된 의미 / 대안

기존 설계의 즉시 경보 송신은 이번 요청의 '30분 무통신'에 따라 사용하지 않는다. 이 브랜치는 배치 측정용이다. 30분 이후 다운로드 대기 동안에는 수집을 멈추고 버퍼를 보존한다. 대기 중 BLE 전력은 수집 중 Deep Sleep 전력과 별도로 평가한다.

일반 RAM/PSRAM에 1초 데이터 전체를 저장하는 선택은 Deep Sleep 보존 조건과 맞지 않는다. 사용자가 '1초 단위 1,800개, 저장 방식 또는 절전 구조 추가 변경'을 선택하여 RTC 버퍼 + Flash 저장으로 결정했다. Flash erase는 새 회차 시작 때 전용 64 KiB 영역에만 수행한다. 데이터 36,000 B를 600 B 단위로 약 60회 저장한다. 수집 중 BLE/Wi-Fi API는 호출하지 않는다.

ULP 정지 또는 버퍼/CRC 이상은 나머지 레코드를 명시적 MISSING으로 기록한다. 정지 감지 후에도 30분 마감 전에는 BLE를 시작하지 않는다. ADC/FIFO의 실제 창 경계는 ULP 수거 간격만큼 양자화되므로 정밀한 샘플별 타임스탬프가 아니다.

## 검증 계획

- 로컬: ESP-IDF 별도 build/config 생성, ULP ELF의 코드+BSS+stack 여유, wire 구조 크기, CRC/부호/RMS/재수신 PC 테스트.
- 정적 계약: ULP 레코드 -> HP GATT -> Python decoder, UUID/크기/인덱스/CRC 일치.
- 실물 미검증: GPIO I2C 파형과 clock stretch/recovery, 30분 RF 무송신, 실제 100 Hz FIFO 손실, 온도 채널, nRF/Windows 연결과 재접속, UART/USB 제거 후 전류.
- commit/push/flash는 사용자가 수행한다.
