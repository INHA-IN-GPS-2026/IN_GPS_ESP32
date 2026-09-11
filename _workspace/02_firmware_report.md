# 구현 보고: 30분 직접 BLE logger

## 구현

- 새 브랜치에 `main/logger/`를 추가하고 Kconfig로 기존 광고 app과 빌드 경로를 분리했다.
- ULP: GPIO8/9 software I2C, ADXL345 100 Hz stream FIFO, AS6221 1 Hz, 약 150 ms halt 주기. 정수/uint64 통계에서 1초 RMS를 계산한다.
- RTC: 30개×2 버퍼, 32-bit ready publication + memory fence. HP는 완성 버퍼만 읽고 Flash 쓰기/되읽기 검증 뒤 해제한다. ULP는 HP를 기다리며 spin하거나 기존 데이터를 덮지 않는다.
- Flash: 새 `logger` 파티션 64 KiB, 기록 36,000 B. 600 B 묶음 저장, 완료 metadata/commit marker는 마지막에 기록한다. 기존 NVS/센서 설정에는 쓰지 않는다.
- HP: 정상 Deep Sleep 복귀에서 센서/ULP 바이너리를 다시 초기화하지 않는다. ULP wake 및 독립 30초 timer, 진행 정지와 기록 CRC/순서 확인. 실패하면 마감까지 무통신을 유지하고 미수집 항목을 MISSING으로 채운다.
- BLE: 연결 가능한 custom GATT, INFO/DATA/CONTROL, default MTU23에 들어가는 20 B 레코드, 단일 연결. Indication ACK 후 다음 기록 전송. index READ로 부분 수신 복구.
- FINISH: BLE 종료 + 무기한 Deep Sleep, Flash 보존. NEXT: 소비 marker 후 재부팅, 다음 30분 측정. 단순 disconnect는 재다운로드 대기.

## 기존 시스템과 구분

기존 mfg_data를 송신하지 않아 STM32/MQTT/서버/Android DTO의 계약 변경은 없다. 기존 앱의 15초 offline 판정은 이 독립 실험 경로에 적용하지 않는다. 기존 `app_main.c`, sensor/ble/watchdog 소스 및 추적 `sdkconfig`는 변경하지 않았다.

이 경로는 IDF Task/Interrupt watchdog과 ULP 진행 감시를 사용한다. 기존 광고 전용 `wdt_guard` heartbeat 정책은 사용하지 않는다. 일반 장애 reset 후 부분 회차는 폐기하고 새 회차를 시작한다. 기존 경보 상태 머신/SHF/게이트웨이 통합을 구현한 것으로 취급하지 않는다.

## 실물 검증 항목

GPIO 타이밍/풀업·온도 주소·FIFO 표본 보존·RTC clock 정확도·정상 wake/Flash와 동시 ULP 동작·Flash/전송 전류·nRF/PC 무선 수신은 미검증. `LOGGER_README.md`의 사용자 검증 절차를 따른다.
