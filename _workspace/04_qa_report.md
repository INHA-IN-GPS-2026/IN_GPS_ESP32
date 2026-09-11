# QA: 30분 ULP/Flash/direct GATT logger

검증일: 2026-09-11. 기반 `81d0551`, 새 브랜치 `feature/ulp-30min-ble-logger`, 미커밋 작업 트리 기준.

> 후속 기본 `build/` 설정 오류와 수정은 [logger_default_build_fix.md](logger_default_build_fix.md)에 기록한다. 아래 최초 빌드 통과는 `build-logger/` 범위였으며, 루트 sdkconfig는 후속 작업에서 logger 필수값으로 동기화했다.

## 통과

| 항목 | 생산자/소비자 또는 실행 근거 |
|---|---|
| 1초×1,800개, 20 B wire 구조 | `main/logger/log_format.h:7`, `:17` ↔ `tools/pc_logger/protocol.py:15`, `:35`; C static assert, Python 테스트 |
| TH1/TH2 signed ×100, uint16 RMS, valid/error | `main/logger/ulp/main.c:57` ↔ `tools/pc_logger/protocol.py:35`; -0.50°C, RMS 0/32768/65535/invalid 테스트 |
| ULP 완성 버퍼 publication/HP 소비 | `main/logger/ulp/main.c:77` ↔ `main/logger/logger_main.c:35`; 미수신 버퍼 미덮어쓰기 호스트 시나리오 |
| HP → Flash 순서/CRC/되읽기 | `main/logger/logger_main.c:49`, `:55` ↔ `main/logger/store.c:56`; 양쪽 코드 대조. 실제 Flash 동작은 미검증 |
| INFO metadata 및 전체 CRC32 | `main/logger/store.h:5`, `main/logger/store.c:68` ↔ `tools/pc_logger/protocol.py:15`, `:55`; IDF CRC API의 complemented init/final 규칙 확인 |
| UUID / GATT READ/Indication/control | `main/logger/ble_logger.c:21`, `:34`, `:47` ↔ `tools/pc_logger/protocol.py:8`, `:76`, `:81`, `tools/pc_logger/download.py:37` |
| 전송 전 데이터 수집 무선 호출 경로 분리 | `main/logger/logger_main.c:97` 수집/wake 경로와 `main/logger/ble_logger.c:157` 대조. RF 실측과 구분 |
| 저장 이후 FINISH/NEXT 순서 | `tools/pc_logger/download.py:105` 이후 CRC/파일 저장/명령 ↔ `main/logger/ble_logger.c:77`; 모의 peer가 파일 존재와 전체 바이트 확인 뒤 명령 허용 |
| ESP-IDF 5.5.2 전체 빌드 | `tools/build_logger.py`, `build-logger-output.txt`: 최종 빌드 성공, app `0x82b60`=535,392 B; 1 MiB app 영역 여유 49% |
| ULP 메모리 | ELF .text 3,970 B, .rodata 256 B, .data 8 B, .bss 1,344 B; alignment 포함 끝 5,580 B. 예약 7,168 B → stack/headroom 1,588 B. 도구의 최소 768 B gate 통과 |
| ULP C 실제 로직 호스트 테스트 6개 | `tools/ulp_tests/test_ulp.c`, `build-logger/test_ulp.exe`: baseline/wrap/backpressure/gap/fault/full 통과. baseline=1,800개×100표본, backpressure=1,795개 보존+5개 drop 표시 |
| PC 프로토콜/모의 다운로드 테스트 9개 | `python -m unittest discover -s tools/pc_logger -p "test_*.py" -v`; CRC 손상, 부호/범위, 누락/중복, 35번째 read 연결 중단 후 34개 보존/1,766개 재개, 파일 저장 전 ACK 금지 |
| 실행용 PC 사본 | `C:/Users/YechanLee/ingps_ble_logger` 6개 파일 SHA256이 저장소 정본과 일치. 사본 위치에서 9개 테스트 통과 |
| 기존 사용자 코드 보존 | 시작 clean. `git diff --quiet -- main/app_main.c sdkconfig main/sensor main/ble main/watchdog`=0, `git diff --check` 통과 |

## 실패

최종 로컬 검증에서 남은 실패는 없다. 최초 빌드의 ULP include 경로, Kconfig 예약 범위, IDF RTC slow 영역 충돌, NimBLE 심볼 오류는 수정 후 재빌드했다. 실물 항목을 통과에 포함하지 않는다.

## 미검증 / 사용자 실행

- ESP32 플래시 및 실제 30분 연속 수집. 새 파티션 테이블을 포함하여 `LOGGER_README.md` 명령으로 사용자가 플래시한다.
- 실제 온도 주소/채널, 1 Hz 변환 유효성, GPIO8/9 I2C 파형, 150 ms halt + 처리시간, FIFO overrun/실제 RMS 오차.
- ULP/HP 메모리 fence 및 RTC retention의 실칩 동작, 정상 wake 중 ULP 지속, Flash 쓰기 중 센서 수거, 전원 이상 복구.
- 30분 RF 무송신, nRF Connect/Windows Bluetooth 실제 연결·Indication·READ·재연결·FINISH/NEXT.
- UART/USB 분리 후 수집/Flash wake/연결대기/다운로드 각각의 전류와 배터리·Supercap 전압창 비교.
- 기존 gateway/MQTT/DB/API/Android의 전체 계약 검증은 수행하지 않았다. 신규 경로에 참여하지 않으며 해당 파일을 변경하지 않았다.
- commit/push/배포/실물 플래시는 수행하지 않았다.

후속 5분 검증 설정: 현재 수집 기간은 300초다. 5분 ESP-IDF 빌드, ULP 5분/30분 호스트 테스트 12회, PC 테스트 11개가 통과했다. 상세 변경과 실물 확인/30분 복원 절차는 `logger_5min_validation.md`를 따른다.

후속 실물 데이터 확인: 다운로드 3회차(1800/300/300개)의 CRC는 통과했으나 모든 기록이 samples=0, flags=0x0120으로 센싱은 실패했다. 실물 정상 수집으로 판정할 수 없다. 실행 원인 구분을 위한 진단 빌드와 관측 근거는 `logger_missing_diagnosis.md`를 따른다.

후속 stage 8 trap: 사용자 진단 로그와 실제 ELF에서 초기 publication의 FENCE 위치가 일치했다. ULP FENCE를 compiler barrier로 교체하고 ELF 명령 검사를 빌드에 추가했다. 수정 전 ELF가 검사에서 실패하는 것을 확인했다. 수정 후 실물 센싱/절전 경계 검증은 사용자 재플래시 후 수행한다.
