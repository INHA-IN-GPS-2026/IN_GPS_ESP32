# 코드 검토

주 에이전트가 버퍼 소유권, 정수 폭, 초기화/복구 경로, BLE 확인/재전송, 파일 저장 순서를 검토했다.

## 검토 중 수정한 항목

- Kconfig에 따른 조건부 REQUIRES는 IDF의 초기 의존성 탐색에서 ULP 헤더를 누락시켰다. REQUIRES를 무조건 선언하고 SRCS만 분기했다 (`main/CMakeLists.txt`).
- ULP 예약 영역을 물리 최대값으로 설정하면 IDF 자체 RTC slow 데이터와 충돌했다. ULP 7,168 B, 나머지는 IDF에 남기고 ELF의 실제 코드+BSS 및 스택 여유 검사를 추가했다.
- FIFO 마지막 창도 수거하도록 창 마감 순서를 조정했다. 200표본 상한과 ±4096 count 범위 검증으로 uint64 통계 범위를 제한했다 (`main/logger/ulp/main.c`).
- Flash는 각 레코드마다 쓰는 대신 완성된 600 B 버퍼 단위로 쓰고 되읽어 검증한다 (`main/logger/store.c`).
- BLE indication의 송신 완료(status 0)와 ATT 확인(EDONE)을 구분했다. 재연결 시 stream/subscription/cursor 상태를 정리한다 (`main/logger/ble_logger.c`).
- FINISH/NEXT 후 host stop의 disconnect 이벤트에서 다시 광고하려는 경로를 `stopping` 상태로 차단했다. 이 경로를 열어 두면 shutdown 중 API 오류로 reset될 수 있다.
- PC는 기록 CRC/인덱스/전체 CRC 확인 후 파일을 flush/fsync하고 종료 명령을 보낸다. 중단 시 재개 파일을 보존한다 (`tools/pc_logger/download.py`).

## 잔여 한계

- 실제 FIFO/1초 창 타이밍과 RTC-GPIO/I2C 전기적 동작은 호스트 테스트로 검증되지 않는다.
- 개방된 시험용 GATT이며 pairing/authentication은 없다. 장치 접근을 제한해야 하는 제품 배포에는 별도 요구사항이 필요하다.
- 다운로드 대기 중 광고는 계속된다. FINISH/다음 회차 시작까지의 대기 전력을 실측해야 한다.
- 수집 중 비정상 reset의 부분 회차 복구/Flash wear-leveling/다중 회차 저장은 제공하지 않는다.
- legacy 광고용 5-layer watchdog/crash-loop escalation을 그대로 이식한 것은 아니다. 새 경로는 IDF WDT + RTC timer 기반 ULP liveness를 적용한다.

확인한 범위에서 남은 명확한 정확성/안정성 결함은 발견하지 못했다. 하드웨어/무선/전원 차원의 안정성 합격 판정은 아니다.
