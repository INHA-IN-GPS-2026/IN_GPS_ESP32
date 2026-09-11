# 문서 동기화

- `LOGGER_README.md`: 최종 1초×1,800개/RTC+Flash/직접 GATT 동작, PC와 nRF 사용, 파티션/플래시/검증 절차.
- `README.md`: 현재 브랜치 설명과 기존 광고 경로의 문서 범위를 첫머리에 구분.
- `main/CMakeLists.txt`: 낡은 'ULP는 I2C 접근 불가' 설명을 RTC-GPIO software I2C와 전용 hardware I2C 핀 제약으로 정정.
- `sdkconfig.defaults`: 새 logger의 영구 설정. 추적 `sdkconfig`와 새 `build-logger/sdkconfig`의 용도를 구분.
- `_workspace/01_analyst_spec.md`: 사용자 해상도 답변(1초) 및 수집 중 무통신 확인을 반영.
- PC 도구 정본은 저장소 `tools/pc_logger/`, 실행 사본은 사용자 홈의 `ingps_ble_logger/`로 제공.

원래 2026-09-06 아키텍처 문서는 원본으로 보존했다. 기존 README 본문의 13 B/SHF 등 legacy 설명 드리프트는 이 작업의 직접 GATT 계약과 구분하며 수정하지 않았다. STM32/서버/Android 문서는 변경하지 않았다.

후속 5분 검증 설정: README/LOGGER_README/PC 및 ULP 테스트 README에 현재 300초 설정과 1800초 복원을 반영했다. PC 실행 사본도 동기화했다. 상세 검증은 `logger_5min_validation.md`에 있다.

센싱 누락 진단: `LOGGER_README.md`와 `logger_missing_diagnosis.md`에 전체 MISSING 관측, 첫 30초 진단 로그, 저장된 회차가 있을 때 NEXT 사용, 진단 설정 해제 방법을 기록했다. 후속 stage 8/trap=1 실물 로그와 ELF의 FENCE 위치가 일치하여 원인 및 수정/재발 방지 검사를 추가 기록했다. 수정 후 실물 정상 수집은 아직 미검증이다.

후속 ESP-IDF 기본 빌드 오류 대응: 루트 sdkconfig도 logger 필수값에 동기화했고, `LOGGER_README.md`에 일반 빌드와 전용 빌드의 구분/복구 절차를 추가했다. 상세 근거는 `logger_default_build_fix.md`에 있다.
