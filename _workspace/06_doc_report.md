# 문서 동기화

- `LOGGER_README.md`: 최종 1초×1,800개/RTC+Flash/직접 GATT 동작, PC와 nRF 사용, 파티션/플래시/검증 절차.
- `README.md`: 현재 브랜치 설명과 기존 광고 경로의 문서 범위를 첫머리에 구분.
- `main/CMakeLists.txt`: 낡은 'ULP는 I2C 접근 불가' 설명을 RTC-GPIO software I2C와 전용 hardware I2C 핀 제약으로 정정.
- `sdkconfig.defaults`: 새 logger의 영구 설정. 추적 `sdkconfig`와 새 `build-logger/sdkconfig`의 용도를 구분.
- `_workspace/01_analyst_spec.md`: 사용자 해상도 답변(1초) 및 수집 중 무통신 확인을 반영.
- PC 도구 정본은 저장소 `tools/pc_logger/`, 실행 사본은 사용자 홈의 `ingps_ble_logger/`로 제공.

원래 2026-09-06 아키텍처 문서는 원본으로 보존했다. 기존 README 본문의 13 B/SHF 등 legacy 설명 드리프트는 이 작업의 직접 GATT 계약과 구분하며 수정하지 않았다. STM32/서버/Android 문서는 변경하지 않았다.
