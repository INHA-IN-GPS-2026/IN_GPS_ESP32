# 기본 ESP-IDF 빌드의 ULP RAM 부족 수정

기준: `6109189`, `feature/ulp-30min-ble-logger`. 시작 시 사용자 변경은 루트 `sdkconfig`에 추가된 `CONFIG_INGPS_BATCH_LOGGER=y`뿐이었다.

## 원인

- 이전 검증은 `build-logger/sdkconfig`를 지정한 전용 빌드에 한정되었다.
- 사용자 명령의 기본 `build/`는 기존 루트 `sdkconfig`를 읽었다. 이 설정은 ULP 예약 4,096 B, 기존 single-app partition, 최대 BLE 연결 3개였다.
- 실제 생성 linker script `build/esp-idf/main/ulp_logger/ulp_riscv.ld`의 `ram LENGTH=4096`과 `.bss` 끝 `0x15cc`(5,580 B)가 충돌했다. 스택 여유도 별도로 필요하다.
- `sdkconfig.defaults`는 이미 존재하는 sdkconfig 값을 덮어쓰지 않으므로 defaults에 7,168 B를 적어 둔 것만으로는 기본 빌드가 수정되지 않았다.

## 수정

- 원본 백업: `build/sdkconfig.before_logger_sync_20260911_135037.bak`.
- 루트 sdkconfig에서 관련 선택만 제거하고 ESP-IDF가 기존 `sdkconfig.defaults`를 적용하여 다시 생성하게 했다. ULP=7168, custom=`partitions_logger.csv`, 최대 연결=1. 관련 파생/이전 이름 값도 Kconfig가 함께 동기화했다.
- `main/logger/check_config.cmake`: logger를 빌드할 때 활성 ULP/파티션/NimBLE 설정을 확인하고 구체적인 복구 명령을 보여 준다. `main/CMakeLists.txt`에서 로드한다.
- `tools/sync_logger_config.py`: 기존 sdkconfig 백업 후 필수 설정만 defaults에서 동기화. 반복 실행은 무변경이고 ESP32-S3 외 타깃은 거절한다. 다른 체크아웃의 과거 설정 복구용이다.
- `sdkconfig.defaults`: guard가 요구하는 GATT server를 명시적으로 활성화했다.
- `LOGGER_README.md`: 일반 `idf.py build`/IDE와 전용 `build-logger/`의 차이, 설정 동기화, 각 경로의 플래시 명령을 명시했다.

영향은 ESP 빌드 구성에 한정된다. ULP 수집 로직/GATT wire protocol/PC decoder/게이트웨이/서버/Android는 변경하지 않았다.

## 검증

- 통과: 설정 동기화 단위 테스트 3개(메모리·파티션·연결의 동시 복구/반복 실행 무변경/잘못된 타깃 거절).
- 통과: CMake guard 5개(정상 통과, ULP4096 거절, 파티션 누락 거절, 다중 연결 거절, legacy app일 때 검사 비활성).
- 통과: 백업과 현재 sdkconfig를 key/value로 비교하여 관련 7개 값 외에는 모두 보존됨. 기존 사용자 logger 활성화도 보존.
- 통과: 일반 `idf.py build`(루트 sdkconfig, 기본 build/) 전체 빌드 완료. app 0x851b0=545,200 B, 1 MiB 파티션 여유 48%.
- 통과: 새 linker script의 ram LENGTH=7168, ULP 코드·데이터 끝 5,580 B, stack/headroom 1,588 B. 생성 partition binary에서 logger 0x110000/64 KiB 확인.
- 실행 로그: `build/logger_default_build_output.txt`. 크기/해시: `_workspace/logger_default_build_result.json`.
- 미검증: 실제 플래시/30분 측정/BLE 수신/전류. 이번 작업에서 실물 작업은 수행하지 않았다.

## 코드 검토

메모리만 수정하면 Flash partition 누락으로 다음 단계에서 부팅이 실패하고, 연결 수가 3개면 단일 cursor/stream 상태를 공유하게 된다. 세 설정을 함께 교정하고 configure 시 검사한다. 설정 복구 도구는 검사 때문에 menuconfig를 시작할 수 없는 경우에도 독립적으로 실행할 수 있다.
