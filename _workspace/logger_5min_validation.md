# 5분 검증 설정 (2026-09-11)

수용 기준: 1초 해상도와 수집 중 무통신을 유지하며 300초/300개 후 직접 GATT로 전환한다. 실물 검증 후 1800초/1800개로 되돌릴 수 있어야 한다. 현재 설정은 300초로 유지한다.

## 변경과 계약 확인

- ESP 변경: `main/Kconfig.projbuild:9`의 수집 기간 설정을 `main/logger/log_format.h:11`에서 HP/ULP 공통으로 사용한다. 영구 설정 `sdkconfig.defaults`는 300초이며 `tools/sync_logger_config.py`로 루트 sdkconfig를 백업 후 동기화했다. 30초 저장 주기, 센서 주기, GATT 전송 방식은 그대로다.
- PC 변경: 생산자 `main/logger/store.c:47`의 INFO count/interval/duration을 소비자 `tools/pc_logger/protocol.py:25`가 검증한다. 300..1800초, interval=1, count=duration을 허용한다. `tools/pc_logger/download.py:92`는 INFO 개수만큼 읽고 검증 후 파일을 저장한다. 통과: 300개/1800개 계약 모두 코드와 모의 테스트에서 일치.
- 사용자 실행 사본 `C:/Users/YechanLee/ingps_ble_logger`의 6개 파일은 사용자 수정 여부를 확인한 뒤 동기화했고 정본과 SHA256 일치를 확인했다. downloads는 변경하지 않았다.
- STM32/게이트웨이, 서버/DB/MQTT, 기존 Android/웹: 신규 직접 GATT 경로에 참여하지 않으므로 변경 불필요.
- `README.md`, `LOGGER_README.md`, PC/ULP 테스트 README를 동기화했다.

## 검증 결과

- 통과: 기본 `idf.py build`, 루트 sdkconfig/기본 build 경로, ESP-IDF 5.5.2. 생성 `build/config/sdkconfig.h:473`에서 기간 300 확인. 로그 `build/logger_5min_build_output.txt`.
- 통과: app 545200 B, SHA256 `ab7a23790917503849cefc1320a25d347476a09e6c4e8b58144f979de4e9af5c`. ULP 할당 끝 5580 B, 예약 7168 B, stack/headroom 1588 B.
- 통과: 실제 ULP C 코드 호스트 테스트 300초/1800초 각각 baseline, wrap, backpressure, gap, fault, full (12회). 정상 수집은 각각 300/1800개, backpressure는 각각 295/1795개 보존 + 5개 drop. MSVC의 기존 정수 축소 경고는 남아 있으며 기존 필드 범위 내 사용을 유지했다.
- 통과: PC 테스트 11개. 300개 다운로드의 6000 B BIN, CSV 300행, 저장 후 NEXT 요청; 기존 1800개 중단/재개 및 CRC 실패 시 ACK 금지 포함.
- 통과: 설정 동기화 테스트 3개 및 git diff --check.
- 실패: 최종 로컬 검사에 남은 실패 없음.
- 미검증: 실칩 5분/30분 타이밍, 센서 유효성, RF 무통신, Windows/nRF 수신 시간, 실제 전류. 플래시/commit/push는 수행하지 않았다. 새 구성으로 30분 전체 펌웨어 재빌드는 복원 시 수행하며 이번 전체 빌드는 5분 설정이다.

## 코드 검토와 사용자 검증

확인한 변경 범위에서 정확성/안정성 결함 없음. ULP와 HP는 같은 생성 sdkconfig를 사용하고 저장 한도/종료/INFO가 동일한 기간을 따른다. PC는 메타데이터의 개수와 기간 불일치 및 회차 경계 밖 레코드를 거절한다. 이전 사용자 변경은 유지했다.

사용자는 ESP-IDF 터미널에서 `idf.py -p COM7 flash monitor`를 실행한다(COM7은 실제 포트). 시작 `RF off: 300 x 1s records (300 s)`와 완료 `Batch ready: 300 records; starting direct GATT`를 확인하고 PC에서 다운로드/CRC/CSV flags를 확인한다.

5분 실물 검증 후 `sdkconfig.defaults`의 `CONFIG_INGPS_LOGGER_DURATION_S=1800`으로 변경하고 `python tools/sync_logger_config.py`, `idf.py build`, 사용자 플래시 순으로 복원한다. 전용 build-logger 경로는 `tools/build_logger.py --fresh-config`를 사용한다. PC 수신기는 그대로 사용한다.

현재 저장 정책은 Flash의 완료 회차 기간과 펌웨어 기간이 다르면 새 회차를 시작하며 이전 데이터를 지운다. 필요한 회차는 기간 변경/플래시 전에 다운로드한다. 같은 기간의 완료 회차가 남아 있다면 reset으로 새 수집이 시작되지 않으므로 `--next-batch`로 다운로드 후 다음 회차를 요청한다.
