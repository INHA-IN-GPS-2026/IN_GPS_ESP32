# 전 기록 센싱 누락 진단 (2026-09-11)

## 관측으로 확인한 내용

사용자 보고: 같은 보드의 이전 일반 펌웨어에서는 온도/진동이 모두 정상 동작했다.

`C:/Users/YechanLee/ingps_ble_logger/downloads`의 아래 원본 BIN을 실제 protocol.Batch로 다시 읽어 개별 CRC16 및 전체 CRC32를 검증하고 CSV와 대조했다.

| 파일 stem | 개수 | flags / samples | 판정 |
|---|---:|---|---|
| ingps_94A990CEC1E6_71f20988 | 1800 | 모두 0x0120 / 0 | 유효 센싱 없음 |
| ingps_94A990CEC1E6_bb77482b | 300 | 모두 0x0120 / 0 | 유효 센싱 없음 |
| ingps_A4CB8FCEA7E2_aeb972f7 | 300 | 모두 0x0120 / 0 | 유효 센싱 없음 |

- 통과: 세 파일의 길이/순번/시간 필드/CRC. 전송/CSV 변환 중 센서값이 사라진 정황 없음.
- 실패: 온도 TH1/TH2 모두 -32768 invalid, RMS valid 없음, 표본 0. 0x0120은 LOG_MISSING(0x100) + LOG_TIMING_GAP(0x20).
- 이 레코드는 HP `missing_record()` 또는 ULP `append(true)`가 만드는 결측 표현이다. 파일만으로 어느 경로인지 구분할 수 없다. 일반적인 I2C 읽기 실패라면 ULP 루프는 시간 진행에 따라 센서 오류 플래그를 가진 기록을 만들므로, 실행/시간/은행 전달 경로를 우선 조사한다.
- 미확정: ULP 미시작, 초기화/실행 중 정지, RTC 시간 정체, 은행 검증 실패 중 실제 원인. 모든 레코드 CRC가 맞는 것은 센싱 정상의 증거가 아니다.

## 진단 변경 (동작 수정 아님)

- 수집 기간은 300초로 유지. 센서 설정, GPIO 구동, 수집/저장 로직, watchdog, GATT 계약은 바꾸지 않았다.
- `CONFIG_INGPS_LOGGER_DIAGNOSTICS`로 ULP 별도 진단 심볼과 HP UART 출력을 켠다. main 진입 횟수(entries), stage, tick/elapsed, 센서 상태, initialized/progress/produced/stored, ready/count, stopped/stalled, ULP timer/trap을 기록한다.
- 관측용 공유 필드는 비동기 스냅샷이다. stage 하나만으로 원인을 단정하지 않고 두 시점의 변화와 다른 필드를 함께 본다.
- 게이트웨이/서버/Android/웹 및 PC wire 포맷은 변경 불필요.

| stage | 마지막 진입 단계 |
|---:|---|
| 0 | 아직 main 진입 기록 없음 |
| 1 | main 진입 / stop, done, magic, clock guard |
| 2 | GPIO 초기화 |
| 3 | 초기 I2C 버스 복구 |
| 4 | ADXL345 설정 |
| 5 / 6 | 온도 채널 1 / 2 설정 |
| 7 | 초기 RTC 시간 읽기 |
| 8 | 초기화 완료 publication |
| 9 | 반복 실행 RTC 시간/elapsed 계산 |
| 10 | 가속도 재설정/읽기 |
| 11 | 온도 읽기 |
| 12 | 1초 창 종료/레코드 publication |
| 13 | 전체 수집 종료 |
| 14 | progress publication |
| 15 | 정상 main 반환 직전 |

## stage=8 trap 원인과 수정

후속 실물 로그: `entries=1 stage=8 init=0 progress=0 produced=0 trap=1`, `ticks_per_s=138884`, `magic=314c4749`. ULP는 main에 진입하고 초기 RTC 읽기까지 진행했지만 초기화 완료 표시 전에 trap으로 멈췄다. `accel_ok/temp_valid` 진단값은 stage 12에서 복사하므로 이 로그의 0만으로 센서 설정 실패라고 판정하지 않는다.

진단 펌웨어의 ULP ELF를 역어셈블하면 stage 8 저장(0x594) 다음 명령이 `0x598: 0330000f fence rw,rw`이고, 그 다음이 initialized=1 저장이다. 같은 명령이 publication 지점 4곳에 있다. 이 실행 위치와 trap/초기화 미완료 상태가 일치하여 `publish()`에 넣은 FENCE를 원인으로 판단했다. 이전 호스트 테스트는 이 inline assembly를 제외했으므로 검출하지 못했다.

수정: `main/logger/ulp/main.c`의 하드웨어 FENCE를 기계어를 생성하지 않는 compiler memory barrier로 교체했다. 공유 RTC 구조체의 volatile 접근, ready를 마지막에 게시하는 단일 생산자/소비자 소유권, HP측 Xtensa memw는 유지한다. 로컬 ESP-IDF 5.5.2 `components/ulp/ulp_riscv/ulp_core/ulp_riscv_lock.c`도 ULP측 volatile RTC 접근으로 동기화하며 FENCE를 사용하지 않는다. 이번 판단을 다른 RISC-V CPU 전체에 일반화하지 않는다.

재발 방지: `tools/check_ulp_instructions.py`는 실제 ELF 실행 섹션에서 16/32비트 명령 경계를 따라 FENCE 계열 opcode를 검사한다. `main/CMakeLists.txt`의 main 라이브러리 빌드 후 검사로 기본 ESP-IDF/전용 빌드에 함께 적용된다. 수정 전 ELF `build/ulp_logger_before_fence_fix.elf`가 0x598에서 실패하는 것을 확인했다. 데이터 섹션의 같은 바이트 패턴은 검사 대상이 아니다.

수정 후 검증: ESP-IDF 5.5.2 기본 빌드 통과, 실제 ULP ELF 1446개 명령에 FENCE 없음. 수정 전 ELF는 동일 검사에서 0x598 FENCE로 실패했다. 앱 크기 0x85690 B이며 빌드 로그는 `build/logger_fence_fix_build_output.txt`다. 이번 회귀 검증은 호스트 시뮬레이션 대신 문제가 발생한 실제 RISC-V 기계어를 대상으로 했다.

실물 재확인 기준: 새 회차 시작 후 init=1, trap=0이 되고, 약 30초 후 entries/progress/elapsed가 증가하며 produced/stored가 약 30으로 증가해야 한다. 이후 실제 센서 flags/samples/CSV 유효성을 확인한다. 수정 후 실물 센싱 성공은 아직 미검증이다.

## 사용자 확인과 원복

로컬 검증: 진단 활성 300초 ESP-IDF 5.5.2 빌드 통과 (`build/logger_diagnostic_build_output.txt`, app 0x856a0 B). ULP ELF 할당 끝 5776 B / 예약 7168 B / headroom 1392 B로 768 B 최소 여유를 통과했다. 진단 활성 ULP 호스트 테스트 7개(baseline/wrap/backpressure/gap/fault/full/all_fault), 설정 동기화 테스트 3개와 diff 공백 검사가 통과했다. all_fault에서도 300개 오류 기록을 만들고 MISSING으로 변하지 않음을 확인했다. 호스트 테스트는 실제 GPIO/시간/절전/메모리 fence를 모사하지 않으므로 실물 원인 해결은 미검증이다.

새 진단 빌드를 사용자가 `idf.py -p COM7 flash monitor`로 플래시한다(실제 포트로 변경). 같은 기간의 완료 회차가 있으면 즉시 BLE가 켜질 수 있다. `python download.py --stream --next-batch`로 기존 파일을 PC에 보존하고 새 회차를 시작한다. 새 수집 시작과 약 30초 후 `LOGGER_DIAG` 두 묶음이 필요하다. 원인 구분을 위해 5분 전체 대기는 불필요하다.

진단 해제는 `sdkconfig.defaults`의 `CONFIG_INGPS_LOGGER_DIAGNOSTICS=n`, `python tools/sync_logger_config.py`, `idf.py build`, 사용자 플래시 순서다. 진단 자체는 ULP 연산과 UART 출력을 추가하므로 전류 측정 전에 해제한다. 원본 다운로드는 수정하지 않았다.
