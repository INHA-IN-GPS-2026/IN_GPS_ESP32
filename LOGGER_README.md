# IN-GPS: 30분 무통신 수집 → 직접 BLE 다운로드

브랜치: `feature/ulp-30min-ble-logger`, 기반 `81d0551`.

## 동작

1. 새 회차 시작 시 `logger` Flash 파티션만 지운다.
2. HP CPU는 Deep Sleep에 들어간다. ULP가 GPIO8(SDA)/9(SCL)에서 ADXL345 100 Hz FIFO를 계속 수거하고 AS6221 두 채널을 약 1초마다 읽는다.
3. **1초 기록 1,800개**를 만든다. 30개씩 두 RTC 버퍼를 번갈아 채우며, HP가 약 30초마다 깨어 완성 버퍼 600 B를 Flash에 저장한다. 이 구간에는 Bluetooth 초기화/광고/연결 및 Wi-Fi 통신이 없다.
4. 30분 후 수집을 끝내고 `IN_GPS_LOG_xxxxxx`라는 연결 가능한 BLE 장치로 나타난다. 모바일 nRF Connect 또는 PC가 직접 연결한다. 게이트웨이/서버/기존 Android 앱을 사용하지 않는다.
5. PC 기본 다운로드는 전체 CRC 검증과 파일 저장 후 FINISH를 보내 BLE를 종료하고 무기한 Deep Sleep으로 들어가게 한다. reset하면 완료 데이터를 다시 받을 수 있다. `--next-batch`는 저장 후 새 30분 수집을 시작한다.

수집 후에는 수신기가 연결할 때까지 검색용 광고가 필요하다. 기본은 다운로드 대기 중 계속 광고하며, 수신기가 없으면 전력이 계속 소모된다. nRF Connect에서는 아래 FINISH를 직접 보내 종료한다. 단순 연결 해제는 데이터를 보존하고 재연결 광고를 시작한다.

## 저장 내용과 한계

- 온도: TH1/TH2 물리 채널의 마지막 판독값, int16 ×100 °C. 기존 광고 payload의 채널 순서와 구분한다.
- 진동: 각 1초 창의 DC 제거 축별 RMS, uint16 mg. `sqrt(E[x²] - E[x]²)`, 256 counts/g. 내부 온도 SHF 추정값은 저장하지 않는다.
- 100 Hz 원시 3축 시계열은 저장하지 않는다. 1초 기록에 실제 FIFO 표본 수를 포함한다.
- 창 경계는 ULP 수거 시점 기준이다. 약 150 ms 수면 + 처리시간의 오차가 있으며, 샘플별 정확한 UTC 타임스탬프가 아니다.
- 온도 주소 기본은 TH1=`0x48`, TH2=`0x49`. 기존 NVS `as6221/addr1`, `addr2`를 읽는다. 이 경로는 주소 자동 탐색을 하지 않으므로 다른 주소의 모듈은 NVS 설정/기본값을 맞춘다. 같은 주소 두 개/범위 밖 설정은 기본 주소로 되돌린다.
- NACK/센서 오류/보수적인 FIFO-full 판정/250 ms 초과 수거 간격/표본 누락을 flags로 제공한다. RMS가 유효하지 않으면 CSV는 빈칸이다. CRC 통과는 측정 정확도 통과를 뜻하지 않는다.
- 완료 회차는 Flash에 보존된다. 수집 도중 전원 단절/비정상 reset은 부분 회차를 버리고 새 30분 수집을 시작한다. 새 회차를 시작하면 이전 완료 데이터도 지워진다.
- 이번 브랜치는 배치 측정용이다. 원 설계의 즉시 경보 송신 정책은 30분 무통신 요구에 따라 적용하지 않는다.

## 빌드와 사용자 플래시

로컬 설치된 ESP-IDF 5.5.2를 사용한다. 빌드 전용 도구는 플래시/commit/push를 수행하지 않는다.

```powershell
cd C:\esp\in_gps_project
& C:\Espressif\python_env\idf5.5_py3.13_env\Scripts\python.exe tools\build_logger.py
```

설정을 처음부터 재생성하려면 `--fresh-config`를 붙인다. 생성 대상은 `build-logger/sdkconfig`이고 정본은 `sdkconfig.defaults`다. 기존 추적 파일 `sdkconfig`와 `main/app_main.c`는 보존한다. 빌드 도구 경로가 다르면 `--idf`, `--tools`, `--python`을 지정한다.

플래시는 **사용자가 ESP-IDF 터미널에서** 수행한다. COM 번호를 실제 장치로 바꾼다.

```powershell
cd C:\esp\in_gps_project
idf.py -B build-logger -D SDKCONFIG=C:/esp/in_gps_project/build-logger/sdkconfig -p COM7 flash
```

새 파티션 테이블도 함께 플래시해야 한다. `erase-flash`는 필요하지 않다. NVS(`0x9000..0xefff`)와 factory app(`0x10000`, 1 MiB)은 기존 단일 app 배치를 유지하며, logger(`0x110000`, 64 KiB)를 추가한다. 실장된 장치가 다른 파티션 배치를 쓰는지는 플래시 전 확인한다.

기존 게이트웨이 광고 애플리케이션은 `CONFIG_INGPS_BATCH_LOGGER=n`일 때 별도 선택되며, 기존 소스는 유지되어 있다.

## PC 수신

실행용 사본: `C:\Users\YechanLee\ingps_ble_logger`. 버전 관리 정본: `tools/pc_logger/`.

```powershell
cd C:\Users\YechanLee\ingps_ble_logger
python -m pip install -r requirements.txt
python download.py --scan
python download.py --address AA:BB:CC:DD:EE:FF
```

한 대만 검색되면 `python download.py`로 충분하다. Windows 설정의 COM 포트/Classic SPP가 아니라 BLE GATT 연결이다. BLE 어댑터가 필요하고 모바일 연결은 먼저 끊는다. 여러 대가 있으면 주소를 지정한다.

- 기본: index 선택 + READ로 다운로드. 중단되면 같은 명령으로 이어 받는다.
- `--stream`: Indication으로 수신한 후 누락 인덱스를 READ로 보완한다.
- `--keep-awake`: 파일 저장 후 BLE 연결 대기를 유지한다.
- `--next-batch`: 파일 저장 후 이전 회차를 소비 처리하고 새 30분 수집을 시작한다.
- `--out 경로`: 저장 폴더 지정. 기본은 도구 폴더 아래 `downloads/`.

결과는 `.csv`(온도/RMS/품질), `.bin`(36,000 B 원본), `.json`(장치·회차·전체 CRC·다운로드 시각), `.partial.json`(재개용)이다. CSV 저장 전 모든 인덱스와 기록별 CRC16, 전체 CRC32를 검증한다. PC 파일이 저장되기 전에는 FINISH/NEXT를 보내지 않는다. 종료 명령의 응답 실패는 파일 저장 성공과 따로 보고한다.

## nRF Connect for Mobile

1. 수집 시작 후 30분을 기다리고 Scan에서 `IN_GPS_LOG_...`에 Connect한다.
2. 서비스 `8c5e0001-7a6b-4d21-9c35-494e47505331`을 연다.
3. INFO(`...0002...`) READ: 아래 20 B 메타데이터.
4. DATA(`...0003...`)의 **Indication**을 활성화한다(CCCD `02 00`).
5. CONTROL(`...0004...`)에 **Write Request / Byte array / HEX `01`**을 쓴다. 기록 1,800개가 20 B씩 전달된다. MTU 23에서도 동작한다. nRF 로그의 HEX 표시가 센서 레코드 원본이다.
6. 특정 기록만 읽으려면 CONTROL에 `02 <index LE 2 B>`를 쓰고 DATA READ. 예: 0번=`02 00 00`, 1,799번=`02 07 07`. 스트리밍 중에는 먼저 `05`로 중지한다.
7. 전송 완료 후 BLE 종료: INFO의 마지막 8바이트(session + crc32)를 그대로 복사하여 CONTROL에 `03 <8바이트>`를 쓴다. 새 회차 시작은 `04 <같은 8바이트>`다. 전송 도중 명령은 거절될 수 있으므로 마지막 Indication 확인 뒤 보낸다.

데이터를 눈으로 확인하는 nRF 경로도 제공하지만, **자동 CSV 생성/전체 CRC 검증/재개는 PC 도구가 담당**한다. 개방된 실험용 GATT 서비스이며 암호화 페어링/사용자 인증은 구현하지 않았다.

## Wire contract v1

모든 다중 바이트 숫자는 little-endian이다.

| INFO 오프셋 | 형식 | 값 |
|---|---|---|
| 0 | uint32 | magic `0x314c4749` (`IGL1`) |
| 4 / 5 | uint8 / uint8 | version 1 / record size 20 |
| 6 / 8 / 10 | uint16 ×3 | count 1800 / interval 1 / duration 1800 |
| 12 / 16 | uint32 ×2 | session ID / 전체 기록 CRC32 (zlib/ISO-HDLC) |

| DATA 오프셋 | 형식 | 의미 |
|---|---|---|
| 0 / 2 | uint16 ×2 | index 0..1799 / end_s 1..1800 |
| 4 / 6 | int16 ×2 | TH1/TH2 ×100 °C (`-32768` invalid) |
| 8 / 10 / 12 | uint16 ×3 | RMS X/Y/Z mg |
| 14 / 16 / 18 | uint16 ×3 | samples / flags / 앞 18 B의 CRC16 CCITT-FALSE |

flags: bit0 TH1 valid, bit1 TH2 valid, bit2 RMS valid, bit3 I2C error, bit4 FIFO full(손실 가능), bit5 timing gap, bit6 temp error, bit7 accel error, bit8 missing. 오류 비트와 필드별 valid를 함께 해석한다.

## 검증 범위

로컬 빌드/테스트의 최종 결과는 `_workspace/04_qa_report.md`에 기록한다. 실물 검증은 별도다:

1. 로직 분석기로 GPIO8/9의 open-drain/ACK/STOP/실제 SCL/FIFO 수거 간격/최대 occupancy 확인.
2. 30분간 RF 광고가 없고, 1,800개 기록이 만들어지며, 정상 조건에서 `samples`와 flags가 타당한지 확인.
3. 센서 분리, SDA/SCL stuck-low, BLE 연결 해제/재연결, PC 중단/재개, reset 후 완료 데이터 복원 시험.
4. nRF/Windows BLE 실제 다운로드 및 종료/다음 회차 시작 시험.
5. UART/USB를 빼고 같은 공급/충전/Vstart→Voff 조건에서 수집 중 평균전류, Flash wake 전하량, 전송/연결대기 전류를 각각 측정.

구현 참고: [Espressif ULP RISC-V](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/api-reference/system/ulp-risc-v.html), [ADXL345 Rev.G](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf), [Bleak client API](https://bleak.readthedocs.io/en/latest/api/client.html). 실제 API와 핀 제약은 로컬 ESP-IDF 5.5.2 소스도 대조했다.
