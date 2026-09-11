# IN-GPS PC 직접 BLE 수신 도구

실행 폴더: `C:\Users\YechanLee\ingps_ble_logger`

```powershell
cd C:\Users\YechanLee\ingps_ble_logger
python -m pip install -r requirements.txt
python download.py --scan
python download.py --address AA:BB:CC:DD:EE:FF
```

장치 한 대만 보이면 `python download.py`로 충분하다. 장치는 **설정한 수집 시간 완료 뒤(현재 30분, 단축 검증 5분)** 검색된다. BLE 어댑터가 필요하며 모바일 nRF Connect가 연결돼 있으면 먼저 연결을 끊는다.

- 기본: 1초 기록 1,800개 수신 → CRC 검증 → `downloads/`에 CSV/BIN/JSON 저장 → BLE 종료 요청.
- 중간에 끊기면 같은 명령을 다시 실행한다. `.partial.json`으로 같은 회차를 이어 받는다.
- `--stream`: Indication 수신 후 누락 인덱스를 READ로 보완.
- `--keep-awake`: 저장 후 BLE 대기를 유지.
- `--next-batch`: 저장 후 설정된 시간의 다음 수집을 시작. 이전 장치 회차는 삭제되므로 PC에 먼저 보존한다.
- `--out 경로`: 출력 폴더 지정.

CSV의 온도/RMS 빈칸은 invalid를 뜻한다. `flags`는 0x0001/2/4=TH1/TH2/RMS valid, 0x0008=I2C error, 0x0010=FIFO full, 0x0020=timing gap, 0x0040=temp error, 0x0080=accel error, 0x0100=missing. CRC 정상과 센서 데이터 정상은 구분한다. `end_s`는 시작 후 상대 초이고 `.json`의 UTC는 다운로드 시각이다.

프로토콜: `8c5e0001-7a6b-4d21-9c35-494e47505331` 서비스, 끝의 첫 블록 `0002` INFO(read), `0003` DATA(read/indicate), `0004` CONTROL(write).

nRF Connect: DATA Indication을 켜고 CONTROL에 HEX `01` Write Request. 완료 후 INFO 마지막 8 B 앞에 `03`을 붙여 CONTROL에 쓰면 BLE가 종료된다. `04`는 다음 회차 시작이다. 단순 disconnect는 재연결 광고를 재개한다.

펌웨어·파티션·정확한 wire 레이아웃·실물 확인 절차는 `C:\esp\in_gps_project\LOGGER_README.md`를 참조한다. 정본 소스는 해당 저장소 `tools/pc_logger/`이며 이 폴더는 실행용 사본이다.

자체 테스트(무선 연결 없음): `python -m unittest discover -p "test_*.py" -v`.

라이브 Windows/nRF BLE 수신은 실제 센서 플래시 후 확인해야 한다. 프로토콜/모의 연결 테스트가 실물 RF 검증을 대신하지 않는다.

수신기는 INFO의 count/duration_s를 확인하여 5분 300개와 30분 1,800개를 모두 지원한다. BIN은 각각 6,000 B / 36,000 B이며 CSV 데이터 행은 각각 300 / 1,800개다.
