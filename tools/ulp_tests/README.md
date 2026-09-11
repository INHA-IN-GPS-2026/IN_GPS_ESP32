# ULP C 로직 호스트 테스트

센서/시계/HP wake를 stub으로 대체하여 **실제 `main/logger/ulp/main.c`**를 실행한다. GPIO I2C 파형, sleep, cross-core memory fence의 하드웨어 동작은 검증하지 않는다. MSVC /W4의 좁은 정수 대입 경고는 wire 크기/주소/표본 상한을 검토했으며 ESP-IDF 빌드와 별개다.

Visual Studio 2022 x64 Native Tools 터미널에서 저장소 루트 기준:

```bat
cl /nologo /std:c11 /W4 /Itools\ulp_tests\stubs tools\ulp_tests\test_ulp.c /Febuild-logger\test_ulp.exe /Fobuild-logger\test_ulp.obj
build-logger\test_ulp.exe baseline
build-logger\test_ulp.exe wrap
build-logger\test_ulp.exe backpressure
build-logger\test_ulp.exe gap
build-logger\test_ulp.exe fault
build-logger\test_ulp.exe full
```

- baseline: 1,800개, 각 100표본, TH1=-0.50°C/TH2=25°C, RMS=(1000,500,0) mg.
- wrap: RTC 32-bit counter wrap을 넘어서 같은 결과.
- backpressure: HP 처리를 65초 지연하여 5개 드롭, 기존 60개 미덮어쓰기.
- gap: 장기 수거 지연을 MISSING/timing gap으로 표시.
- fault: 센서 NACK과 60초 retry 후 복구.
- full: FIFO full이면 정상 RMS valid로 표시하지 않음.

PC 테스트: `python -m unittest discover -s tools/pc_logger -p "test_*.py" -v`.

센싱 누락 진단: `/DCONFIG_INGPS_LOGGER_DIAGNOSTICS=1`로 빌드하면 정상 완료 시 진단 stage/elapsed도 검증한다. `all_fault` 시나리오는 모든 센서가 계속 응답하지 않아도 I2C/가속도 오류 기록이 생성되며 전체 MISSING으로 변하지 않는지 확인한다.

5분 경계도 검증하려면 cl 명령에 `/DCONFIG_INGPS_LOGGER_DURATION_S=300`을 추가한다. 생략 시 호스트 테스트는 1800초를 사용한다. 두 설정 모두 같은 6개 시나리오를 실행한다.
