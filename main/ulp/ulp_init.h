#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief ULP RISC-V ADC 초기화 및 실행.
 *        HW 핀맵: GPIO5/6/7(CH4/5/6) = ADXL X/Y/Z — 이 3채널이 전부다.
 *        온도는 AS6221(I2C, GPIO13/14)이 메인 CPU에서 담당하므로 ULP는 읽지 않는다.
 *        (GPIO8/CH7은 SDA_OUT이라 ADC 미사용.) 설정 후 ULP 바이너리 로드.
 *
 * ★2026-07-15: 딥슬립 burst 재설계용으로 do_zero_cal 파라미터를 추가했다가,
 * "측정값 1초마다 BLE 송신" 요구사항과 충돌(BLE_INIT만 1.1초+ 소요)해 롤백함.
 * 현재는 app_main에서 부팅 시 1회만 do_zero_cal=true로 호출하고, 이후 ULP는
 * 계속 돌아감(상시 광고 구조, [[ingps-deepsleep-redesign]] 참고). 파라미터는
 * 나중에 딥슬립을 다시 시도할 때 쓸 수 있게 남겨둠.
 *
 * @param do_zero_cal true면 cal_phase=1로 시작(zero_x/y/z=0, sum_raw_x/y/z 누적 시작 —
 *                    호출자가 잠시(app_main: 정상 1s / SAFE 50ms) 기다렸다가
 *                    sum_raw_x/y/z를 n으로 나눠 zero를 구해야 함).
 *                    ⚠ 이 단계를 건너뛰고 zero=0으로 두면 안 된다: dx≈raw≈2048이
 *                      되어 5.3s를 넘는 광고 창에서 sum_sq(uint32)가 오버플로하고
 *                      RMS가 3축 모두 0으로 나간다. app_main.c 캘리브 주석 참조.
 *                    false면 아래 zero_x/y/z를 즉시 적용하고 cal_phase=0(정상 측정)으로 시작.
 * @param zero_x/y/z  do_zero_cal=false일 때 사용할 기지의 ADXL zero.
 */
void start_ulp_adc_measurement(bool do_zero_cal, int16_t zero_x, int16_t zero_y, int16_t zero_z);
