// sensor/adxl345.h — Analog Devices ADXL345 3축 가속도계 (I2C) 드라이버.
//
// 근거: ADXL345 Datasheet Rev. E · 회로도 Docs/Schemetic/I2C_final_ver.pdf (U2)
//
// 회로도에서 확정한 결선:
//   · CS(7)              → 3.3V          ⇒ I2C 모드
//   · SDO/ALT_ADDRESS(12)→ GND           ⇒ 7-bit 주소 0x53
//   · VDD_I/O(1), VS(6)  → 3.3V (VS는 L1 47R 페라이트 경유)
//   · SCL(14)/SDA(13)    → SCL_OUT/SDA_OUT (단일 버스, GPIO9/8)
//   · INT1(8), INT2(9)   → ★도면상 X 마크 = 명시적 미접속
//   · RESERVED_3(3) 오픈, RESERVED_11(11) → GND  (둘 다 데이터시트 준수)
//
// ★INT 미접속의 결과: 인터럽트 기반 웨이크·워터마크 통지를 쓸 수 없다.
//   FIFO는 폴링으로만 드레인한다. 딥슬립 재설계에서 "진동 이벤트로 즉시 기상"을
//   하려면 보드 리비전이 필요하다
//   (Docs/INGPS_슈퍼캡_축전시간_분석_2026-08-11.md §5-4-3 옵션 3의 전제가 깨진다).
//
// === 운용 구성 — C-2 (Docs/INGPS_전력예산_디지털전환_ADXL345_2026-08-07.md §3-2) ===
//   100Hz low-power 상시 측정 + FIFO stream 32-deep + 광고 사이클마다 버스트 리드.
//   전류 50µA(데이터시트 typ, 2.5V 기준 — 🔶3.3V 실측 미검증).
//
//   ADXL335(350µA, ULP+SARADC+RTC_PERIPH 상시 ON)를 대체하며, 실익의 대부분은
//   센서 본체가 아니라 **ULP 서브시스템이 통째로 사라지는 것**이다(추정 300~900µA).
//
// === RMS 산출 창이 짧아진다 ===
//   구: ULP가 194.7Hz로 광고 주기(1~3s) 전체를 연속 적분 → 창 = 광고 주기.
//   신: FIFO 32-deep @100Hz → 창 = 320ms (사이클 길이와 무관하게 고정).
//   정상상태 진동에서는 등가지만, 3s 주기에서 320ms 창은 **듀티 11%**라
//   간헐적 충격은 놓칠 수 있다. mfg_data 13B 계약과 RMS 수식은 그대로다.
#pragma once

#include <stdbool.h>
#include <stdint.h>

/** FULL_RES 모드의 감도 — 레인지와 무관하게 3.9mg/LSB로 고정된다.
    counts/g = 1000/3.9 ≈ 256. accel_rms_to_mg()의 sens 인자로 쓴다. */
#define ADXL345_SENS_FULL_RES   256.0f

/**
 * @brief 버스에서 ADXL345를 찾아 붙이고 측정을 시작한다.
 *        ingps_i2c_bus_init() 성공 이후에 호출할 것.
 *
 * 순서: standby → BW_RATE/DATA_FORMAT/FIFO_CTL 기록 → measure.
 * 데이터시트 권고대로 설정은 전부 standby에서 한다.
 *
 * @return DEVID(0xE5)까지 확인되면 true. false여도 앱은 계속 부팅한다
 *         (RMS만 0으로 광고되고 온도는 정상).
 */
bool adxl345_init(void);

/**
 * @brief FIFO에 쌓인 샘플을 전부 드레인해 축별 RMS(mg)를 낸다.
 *
 * 분산 공식(var = E[x²] − E[x]²)이 DC를 제거하므로 **zero 캘리브레이션이
 * 필요 없다.** ADXL335 시절 부팅마다 돌던 1초 캘리브(≈0.3J)가 여기서 사라진다 —
 * 슈퍼캡 콜드스타트 마진에 직접 기여하는 항목이다.
 *
 * 오버플로 검산: FULL_RES ±16g에서 |v| ≤ 4096, FIFO 최대 32샘플이므로
 *   sum_sq ≤ 4096² × 32 = 5.4e8 (uint32 한계 4.29e9), |sum| ≤ 1.3e5 (int32 안전).
 *
 * @param rms_x/y/z 축별 RMS(mg). 실패하거나 샘플이 0이면 0.
 * @param out_n     이번에 실제로 읽은 샘플 수(진단용, NULL 허용).
 * @return 샘플을 1개 이상 읽었으면 true.
 */
bool adxl345_read_rms(uint16_t *rms_x, uint16_t *rms_y, uint16_t *rms_z,
                      uint32_t *out_n);

/** @return 마지막 접근이 성공한 상태인지. 부팅 시 미검출이면 계속 false. */
bool adxl345_present(void);

/**
 * @brief [진단 전용] 센서를 standby로 내려 표본 생성을 멈춘다.
 *        FIFO가 더 이상 차지 않으므로 다음 드레인부터 n=0 -> 드라이버 백오프 ->
 *        WDT_HB_ACCEL 정지 -> 헬스모니터 재부팅으로 이어진다.
 *        watchdog/wdt_test.c의 장애 주입 mode 1이 쓰며, 브링업에서
 *        "가속도계가 죽으면 워치독이 잡는가"를 실제로 확인할 때도 쓸 수 있다.
 *        재부팅하면 adxl345_init()이 다시 measure로 올린다.
 * @return I2C 쓰기가 성공했는지.
 */
bool adxl345_test_force_standby(void);
