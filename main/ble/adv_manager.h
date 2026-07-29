#pragma once
/* ble/adv_manager.h — 적응형 광고 정책 상태머신 (Analog 1.0.0, 2026-07-29)
 *
 * 목적: LS14500로 1년 런타임. adv interval이 최대 절감 레버(07-29 전력실험)이나
 * "모바일 실시간성" 요구와 상충 → 이벤트 기반 적응형으로 trade-off 제거:
 *   - MOVING(1s): 모션/ΔT/이벤트 발생 시 즉시 복귀 (<1 사이클)
 *   - STATIONARY(3s): quiet_ms(기본 5분) 동안 모션·온도변화 없을 때만
 *   - SAFE(10s): wdt_guard crash-loop escalation 한계 초과 시
 *   - FIXED(1s): 정책 knob으로 적응형 비활성(비교실험/보수 운용)
 *
 * 모든 파라미터는 NVS "advm" 네임스페이스에서 로드(없으면 기본값):
 *   policy(u8: 0=FIXED,1=ADAPTIVE=기본), mv_ms(u16,1000), st_ms(u16,3000),
 *   quiet_ms(u32,300000), mot_mg(u16,25), dt_x100(u16,50),
 *   name(u8,1=광고에 이름 포함=기본; 0이면 제거로 TX 시간 단축),
 *   tx_dbm(i8,0; -12..+9, 3dB 단위)
 */
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    ADVM_FIXED = 0,     /* 정책상 고정 1s */
    ADVM_MOVING,        /* 적응형: 활성(1s) */
    ADVM_STATIONARY,    /* 적응형: 정지(3s) */
    ADVM_SAFE,          /* crash-loop escalation: 10s 최소 광고 */
} advm_state_t;

/** NVS knob 로드 + 초기 상태 결정. nvs_flash_init 이후, BLE 시작 전에 호출. */
void adv_manager_init(void);

/** 매 광고 갱신 사이클마다 호출. 측정값으로 상태 전이를 결정한다.
 *  @param rms_max_mg 3축 RMS 중 최대값(mg)
 *  @param t1_x100/t2_x100 최신 온도(°C×100)
 *  @return 현재 상태 */
advm_state_t adv_manager_update(uint16_t rms_max_mg, int16_t t1_x100, int16_t t2_x100);

/** 현재 광고 인터벌 (BLE 단위: 0.625ms). ble_gap_adv_params.itvl_min/max용. */
uint16_t adv_manager_itvl_units(void);

/** 현재 페이로드 갱신 주기(ms). adv 인터벌과 동일 cadence. */
uint32_t adv_manager_cycle_ms(void);

/** 직전 update()에서 인터벌이 바뀌었는지 (adv stop/start 필요 여부). 읽으면 클리어. */
bool adv_manager_take_itvl_changed(void);

/** 광고 AD에 디바이스 이름 포함 여부 (knob advm/name). */
bool adv_manager_name_in_adv(void);

/** TX 파워 knob 적용 (esp_ble_tx_power_set). 컨트롤러 기동 후(on_sync) 호출. */
void adv_manager_apply_tx_power(void);

/** 보관 모드: 광고 중단 + 딥슬립(7µA). 웨이크 소스 없음 — EN/전원 재인가로만 복귀.
 *  현재 호출 경로 없음(비연결 광고라 수신 채널 부재). 추후 버튼/설정 채널용 API. */
void adv_manager_enter_storage(void) __attribute__((noreturn));
