// watchdog/wdt_guard.h — 통합 워치독/자가복구 관리 (2026-07-18)
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 앱 레벨 heartbeat 클라이언트.
   HW WDT(Task/INT WDT)가 못 보는 "태스크는 도는데 논리적으로 죽은" 상태를
   감지하기 위한 신선도 검사 슬롯. 성공 시점마다 wdt_guard_heartbeat()로 kick. */
typedef enum {
    WDT_HB_ADV = 0,   /* BLE 광고 데이터 갱신 성공 (정상 시 1초 주기) */
    WDT_HB_MAX
} wdt_hb_id_t;

/**
 * @brief 워치독 계층 초기화. app_main 최상단에서 1회 호출.
 *        - 리셋 사유 로그 + 연속 비정상 리셋 카운터(RTC_NOINIT) 갱신
 *        - Task WDT 재설정(8s, trigger_panic=true, idle 양코어 감시)
 *        - 호출한 태스크(app_main)를 TWDT에 등록(부팅 구간 감시)
 *        - 헬스모니터 태스크 기동(앱 체크는 wdt_guard_boot_done() 전까지 보류)
 */
void wdt_guard_init(void);

/** @brief 현재 태스크의 TWDT 카운터 리셋. 부팅 단계 사이/주기 루프에서 호출. */
void wdt_guard_feed(void);

/** @brief 현재 태스크를 TWDT 감시 대상으로 등록(장수명 태스크 시작부에서 호출). */
void wdt_guard_task_subscribe(void);

/** @brief 앱 레벨 heartbeat 갱신. "실제로 일이 성공한" 시점에만 호출할 것. */
void wdt_guard_heartbeat(wdt_hb_id_t id);

/**
 * @brief 부팅(캘리브 포함) 완료 선언. app_main 마지막 줄에서 호출.
 *        - app_main 태스크를 TWDT에서 해제(main task는 곧 자기삭제되므로)
 *        - 헬스모니터의 앱 체크(ULP 정지·heartbeat 신선도) 활성화
 *        - WDT_HB_ADV 시계를 arm — 이후 15s 내 광고 갱신이 한 번도 성공하지
 *          못하면(예: NimBLE sync 실패) 자가 재부팅
 */
void wdt_guard_boot_done(void);

/** @brief 사유 로그 + UART 플러시 후 esp_restart(). 복구 불가 상황에서 호출. */
void wdt_guard_reboot(const char *reason) __attribute__((noreturn));

/**
 * @brief 비정상 리셋(WDT/panic/자가복구 SW 리셋) 후 재부팅이면, 직전 가동에서
 *        저장해 둔 ADXL zero를 돌려주고 true. → 10초 재캘리브 생략(fast resume)
 *        으로 광고 재개까지 다운타임 ~1.5s로 단축.
 *        전원 인가/브라운아웃/데이터 손상 시 false → 통상 캘리브 수행.
 */
bool wdt_guard_fast_resume(int16_t *zx, int16_t *zy, int16_t *zz);

/** @brief 캘리브 완료 직후 zero를 RTC_NOINIT에 보관(fast resume용). */
void wdt_guard_save_zero(int16_t zx, int16_t zy, int16_t zz);
