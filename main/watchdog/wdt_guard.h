// watchdog/wdt_guard.h — 통합 워치독/자가복구 관리 (2026-07-18)
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ★★ 진단 전용: 워치독 계층 전체 비활성 (벤치 전력 측정용) ★★
 *
 * 1로 두면 이 파일의 공개 API가 전부 no-op이 되고, 추가로:
 *   - L1 헬스모니터 태스크(wdt_monitor_task)를 **생성하지 않는다**.
 *     → 2초 주기 wake가 사라진다. INGPS_BLE_DISABLED=1 빌드에서는 이 태스크가
 *       칩을 깨우는 **유일한** 주기적 소스이므로, 끄면 light sleep 구간이
 *       사실상 무한이 되어 "센서+슬립 순수 floor"가 측정된다.
 *   - IDF가 부팅 시 자동 기동한 Task WDT(L2)를 esp_task_wdt_deinit()으로
 *     **해제한다**. CONFIG_ESP_TASK_WDT_INIT=y라 그냥 두면 기본 5초 TWDT가
 *     양 코어 idle을 계속 감시하므로, 재설정을 건너뛰는 것만으로는 부족하다.
 *   - crash-loop escalation(백오프 딥슬립 / SAFE 모드)도 함께 비활성된다.
 *     즉 재부팅이 나도 스스로 딥슬립으로 도망가지 않으므로, "재부팅 루프인가"를
 *     전류 파형과 부팅 로그로 그대로 관찰할 수 있다.
 *
 * ★유지되는 것 (진단에 필요하므로 일부러 남긴다):
 *   - esp_reset_reason() 판독과 WDT_GUARD 로그 1줄. 재부팅 루프 여부를
 *     확인하려면 이게 반드시 남아 있어야 한다.
 *   - wdt_guard_reboot(): 워치독이 아니라 명시적 에러 핸들러다(광고 인코딩
 *     실패 등). 끄면 실패를 조용히 삼키게 되므로 그대로 둔다.
 *
 * ★이 스위치가 끄지 **못하는** 것 (sdkconfig 소관, 코드로 못 만짐):
 *   - L3 Interrupt WDT  : CONFIG_ESP_INT_WDT (=y, 300ms)
 *   - L5 부트로더 RTC WDT: CONFIG_BOOTLOADER_WDT_ENABLE
 *   L2까지 껐는데도 재부팅이 계속되면 원인은 이 둘 아니면 브라운아웃/슈퍼바이저다
 *   — 그 경우 부팅 로그의 reset reason으로 갈린다(INT_WDT vs BROWNOUT vs POWERON).
 *
 * ⚠ 운영 배포 금지. 필드 노드가 멈춰도 아무도 살려내지 않는다.
 *   측정이 끝나면 반드시 0으로 되돌릴 것.
 */
#ifndef INGPS_WDT_DISABLED
#define INGPS_WDT_DISABLED  0
#endif

#if INGPS_WDT_DISABLED
#warning "INGPS_WDT_DISABLED=1 — 워치독 전 계층 해제된 진단 빌드! 운영 배포 금지"
#endif

/* 앱 레벨 heartbeat 클라이언트.
   HW WDT(Task/INT WDT)가 못 보는 "태스크는 도는데 논리적으로 죽은" 상태를
   감지하기 위한 신선도 검사 슬롯. 성공 시점마다 wdt_guard_heartbeat()로 kick. */
typedef enum {
    WDT_HB_ADV   = 0, /* BLE 광고 데이터 갱신 성공 (정상 시 1초 주기) */
    WDT_HB_ACCEL,     /* ADXL345 FIFO 드레인에서 표본을 1개 이상 얻음
                         (구 ULP total_samples 정지 감시를 대체) */
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
 * @brief 부팅 완료 선언. app_main 마지막 줄에서 호출.
 *        - app_main 태스크를 TWDT에서 해제(main task는 곧 자기삭제되므로)
 *        - 헬스모니터의 앱 체크(heartbeat 신선도) 활성화
 *        - WDT_HB_ADV 시계를 arm — 이후 15s 내 광고 갱신이 한 번도 성공하지
 *          못하면(예: NimBLE sync 실패) 자가 재부팅
 */
void wdt_guard_boot_done(void);

/** @brief 사유 로그 + UART 플러시 후 esp_restart(). 복구 불가 상황에서 호출. */
void wdt_guard_reboot(const char *reason) __attribute__((noreturn));

/** @return crash-loop escalation이 SAFE 모드를 선언했는지.
 *         true면 앱은 최소 기능(10s 광고)으로 부팅해야 한다.
 *         (rev 4.0/ADXL345부터 "캘리브 단축" 항목은 사라졌다 — 분산 공식이
 *          DC를 소거하므로 zero 캘리브레이션 자체가 없다.) */
bool wdt_guard_safe_mode(void);

/** @brief heartbeat 허용 한도(stale_ms) 조정. 적응형 광고 cadence(3s/10s)에서
 *         오탐 재부팅 방지용. 슬롯별 기본값보다 짧게는 못 줄인다
 *         (ADV 15s / ACCEL 75s). */
void wdt_guard_set_hb_stale(wdt_hb_id_t id, uint32_t stale_ms);
