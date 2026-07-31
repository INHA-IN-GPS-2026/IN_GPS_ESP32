// watchdog/wdt_test.h — 워치독 장애 주입 테스트 스위치 (2026-07-18)
//
// 사용법: 아래 WDT_TEST_MODE 숫자만 바꾸고 idf.py build → flash.
//         한 번에 하나씩 테스트하고, 끝나면 반드시 0으로 되돌릴 것!
//         (0이 아니면 컴파일 때 #warning으로 상기시켜 줌)
//
//   0 = off (운영 빌드. 주입 코드가 빌드에서 완전히 빠짐 — no-op inline)
//   1 = ULP 정지         → L1 모니터가 ≤6s 내  "ULP stalled" 재부팅       (다음 부팅 reason=3)
//   2 = ADV 갱신 정지     → L1 모니터가 15s 후  "BLE adv update stalled"  (다음 부팅 reason=3)
//   3 = 태스크 무한 spin  → L2 Task WDT가 ≤8s 내 panic 재부팅             (다음 부팅 reason=6)
//   4 = 인터럽트 정지     → L3 INT WDT가 ≤300ms 내 panic 재부팅           (다음 부팅 reason=5)
//   5 = NULL 역참조 panic → L4 panic handler가 즉시 재부팅                (다음 부팅 reason=4)
//
// 발동 시점: 광고 루프 시작 후 WDT_TEST_TRIGGER_S초 뒤 1회 발동.
//   그 전까지는 정상 동작(1초 광고 갱신)을 관찰할 수 있다.
// 재부팅 후 확인 포인트:
//   - "reset reason=N" 이 위 표와 일치하는지
//   - "[fast resume]" + "reusing saved zero" 로그 (10s 재캘리브 생략)
//   - BLE 스캐너에서 광고 끊김이 ~2s 이내인지
//   - 장애 코드를 켠 채 두면 3회째부터 "crash-loop suspected" 경고가 뜨는지
//     (mode 1은 재부팅 후 ULP가 살아나므로 crash-loop은 mode 3/4/5로 확인)
//
// 참고: "부팅 구간 감시" 테스트(캘리브 중 TWDT)는 스위치로 못 만들고,
//       app_main.c 캘리브 루프의 wdt_guard_feed()를 수동 주석 처리해서 확인.
#pragma once

#include <stdbool.h>

#ifndef WDT_TEST_MODE
#define WDT_TEST_MODE      0
#endif

#ifndef WDT_TEST_TRIGGER_S
#define WDT_TEST_TRIGGER_S 30
#endif

#if WDT_TEST_MODE == 0

/* 운영 빌드: 완전 no-op (호출부 코드 그대로 두어도 오버헤드 없음) */
static inline void wdt_test_tick(void) { }
static inline bool wdt_test_suppress_heartbeat(void) { return false; }

#else

#warning "WDT_TEST_MODE != 0 — 장애 주입 테스트 빌드! 운영 배포 금지"

/** @brief adv 루프에서 1초마다 호출. TRIGGER_S 도달 시 선택된 장애를 발동. */
void wdt_test_tick(void);

/** @brief mode 2: 발동 이후 heartbeat kick을 억제해야 하는지. */
bool wdt_test_suppress_heartbeat(void);

#endif
