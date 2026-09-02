/* nvs_safe.h — 플래시 write를 BLE TX 전류 스파이크와 시간 분리하는 커밋 래퍼.
*/
#pragma once

#include "esp_err.h"
#include "nvs.h"

/**
 * @brief 광고를 잠시 멈춰 flash write와 TX 스파이크를 분리한 뒤 commit, 재개.
 *
 * 호출 컨텍스트: 태스크 전용(수 ms 블록). ISR 금지.
 * 광고가 아직 시작되기 전이면 정지/재개 없이 commit만 수행한다.
 * 재개 실패는 여기서 복구하지 않는다 — WDT_HB_ADV stale로 워치독(L계층)이
 * 자가 재부팅해 회수한다(Docs/Watchdog_설계.md).
 *
 * @return nvs_commit()의 결과.
 */
esp_err_t nvs_safe_commit(nvs_handle_t handle);
