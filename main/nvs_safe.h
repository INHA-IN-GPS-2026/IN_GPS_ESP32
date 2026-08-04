/* nvs_safe.h — 플래시 write를 BLE TX 전류 스파이크와 시간 분리하는 커밋 래퍼.
 *
 * 배경(2026-07-31): BOD를 IDF 기본 LVL7(≈2.44V)로 원복했다. 이 임계는 플래시
 * write 안전선(~2.7V)보다 낮으므로, TX 스파이크로 레일이 주저앉은 순간에
 * flash program/erase가 겹치면 BOD 리셋 이전에 플래시가 스펙 밖 전압에서
 * 쓰기를 진행할 수 있다 → NVS 조용한 손상 위험(adc_cal 계수, advm knob 등).
 *
 * 규칙: 런타임에서 nvs_commit()을 직접 부르지 말 것. 반드시 이 래퍼 경유.
 *  - 현재(2026-07-31) 펌웨어에 런타임 NVS 쓰기는 없다(knob은 파티션 툴로
 *    주입하고 adv_manager는 READONLY로만 연다). 이 래퍼는 앞으로 생길 쓰기
 *    경로(런타임 knob 변경, 통계 저장 등)용 기반이다.
 *  - IDF 내부 쓰기(phy cal 데이터 등)는 BT 기동 초기에 일어나 광고와 겹치지
 *    않으므로 별도 처리 불필요.
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
