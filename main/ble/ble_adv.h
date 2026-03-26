#pragma once

#include <stdint.h>

extern uint8_t g_own_addr_type;

/**
 * @brief BLE 광고 사이클 태스크 (FreeRTOS task entry)
 *        0.5s 광고 → 5s 대기 반복
 */
void adv_cycle_task(void *arg);
