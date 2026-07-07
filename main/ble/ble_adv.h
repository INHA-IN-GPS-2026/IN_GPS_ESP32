#pragma once

#include <stdint.h>

/* ============================================================
 * STM32 게이트웨이가 알아보는 보드 번호. main.c의 ESP32_NUM과 동일 의미.
 * 이 한 줄만 바꿔도 MAC 첫 옥텟과 device_id가 자동으로 ADDR_A 케이스로 따라감.
 * ============================================================ */
#define ESP32_NUM  1

#if   ESP32_NUM == 1
#define ESP_DEVICE_ID  0x01    /* main.c data_A[10] */
#elif ESP32_NUM == 2
    #define ESP_DEVICE_ID  0x02
#elif ESP32_NUM == 3
#define ESP_DEVICE_ID  0x07
#elif ESP32_NUM == 4
#define ESP_DEVICE_ID  0x0A
#else
#error "ESP32_NUM must be 1..4"
#endif

extern uint8_t g_own_addr_type;

/**
 * @brief BLE 광고 사이클 태스크 (FreeRTOS task entry).
 *        광고를 1회 시작하고 1초마다 mfg_data만 갱신.
 */
void adv_cycle_task(void *arg);
