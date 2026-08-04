/* nvs_safe.c — 설계 배경·사용 규칙은 nvs_safe.h 주석 참조. */
#include "nvs_safe.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble/ble_adv.h"

static const char *TAG = "NVS_SAFE";

esp_err_t nvs_safe_commit(nvs_handle_t handle)
{
    int rc = ble_adv_pause();
    if (rc != 0) {
        /* 정지 실패해도 commit은 진행 — 설정 유실보다 BOD 리스크 감수가 낫고,
           이 실패 자체가 드문 예외 경로다. */
        ESP_LOGW(TAG, "ble_adv_pause rc=%d, committing anyway", rc);
    }

    /* 진행 중이던 adv 이벤트의 잔여 TX 종료 + 레일(벌크캡) 재충전 여유 */
    vTaskDelay(pdMS_TO_TICKS(5));

    esp_err_t err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_commit: %s", esp_err_to_name(err));
    }

    rc = ble_adv_resume();
    if (rc != 0) {
        /* 재개 실패 — WDT_HB_ADV stale로 워치독이 자가 재부팅 회수. 로그만. */
        ESP_LOGE(TAG, "ble_adv_resume rc=%d (watchdog will recover)", rc);
    }
    return err;
}
