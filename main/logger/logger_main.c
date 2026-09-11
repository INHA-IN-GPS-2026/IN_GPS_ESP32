#include <string.h>
#include "log_format.h"
#include "store.h"
#include "ulp_logger.h"
#include "ulp_riscv.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_rtc_time.h"
#include "esp_private/esp_clk.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern const uint8_t ulp_logger_bin_start[] asm("_binary_ulp_logger_bin_start");
extern const uint8_t ulp_logger_bin_end[] asm("_binary_ulp_logger_bin_end");
void logger_ble_run(void);
static RTC_FAST_ATTR uint32_t retained_magic, last_progress, stalled;
static RTC_FAST_ATTR uint64_t deadline;
static volatile log_shared_t *const state = (volatile log_shared_t *)&ulp_shared;

static void barrier(void) { __asm__ volatile("memw" ::: "memory"); }
static void missing_record(unsigned index)
{
    log_record_t r = {.index = index, .end_s = index + 1,
        .th1_x100 = LOG_INVALID_TEMP, .th2_x100 = LOG_INVALID_TEMP,
        .flags = LOG_MISSING | LOG_TIMING_GAP};
    r.crc16 = log_crc16(&r, 18);
    ESP_ERROR_CHECK(log_store_append(&r));
}
static void drain(void)
{
    /* Only read a bank after the ULP publishes ready, never the active bank.
       Choose the earlier bank even after a delayed HP wake. */
    for (unsigned pass = 0; pass < 2; ++pass) {
        int b = -1;
        for (unsigned j = 0; j < 2; ++j)
            if (state->ready[j] && (b < 0 || state->records[j][0].index < state->records[b][0].index)) b = j;
        if (b < 0) return;
        barrier();
        uint32_t count = state->count[b];
        if (!count || count > LOG_BANK_RECORDS) { stalled = 1; return; }
        log_record_t records[LOG_BANK_RECORDS];
        for (unsigned i = 0; i < count; ++i) {
            records[i] = state->records[b][i];
            if (records[i].index >= LOG_CAPACITY || records[i].crc16 != log_crc16(&records[i], 18) ||
                (i && records[i].index != records[0].index + i)) { stalled = 1; return; }
        }
        while (log_store_info()->count < records[0].index) missing_record(log_store_info()->count);
        if (records[0].index != log_store_info()->count) { stalled = 1; return; }
        ESP_ERROR_CHECK(log_store_append_many(records, count));
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        barrier(); state->ready[b] = 0;
    }
}
static void stop_ulp(void)
{
    state->stop = 1; barrier();
    vTaskDelay(pdMS_TO_TICKS(300)); /* normal transaction completes */
    ulp_riscv_timer_stop(); ulp_riscv_halt();
}
static void start_acquisition(void)
{
    ulp_riscv_timer_stop(); ulp_riscv_halt();
    ESP_ERROR_CHECK(log_store_begin(esp_random()));
    ESP_ERROR_CHECK(ulp_riscv_load_binary(ulp_logger_bin_start,
        ulp_logger_bin_end - ulp_logger_bin_start));
    state->magic = LOG_MAGIC;
    uint32_t cal = esp_clk_slowclk_cal_get();
    ESP_ERROR_CHECK(cal ? ESP_OK : ESP_FAIL);
    state->ticks_per_s = (1000000ULL << 19) / cal;
    uint8_t addresses[2] = {0x48, 0x49};
    nvs_handle_t handle;
    if (nvs_open("as6221", NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_u8(handle, "addr1", &addresses[0]);
        nvs_get_u8(handle, "addr2", &addresses[1]);
        nvs_close(handle);
    }
    if (addresses[0] < 0x44 || addresses[0] > 0x4b ||
        addresses[1] < 0x44 || addresses[1] > 0x4b || addresses[0] == addresses[1]) {
        addresses[0] = 0x48; addresses[1] = 0x49;
    }
    state->th_addr[0] = addresses[0]; state->th_addr[1] = addresses[1];
    last_progress = 0; stalled = 0;
    retained_magic = LOG_MAGIC;
    deadline = esp_rtc_get_time_us() + (LOG_DURATION_S + 2ULL) * 1000000ULL;
    ESP_ERROR_CHECK(ulp_set_wakeup_period(0, 150000));
    ESP_ERROR_CHECK(ulp_riscv_run());
    /* Permit initial setup to finish before the independent timer is armed. */
    for (unsigned i = 0; i < 100 && !state->initialized; ++i) vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI("LOGGER", "RF off: 1800 x 1s records, TH1=0x%02x TH2=0x%02x", addresses[0], addresses[1]);
}
void app_main(void)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL)); /* IDF task/interrupt watchdogs remain enabled */
    ESP_ERROR_CHECK(log_store_open());
    bool resumed = esp_reset_reason() == ESP_RST_DEEPSLEEP && retained_magic == LOG_MAGIC &&
        state->magic == LOG_MAGIC && state->ticks_per_s > 1000 && state->ticks_per_s < 1000000;
    if (!resumed) {
        /* Do not erase NVS on an init error: addresses/calibration belong to the user. */
        ESP_ERROR_CHECK(nvs_flash_init());
        if (log_store_complete()) {
            ulp_riscv_timer_stop(); ulp_riscv_halt();
            logger_ble_run(); return;
        }
        start_acquisition();
    }
    drain();
    if (!state->done && state->progress == last_progress && state->initialized && resumed) stalled = 1;
    last_progress = state->progress;
    if (stalled) stop_ulp();
    if (state->done || esp_rtc_get_time_us() >= deadline) {
        stop_ulp(); drain();
        while (log_store_info()->count < LOG_CAPACITY) missing_record(log_store_info()->count);
        ESP_ERROR_CHECK(log_store_finish());
        retained_magic = 0;
        ESP_ERROR_CHECK(nvs_flash_init());
        logger_ble_run(); return;
    }
    /* No Bluetooth API is called on this path, including bank/timer wakes. */
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(30ULL * 1000000));
    if (!stalled) ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
    ESP_ERROR_CHECK(esp_task_wdt_reset());
    esp_deep_sleep_start();
}
