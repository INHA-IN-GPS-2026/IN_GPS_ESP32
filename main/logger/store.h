#pragma once
#include <stdbool.h>
#include "log_format.h"
#include "esp_err.h"
typedef struct {
    uint32_t magic;
    uint8_t version, record_size;
    uint16_t count, interval_s, duration_s;
    uint32_t session, crc32;
} log_info_t;
_Static_assert(sizeof(log_info_t) == 20, "metadata wire size");
esp_err_t log_store_open(void);
bool log_store_complete(void);
esp_err_t log_store_begin(uint32_t session);
esp_err_t log_store_append(const log_record_t *record);
esp_err_t log_store_append_many(const log_record_t *records, unsigned count);
esp_err_t log_store_finish(void);
esp_err_t log_store_read(unsigned index, log_record_t *record);
esp_err_t log_store_consume(void);
const log_info_t *log_store_info(void);
