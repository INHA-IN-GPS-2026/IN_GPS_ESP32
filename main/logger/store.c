#include <stdbool.h>
#include <string.h>
#include "store.h"
#include "esp_partition.h"
#include "esp_rom_crc.h"
#include "esp_attr.h"

#define RECORD_OFFSET 256u
#define COMMIT_MARK 0x314b4f4cu
static const esp_partition_t *partition;
static RTC_FAST_ATTR log_info_t info;
static RTC_FAST_ATTR uint32_t running_crc;
static bool complete;

esp_err_t log_store_open(void)
{
    partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x40, "logger");
    if (!partition || partition->size < RECORD_OFFSET + LOG_CAPACITY * sizeof(log_record_t))
        return ESP_ERR_NOT_FOUND;
    return ESP_OK;
}
bool log_store_complete(void)
{
    uint32_t markers[2];
    if (esp_partition_read(partition, 56, markers, sizeof(markers)) != ESP_OK ||
        markers[0] != COMMIT_MARK || markers[1] != UINT32_MAX) return false;
    log_info_t saved;
    if (esp_partition_read(partition, 32, &saved, sizeof(saved)) != ESP_OK ||
        saved.magic != LOG_MAGIC || saved.version != LOG_VERSION ||
        saved.record_size != sizeof(log_record_t) || saved.count != LOG_CAPACITY ||
        saved.interval_s != LOG_INTERVAL_S || saved.duration_s != LOG_DURATION_S) return false;
    uint32_t crc = 0;
    for (unsigned i = 0; i < saved.count; ++i) {
        log_record_t r;
        if (esp_partition_read(partition, RECORD_OFFSET + i * sizeof(r), &r, sizeof(r)) != ESP_OK ||
            r.index != i || r.crc16 != log_crc16(&r, 18)) return false;
        crc = esp_rom_crc32_le(crc, (const uint8_t *)&r, sizeof(r));
    }
    if (crc != saved.crc32) return false;
    info = saved; complete = true;
    return true;
}
esp_err_t log_store_begin(uint32_t session)
{
    esp_err_t e = esp_partition_erase_range(partition, 0, partition->size);
    if (e != ESP_OK) return e;
    info = (log_info_t){LOG_MAGIC, LOG_VERSION, sizeof(log_record_t), 0,
        LOG_INTERVAL_S, LOG_DURATION_S, session, 0};
    running_crc = 0; complete = false;
    return ESP_OK;
}
esp_err_t log_store_append(const log_record_t *r)
{
    return log_store_append_many(r, 1);
}
esp_err_t log_store_append_many(const log_record_t *r, unsigned count)
{
    if (!count || count > LOG_BANK_RECORDS || info.count + count > LOG_CAPACITY) return ESP_ERR_INVALID_ARG;
    for (unsigned i = 0; i < count; ++i)
        if (r[i].index != info.count + i || r[i].end_s != r[i].index + 1 ||
            r[i].crc16 != log_crc16(&r[i], 18)) return ESP_ERR_INVALID_ARG;
    unsigned bytes = count * sizeof(*r);
    esp_err_t e = esp_partition_write(partition, RECORD_OFFSET + info.count * sizeof(*r), r, bytes);
    if (e != ESP_OK) return e;
    log_record_t check[LOG_BANK_RECORDS];
    e = esp_partition_read(partition, RECORD_OFFSET + info.count * sizeof(*r), check, bytes);
    if (e != ESP_OK || memcmp(check, r, bytes)) return ESP_FAIL;
    running_crc = esp_rom_crc32_le(running_crc, (const uint8_t *)r, bytes);
    info.count += count;
    return ESP_OK;
}
esp_err_t log_store_finish(void)
{
    if (info.count != LOG_CAPACITY) return ESP_ERR_INVALID_STATE;
    info.crc32 = running_crc;
    esp_err_t e = esp_partition_write(partition, 32, &info, sizeof(info));
    if (e != ESP_OK) return e;
    uint32_t mark = COMMIT_MARK;
    e = esp_partition_write(partition, 56, &mark, sizeof(mark));
    complete = e == ESP_OK;
    return e;
}
esp_err_t log_store_read(unsigned index, log_record_t *r)
{
    if (!complete || index >= info.count) return ESP_ERR_INVALID_ARG;
    return esp_partition_read(partition, RECORD_OFFSET + index * sizeof(*r), r, sizeof(*r));
}
esp_err_t log_store_consume(void)
{
    uint32_t mark = 0;
    return esp_partition_write(partition, 60, &mark, sizeof(mark));
}
const log_info_t *log_store_info(void) { return &info; }
