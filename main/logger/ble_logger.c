#include <string.h>
#include <stdio.h>
#include "store.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* 8c5e000{1..4}-7a6b-4d21-9c35-494e47505331 (NimBLE little-endian UUID). */
#define UUID(n) BLE_UUID128_INIT(0x31,0x53,0x50,0x47,0x4e,0x49,0x35,0x9c,0x21,0x4d,0x6b,0x7a,n,0x00,0x5e,0x8c)
static const ble_uuid128_t service_uuid = UUID(1), info_uuid = UUID(2),
    data_uuid = UUID(3), control_uuid = UUID(4);
static uint16_t data_handle, connection = BLE_HS_CONN_HANDLE_NONE, cursor, selected;
static uint8_t own_addr_type;
static bool subscribed, streaming, pending, stopping;
static char device_name[28];
static QueueHandle_t action_queue;
static struct ble_npl_event tx_event;
static volatile bool host_ready;

static void check(int rc) { ESP_ERROR_CHECK(rc == 0 ? ESP_OK : ESP_FAIL); }
static void queue_next(void) { ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &tx_event); }
static void transmit(struct ble_npl_event *event)
{
    (void)event;
    if (!streaming || !subscribed || pending || connection == BLE_HS_CONN_HANDLE_NONE) return;
    if (cursor >= log_store_info()->count) { streaming = false; return; }
    log_record_t r;
    if (log_store_read(cursor, &r) != ESP_OK) { streaming = false; return; }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(&r, sizeof(r));
    if (!om) { streaming = false; return; } /* client can replay/read any index */
    pending = true;
    int rc = ble_gatts_indicate_custom(connection, data_handle, om);
    if (rc) { pending = false; streaming = false; }
}
static int access_characteristic(uint16_t conn, uint16_t attr,
    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn; (void)attr;
    if (stopping) return BLE_ATT_ERR_UNLIKELY;
    uintptr_t which = (uintptr_t)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if (which == 2) return os_mbuf_append(ctxt->om, log_store_info(), sizeof(log_info_t)) ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
        if (which == 3) {
            log_record_t r;
            if (log_store_read(selected, &r) != ESP_OK) return BLE_ATT_ERR_UNLIKELY;
            return os_mbuf_append(ctxt->om, &r, sizeof(r)) ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
        }
    }
    if (which != 4 || ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    uint8_t cmd[9];
    if (!len || len > sizeof(cmd) || os_mbuf_copydata(ctxt->om, 0, len, cmd)) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    if (cmd[0] == 1 || cmd[0] == 2) {
        if (len != 3 && !(cmd[0] == 1 && len == 1)) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        unsigned index = len == 1 ? 0 : cmd[1] | (unsigned)cmd[2] << 8;
        if (index >= log_store_info()->count) return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
        if (pending || streaming) return BLE_ATT_ERR_UNLIKELY;
        if (cmd[0] == 2) selected = index;
        else {
            if (!subscribed) return BLE_ATT_ERR_UNLIKELY;
            cursor = index; streaming = true; queue_next();
        }
        return 0;
    }
    if (cmd[0] == 5 && len == 1) { streaming = false; return 0; }
    if ((cmd[0] == 3 || cmd[0] == 4) && len == 9) {
        uint32_t session, crc;
        memcpy(&session, cmd + 1, 4); memcpy(&crc, cmd + 5, 4);
        if (session != log_store_info()->session || crc != log_store_info()->crc32 || pending || streaming)
            return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
        if (xQueueSend(action_queue, cmd, 0) != pdTRUE) return BLE_ATT_ERR_INSUFFICIENT_RES;
        stopping = true; /* suppress re-advertising during graceful host shutdown */
        return 0;
    }
    return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
}
static const struct ble_gatt_svc_def services[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &service_uuid.u,
     .characteristics = (struct ble_gatt_chr_def[]) {
        {.uuid = &info_uuid.u, .access_cb = access_characteristic, .arg = (void *)2, .flags = BLE_GATT_CHR_F_READ},
        {.uuid = &data_uuid.u, .access_cb = access_characteristic, .arg = (void *)3,
         .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE, .val_handle = &data_handle},
        {.uuid = &control_uuid.u, .access_cb = access_characteristic, .arg = (void *)4, .flags = BLE_GATT_CHR_F_WRITE},
        {0}}}, {0}
};
static int gap_event(struct ble_gap_event *event, void *arg);
static void advertise(void)
{
    struct ble_hs_adv_fields fields = {0}, scan = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)&service_uuid; fields.num_uuids128 = 1; fields.uuids128_is_complete = 1;
    scan.name = (uint8_t *)device_name; scan.name_len = strlen(device_name); scan.name_is_complete = 1;
    check(ble_gap_adv_set_fields(&fields)); /* 3 + 18 = 21 B; name in scan response */
    check(ble_gap_adv_rsp_set_fields(&scan));
    struct ble_gap_adv_params params = {.conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN, .itvl_min = 800, .itvl_max = 960};
    check(ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event, NULL));
}
static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (!event->connect.status) connection = event->connect.conn_handle;
        else if (!stopping) advertise();
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        connection = BLE_HS_CONN_HANDLE_NONE;
        subscribed = streaming = pending = false; selected = 0;
        if (!stopping) advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == data_handle) {
            subscribed = event->subscribe.cur_indicate;
            if (!subscribed) streaming = false;
        }
        break;
    case BLE_GAP_EVENT_NOTIFY_TX:
        if (event->notify_tx.attr_handle == data_handle && event->notify_tx.indication && pending) {
            /* Status 0 means queued/sent, EDONE means peer ATT confirmation. */
            if (event->notify_tx.status == BLE_HS_EDONE) {
                pending = false; ++cursor;
                if (cursor >= log_store_info()->count) streaming = false;
                else queue_next();
            } else if (event->notify_tx.status != 0) { pending = false; streaming = false; }
        }
        break;
    default: break;
    }
    return 0;
}
static void synchronized(void)
{
    check(ble_hs_util_ensure_addr(0));
    check(ble_hs_id_infer_auto(0, &own_addr_type));
    uint8_t address[6]; check(ble_hs_id_copy_addr(own_addr_type, address, NULL));
    snprintf(device_name, sizeof(device_name), "IN_GPS_LOG_%02X%02X%02X", address[2], address[1], address[0]);
    check(ble_svc_gap_device_name_set(device_name));
    advertise(); host_ready = true;
}
static void host(void *arg)
{
    (void)arg; nimble_port_run(); nimble_port_freertos_deinit();
}
void logger_ble_run(void)
{
    ESP_LOGI("LOGGER", "Batch ready: %u records; starting direct GATT", log_store_info()->count);
    action_queue = xQueueCreate(1, sizeof(uint8_t));
    ESP_ERROR_CHECK(action_queue ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(nimble_port_init());
    ble_svc_gap_init(); ble_svc_gatt_init();
    check(ble_gatts_count_cfg(services)); check(ble_gatts_add_svcs(services));
    ble_npl_event_init(&tx_event, transmit, NULL);
    ble_hs_cfg.sync_cb = synchronized;
    nimble_port_freertos_init(host);
    int64_t sync_deadline = esp_timer_get_time() + 15000000;
    for (;;) {
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        ESP_ERROR_CHECK(host_ready || esp_timer_get_time() < sync_deadline ? ESP_OK : ESP_ERR_TIMEOUT);
        uint8_t action;
        if (xQueueReceive(action_queue, &action, pdMS_TO_TICKS(500)) != pdTRUE) continue;
        /* Let the write response reach the peer before stopping the host. */
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP_ERROR_CHECK(nimble_port_stop());
        ESP_ERROR_CHECK(nimble_port_deinit());
        if (action == 4) { ESP_ERROR_CHECK(log_store_consume()); esp_restart(); }
        ESP_ERROR_CHECK(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL));
        esp_deep_sleep_start(); /* FINISH retains flash. Reset allows download again. */
    }
}
