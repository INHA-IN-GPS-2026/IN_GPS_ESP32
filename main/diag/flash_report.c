/*
 * flash_report.c — 플래시 용량/사용량 런타임 리포트 (진단 전용)
 *
 * 출력
 *   [1] 플래시 칩   : JEDEC ID, 물리 크기 vs sdkconfig 설정 크기(불일치 경고)
 *   [2] PSRAM       : eFuse 기준 칩 내장 용량 + 펌웨어 사용 여부 (RAM, 참고용)
 *   [3] 파티션 테이블: 부트로더·테이블 포함 전체 레이아웃
 *   [4] app 이미지  : 실행 중 펌웨어의 실제 바이트 수 / 파티션 사용률
 *   [5] 부트로더    : 이미지 크기 / 가용 영역
 *   [6] NVS         : 엔트리 사용률
 *   [7] 요약        : 총 용량 대비 할당·미할당
 *
 * 원본: C:\esp\flash_info (단독 프로젝트). 이 브랜치용으로 app_main 호출형
 * 함수 하나로 축약하고, 전역 로그 묵음을 우회하도록 printf로 통일했다.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_flash.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_app_format.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "esp_heap_caps.h"
#if CONFIG_SPIRAM
#include "esp_psram.h"
#endif

#include "diag/flash_report.h"

/* 부트로더 위치: ESP32는 0x1000, S3는 0x0. IDF가 Kconfig로 노출한다. */
#ifdef CONFIG_BOOTLOADER_OFFSET_IN_FLASH
#define BOOTLOADER_OFFSET CONFIG_BOOTLOADER_OFFSET_IN_FLASH
#else
#define BOOTLOADER_OFFSET 0x0
#endif

#define PART_TABLE_OFFSET CONFIG_PARTITION_TABLE_OFFSET
#define PART_TABLE_SIZE   0x1000   /* 파티션 테이블은 한 섹터를 차지 */

/* ------------------------------------------------------------------ */

static void print_bytes(const char *label, uint64_t bytes)
{
    printf("  %-26s %9" PRIu64 " B (%8.2f KB / %6.3f MB)\n",
           label, bytes, bytes / 1024.0, bytes / (1024.0 * 1024.0));
}

static const char *flash_vendor(uint8_t mfg_id)
{
    switch (mfg_id) {
    case 0xEF: return "Winbond";
    case 0xC8: return "GigaDevice";
    case 0x20: return "XMC/Micron";
    case 0x9D: return "ISSI";
    case 0x1C: return "Eon";
    case 0x0B: return "XTX";
    case 0x68: return "Boya";
    case 0x85: return "Puya";
    case 0xA1: return "Fudan";
    case 0x5E: return "Zbit";
    case 0xC2: return "Macronix";
    default:   return "unknown";
    }
}

static const char *part_type_name(esp_partition_type_t type)
{
    switch (type) {
    case ESP_PARTITION_TYPE_APP:  return "app";
    case ESP_PARTITION_TYPE_DATA: return "data";
    default:                      return "custom";
    }
}

static const char *part_subtype_name(esp_partition_type_t type, esp_partition_subtype_t st)
{
    static char buf[8];

    if (type == ESP_PARTITION_TYPE_APP) {
        if (st == ESP_PARTITION_SUBTYPE_APP_FACTORY) return "factory";
        if (st == ESP_PARTITION_SUBTYPE_APP_TEST)    return "test";
        if (st >= ESP_PARTITION_SUBTYPE_APP_OTA_MIN && st < ESP_PARTITION_SUBTYPE_APP_OTA_MAX) {
            snprintf(buf, sizeof(buf), "ota_%d", st - ESP_PARTITION_SUBTYPE_APP_OTA_MIN);
            return buf;
        }
        return "app_?";
    }
    if (type == ESP_PARTITION_TYPE_DATA) {
        switch (st) {
        case ESP_PARTITION_SUBTYPE_DATA_OTA:      return "ota_data";
        case ESP_PARTITION_SUBTYPE_DATA_PHY:      return "phy";
        case ESP_PARTITION_SUBTYPE_DATA_NVS:      return "nvs";
        case ESP_PARTITION_SUBTYPE_DATA_COREDUMP: return "coredump";
        case ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS: return "nvs_keys";
        case ESP_PARTITION_SUBTYPE_DATA_FAT:      return "fat";
        case ESP_PARTITION_SUBTYPE_DATA_SPIFFS:   return "spiffs";
        case ESP_PARTITION_SUBTYPE_DATA_LITTLEFS: return "littlefs";
        default:                                  return "data_?";
        }
    }
    return "-";
}

/* 이미지 읽기 소스: 파티션(암호화 자동 복호) 또는 raw 플래시 주소 */
typedef struct {
    const esp_partition_t *part;   /* NULL이면 raw 읽기 */
    size_t                 base;   /* raw 읽기 시작 주소 */
} img_src_t;

static esp_err_t img_read(const img_src_t *src, size_t offset, void *dst, size_t len)
{
    if (src->part) {
        return esp_partition_read(src->part, offset, dst, len);
    }
    return esp_flash_read(esp_flash_default_chip, dst, src->base + offset, len);
}

/*
 * ESP 이미지의 실제 크기를 계산한다.
 *   [image header][segment header + data] × N [padding][1B checksum][SHA-256(옵션)]
 * 체크섬 바이트가 16바이트 경계 직전에 오도록 패딩되므로 세그먼트 끝을 16으로 올림한다.
 */
static esp_err_t image_used_size(const img_src_t *src, size_t *out_size)
{
    esp_image_header_t hdr;
    esp_err_t err = img_read(src, 0, &hdr, sizeof(hdr));
    if (err != ESP_OK) {
        return err;
    }
    if (hdr.magic != ESP_IMAGE_HEADER_MAGIC) {
        return ESP_ERR_NOT_FOUND;   /* 이미지 없음 또는 암호화되어 헤더를 못 읽음 */
    }

    size_t off = sizeof(esp_image_header_t);
    for (int i = 0; i < hdr.segment_count; i++) {
        esp_image_segment_header_t seg;
        err = img_read(src, off, &seg, sizeof(seg));
        if (err != ESP_OK) {
            return err;
        }
        if (seg.data_len > 16 * 1024 * 1024) {
            return ESP_ERR_INVALID_SIZE;   /* 손상된 헤더 방어 */
        }
        off += sizeof(seg) + seg.data_len;
    }

    off = ((off + 16) / 16) * 16;          /* 패딩 + 체크섬 1바이트 */
    if (hdr.hash_appended) {
        off += 32;                         /* SHA-256 다이제스트 */
    }

    *out_size = off;
    return ESP_OK;
}

/* ------------------------------------------------------------------ */

/* 반환값: 물리 플래시 크기(B). 읽기 실패 시 설정 크기로 대체. */
static uint32_t report_chip(void)
{
    uint32_t id = 0, cfg_size = 0, phys_size = 0;

    printf("\n[1] 플래시 칩\n");

    if (esp_flash_read_id(esp_flash_default_chip, &id) == ESP_OK) {
        uint8_t mfg = (id >> 16) & 0xFF;
        printf("  JEDEC ID    : 0x%06" PRIx32 "  (mfg 0x%02X = %s, dev 0x%04" PRIx32 ")\n",
               id, mfg, flash_vendor(mfg), id & 0xFFFF);
    } else {
        printf("  JEDEC ID    : 읽기 실패\n");
    }

    if (esp_flash_get_size(esp_flash_default_chip, &cfg_size) != ESP_OK) {
        cfg_size = 0;
    }
    if (esp_flash_get_physical_size(esp_flash_default_chip, &phys_size) != ESP_OK) {
        phys_size = 0;
    }

    print_bytes("물리 크기(실제 칩)", phys_size);
    print_bytes("설정 크기(sdkconfig)", cfg_size);
    printf("  %-26s %s\n", "CONFIG_ESPTOOLPY_FLASHSIZE", CONFIG_ESPTOOLPY_FLASHSIZE);

    if (phys_size && cfg_size && phys_size != cfg_size) {
        printf("  *** 경고: 물리 %.0fMB != 설정 %.0fMB — 초과분 %.0fMB는 사용 불가 ***\n"
               "  ***       menuconfig > Serial flasher config > Flash size 확인 ***\n",
               phys_size / 1048576.0, cfg_size / 1048576.0,
               (phys_size - cfg_size) / 1048576.0);
    }

    return phys_size ? phys_size : cfg_size;
}

/*
 * PSRAM — 이 보드는 ESP32-S3R2(내장 PSRAM 2MB) 패키지다.
 *
 * ⚠ PSRAM은 RAM이지 플래시가 아니다. 위 [1]의 16MB 플래시 계산과는 완전히
 *   별개이며 서로 섞이지 않는다. 다만 "R2니까 2MB가 있다"는 사실과 "펌웨어가
 *   그걸 쓰고 있다"는 전혀 다른 얘기라, 둘을 나눠 찍는다.
 *
 * ⚠ esp_chip_info()의 CHIP_FEATURE_EMB_PSRAM은 쓰지 않는다. S3의 구현
 *   (esp_hw_support/port/esp32s3/chip_info.c)은 features에 WIFI_BGN|BLE만
 *   채우고 내장 플래시/PSRAM 비트를 아예 건드리지 않아서, R2 패키지에서도
 *   항상 "없음"이 나온다. esptool과 동일하게 eFuse를 직접 읽어야 맞다.
 */
static void report_psram(void)
{
    printf("\n[2] PSRAM (참고 — 플래시 아님)\n");

#if CONFIG_IDF_TARGET_ESP32S3
    /* PSRAM_CAP은 BLK1 bit131(2비트) + bit179(1비트)로 흩어진 3비트 합성 필드.
       esp_efuse_read_field_blob이 두 조각을 합쳐 주므로 esptool의
       get_psram_cap()과 같은 값이 나온다. */
    uint32_t cap = 0, vendor = 0, temp = 0;
    esp_efuse_read_field_blob(ESP_EFUSE_PSRAM_CAP, &cap, 3);
    esp_efuse_read_field_blob(ESP_EFUSE_PSRAM_VENDOR, &vendor, 2);
    esp_efuse_read_field_blob(ESP_EFUSE_PSRAM_TEMP, &temp, 2);

    /* esptool targets/esp32s3.py get_chip_features()와 동일한 매핑 */
    const char *cap_str = (cap == 0) ? "없음"
                        : (cap == 1) ? "8MB"
                        : (cap == 2) ? "2MB  (= S3-R2 패키지)"
                        : (cap == 3) ? "16MB"
                        : (cap == 4) ? "4MB" : "코드 미상";
    const char *ven_str = (vendor == 1) ? "AP_3v3" : (vendor == 2) ? "AP_1v8" : "-";
    const char *tmp_str = (temp == 1) ? "105C" : (temp == 2) ? "85C" : "-";

    printf("  칩 내장 용량: %s  (eFuse CAP=%" PRIu32 ", vendor=%s, temp=%s)\n",
           cap_str, cap, ven_str, tmp_str);
#else
    printf("  칩 내장 용량: 판정 생략 (S3 외 타깃은 eFuse 필드가 다름)\n");
#endif

#if CONFIG_SPIRAM
    printf("  펌웨어 사용 : CONFIG_SPIRAM=y\n");
    if (esp_psram_is_initialized()) {
        print_bytes("PSRAM 초기화 크기", esp_psram_get_size());
        print_bytes("PSRAM 힙 총량", heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
        print_bytes("PSRAM 힙 여유", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    } else {
        printf("  *** CONFIG_SPIRAM=y 인데 초기화 실패 — 배선/전압/모드 확인 ***\n");
    }
#else
    /* 이 펌웨어의 현재 상태. 1년 전력 설계상 PSRAM은 켤 이유가 없다 —
       상시 리프레시 전류가 붙고 광고 페이로드(수십 바이트) 처리에 내부 SRAM으로
       충분하다. 즉 "미사용"은 실수가 아니라 의도된 선택이다. */
    printf("  펌웨어 사용 : CONFIG_SPIRAM=n  ->  내장 PSRAM 전량 미사용(유휴)\n");
#endif
}

/* 반환값: 파티션이 차지하는 마지막 끝 주소 */
static uint32_t report_partitions(void)
{
    printf("\n[3] 파티션 테이블\n");
    printf("  %-14s %-6s %-8s %-8s %-8s %8s\n",
           "이름", "타입", "서브타입", "시작", "끝", "크기KB");
    printf("  ---------------------------------------------------------------\n");

    printf("  %-14s %-6s %-8s %08X %08X %8.1f\n", "(bootloader)", "-", "-",
           BOOTLOADER_OFFSET, PART_TABLE_OFFSET,
           (PART_TABLE_OFFSET - BOOTLOADER_OFFSET) / 1024.0);
    printf("  %-14s %-6s %-8s %08X %08X %8.1f\n", "(part.table)", "-", "-",
           PART_TABLE_OFFSET, PART_TABLE_OFFSET + PART_TABLE_SIZE,
           PART_TABLE_SIZE / 1024.0);

    uint32_t total    = 0;
    uint32_t last_end = PART_TABLE_OFFSET + PART_TABLE_SIZE;

    esp_partition_iterator_t it =
        esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
    while (it != NULL) {
        const esp_partition_t *p = esp_partition_get(it);
        printf("  %-14s %-6s %-8s %08" PRIx32 " %08" PRIx32 " %8.1f\n",
               p->label, part_type_name(p->type),
               part_subtype_name(p->type, p->subtype),
               p->address, p->address + p->size, p->size / 1024.0);
        total += p->size;
        if (p->address + p->size > last_end) {
            last_end = p->address + p->size;
        }
        it = esp_partition_next(it);
    }
    esp_partition_iterator_release(it);

    printf("  ---------------------------------------------------------------\n");
    print_bytes("파티션 합계", total);

    return last_end;
}

static void report_app(void)
{
    printf("\n[4] 실행 중인 app 이미지\n");

    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == NULL) {
        printf("  실행 파티션을 확인할 수 없습니다.\n");
        return;
    }

    const esp_app_desc_t *desc = esp_app_get_description();
    if (desc) {
        printf("  프로젝트    : %s (v%s)\n", desc->project_name, desc->version);
        printf("  빌드        : %s %s / IDF %s\n", desc->date, desc->time, desc->idf_ver);
    }
    printf("  파티션      : %s @ 0x%08" PRIx32 "\n", running->label, running->address);

    img_src_t src = { .part = running, .base = 0 };
    size_t used = 0;
    if (image_used_size(&src, &used) != ESP_OK) {
        printf("  이미지 크기 계산 실패 (암호화 또는 헤더 손상)\n");
        return;
    }

    print_bytes("app 이미지 사용", used);
    print_bytes("app 파티션 크기", running->size);
    print_bytes("app 파티션 여유", (uint64_t)running->size - used);
    printf("  %-26s %.1f %%\n", "app 파티션 사용률", 100.0 * used / running->size);

    if (used > (size_t)(running->size * 0.9)) {
        printf("  *** 경고: app 파티션 90%% 초과 — 파티션 테이블 조정 필요 ***\n");
    }
}

static void report_bootloader(void)
{
    printf("\n[5] 부트로더\n");

    img_src_t src = { .part = NULL, .base = BOOTLOADER_OFFSET };
    size_t used = 0;
    if (image_used_size(&src, &used) != ESP_OK) {
        printf("  부트로더 크기 계산 실패 (플래시 암호화 시 정상)\n");
        return;
    }

    print_bytes("부트로더 사용", used);
    print_bytes("부트로더 가용 영역", (uint64_t)PART_TABLE_OFFSET - BOOTLOADER_OFFSET);
}

/* ⚠ nvs_flash_init()은 호출하지 않는다 — app_main이 이미 초기화했고,
   여기서 다시 부르면 (에러 경로에서) erase까지 얽힐 수 있다. */
static void report_nvs(void)
{
    printf("\n[6] NVS\n");

    nvs_stats_t st;
    esp_err_t err = nvs_get_stats(NULL, &st);
    if (err != ESP_OK) {
        printf("  nvs_get_stats 실패: %s (NVS 미초기화?)\n", esp_err_to_name(err));
        return;
    }

    /* free_entries는 예약분을 포함하므로 실제로 쓸 수 있는 양은 available_entries다. */
    printf("  %-26s %u / %u (저장가능 %u, free %u)\n", "엔트리",
           (unsigned)st.used_entries, (unsigned)st.total_entries,
           (unsigned)st.available_entries, (unsigned)st.free_entries);
    printf("  %-26s %u\n", "네임스페이스 수", (unsigned)st.namespace_count);
    print_bytes("NVS 사용(엔트리×32B)", (uint64_t)st.used_entries * 32);
    if (st.total_entries) {
        printf("  %-26s %.1f %%\n", "NVS 사용률",
               100.0 * st.used_entries / st.total_entries);
    }
}

static void report_summary(uint32_t flash_size, uint32_t last_end)
{
    printf("\n[7] 요약\n");
    print_bytes("플래시 총 용량", flash_size);
    print_bytes("할당된 끝 주소", last_end);

    if (flash_size > last_end) {
        print_bytes("미할당(남은 공간)", (uint64_t)flash_size - last_end);
        printf("  %-26s %.1f %%\n", "할당률", 100.0 * last_end / flash_size);
    } else if (flash_size == last_end) {
        printf("  플래시 전체가 파티션으로 할당되어 있습니다.\n");
    } else {
        printf("  *** 오류: 파티션 끝(0x%08" PRIx32 ")이 플래시 크기(0x%08" PRIx32 ")를 초과 ***\n",
               last_end, flash_size);
    }
}

/* ------------------------------------------------------------------ */

void flash_report_print(void)
{
    printf("\n================ FLASH USAGE REPORT ================\n");

    uint32_t flash_size = report_chip();
    report_psram();
    uint32_t last_end   = report_partitions();
    report_app();
    report_bootloader();
    report_nvs();
    report_summary(flash_size, last_end);

    printf("====================================================\n\n");
    fflush(stdout);
}
