idf_component_register(
    SRCS
        "app_main.c"
        "ble/ble_adv.c"
        "ble/adv_manager.c"
        "nvs_safe.c"
        "sensor/sensor.c"
        "sensor/as6221.c"
        # adc_cal.c: 이 브랜치에서는 호출되지 않는다(온도가 I2C로 넘어가며 NTC
        # raw→mV 곡선보정이 불필요해짐). 배터리 모니터링 등 향후 아날로그 채널을
        # 위해 빌드에는 남겨 둔다 — 초기화하지 않으면 런타임 비용이 없다.
        "sensor/adc_cal.c"
        "ulp/ulp_init.c"
        "watchdog/wdt_guard.c"
        "watchdog/wdt_test.c"
        # 진단 전용(branch flash_info_check). app_main.c의 INGPS_FLASH_REPORT가
        # 0이면 부팅 시 호출되지 않으므로 빌드에 남아 있어도 런타임 비용이 없다
        # (링커가 미참조 함수를 걷어내므로 이미지 크기 영향도 사실상 0).
        "diag/flash_report.c"
    INCLUDE_DIRS "."
    REQUIRES ulp nvs_flash bt esp_adc esp_driver_i2c hal led_strip esp_pm esp_hw_support
    # flash_report.c 의존: 플래시 크기·파티션·실행 이미지 조회.
    # esp_image_header_t는 esp_app_format 컴포넌트가 아니라 bootloader_support의
    # esp_app_format.h에 있다(IDF 5.5 기준). esp_app_desc.h만 esp_app_format 소속.
    # efuse: PSRAM_CAP/FLASH_CAP 직접 조회. esp_chip_info()는 S3에서 이 비트를
    #        채우지 않으므로(chip_info.c는 WIFI|BLE만 설정) eFuse가 유일한 근거.
    # esp_psram: CONFIG_SPIRAM=y로 바뀌었을 때 실제 초기화 크기 조회용.
    PRIV_REQUIRES spi_flash esp_partition app_update bootloader_support esp_app_format efuse esp_psram
)

target_link_libraries(${COMPONENT_LIB} INTERFACE m)

set(ulp_app_name "ulp_main")
set(ulp_sources "ulp/ulp_main.c")
set(ulp_exp_dep_srcs "app_main.c")

ulp_embed_binary(${ulp_app_name} "${ulp_sources}" "${ulp_exp_dep_srcs}")
