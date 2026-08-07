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
    INCLUDE_DIRS "."
    REQUIRES ulp nvs_flash bt esp_adc esp_driver_i2c hal led_strip esp_pm esp_hw_support
)

target_link_libraries(${COMPONENT_LIB} INTERFACE m)

set(ulp_app_name "ulp_main")
set(ulp_sources "ulp/ulp_main.c")
set(ulp_exp_dep_srcs "app_main.c")

ulp_embed_binary(${ulp_app_name} "${ulp_sources}" "${ulp_exp_dep_srcs}")
