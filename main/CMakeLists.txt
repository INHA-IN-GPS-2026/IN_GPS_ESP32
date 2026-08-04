idf_component_register(
    SRCS
        "app_main.c"
        "ble/ble_adv.c"
        "ble/adv_manager.c"
        "nvs_safe.c"
        "sensor/sensor.c"
        "sensor/adc_cal.c"
        "ulp/ulp_init.c"
        "watchdog/wdt_guard.c"
        "watchdog/wdt_test.c"
    INCLUDE_DIRS "."
    REQUIRES ulp nvs_flash bt esp_adc hal led_strip esp_pm esp_hw_support
)

target_link_libraries(${COMPONENT_LIB} INTERFACE m)

set(ulp_app_name "ulp_main")
set(ulp_sources "ulp/ulp_main.c")
set(ulp_exp_dep_srcs "app_main.c")

ulp_embed_binary(${ulp_app_name} "${ulp_sources}" "${ulp_exp_dep_srcs}")
