if(CONFIG_INGPS_BATCH_LOGGER)
    set(app_sources "logger/logger_main.c" "logger/store.c" "logger/ble_logger.c")
else()
    set(app_sources "app_main.c" "ble/ble_adv.c" "ble/adv_manager.c"
        "nvs_safe.c" "sensor/i2c_bus.c" "sensor/sensor.c" "sensor/as6221.c"
        "sensor/adxl345.c" "sensor/adc_cal.c" "watchdog/wdt_guard.c" "watchdog/wdt_test.c")
endif()
# Dependency expansion runs before sdkconfig is loaded. Keep REQUIRES unconditional.
idf_component_register(SRCS ${app_sources}
    INCLUDE_DIRS "." "logger"
    REQUIRES nvs_flash bt ulp esp_partition esp_timer esp_system esp_hw_support
             esp_adc esp_driver_i2c hal led_strip esp_pm)
target_link_libraries(${COMPONENT_LIB} INTERFACE m)
if(CONFIG_INGPS_BATCH_LOGGER)
    include("${CMAKE_CURRENT_LIST_DIR}/logger/check_config.cmake")
    ulp_embed_binary(ulp_logger
        "logger/ulp/main.c;logger/ulp/soft_i2c.c"
        "logger/logger_main.c")
    idf_build_get_property(idf_python PYTHON)
    add_custom_command(TARGET ${COMPONENT_LIB} POST_BUILD
        COMMAND "${idf_python}" "${CMAKE_CURRENT_LIST_DIR}/../tools/check_ulp_instructions.py"
                "${CMAKE_CURRENT_BINARY_DIR}/ulp_logger/ulp_logger.elf"
        COMMENT "Checking S3 ULP instruction compatibility"
        VERBATIM)
endif()
# GPIO8/9: ULP RTC-GPIO software I2C; dedicated RTC I2C uses other pins.
