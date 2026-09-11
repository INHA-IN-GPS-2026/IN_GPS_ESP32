# Validate the active sdkconfig, including ordinary idf.py / IDE builds.
# sdkconfig.defaults does not override values already saved in sdkconfig.
if(CONFIG_INGPS_BATCH_LOGGER)
    set(logger_config_errors "")
    if(NOT CONFIG_ULP_COPROC_ENABLED OR NOT CONFIG_ULP_COPROC_TYPE_RISCV)
        list(APPEND logger_config_errors "enable ULP RISC-V")
    endif()
    if(NOT CONFIG_ULP_COPROC_RESERVE_MEM EQUAL 7168)
        list(APPEND logger_config_errors "CONFIG_ULP_COPROC_RESERVE_MEM must be 7168 (active: ${CONFIG_ULP_COPROC_RESERVE_MEM})")
    endif()
    if(NOT CONFIG_PARTITION_TABLE_CUSTOM OR NOT CONFIG_PARTITION_TABLE_CUSTOM_FILENAME STREQUAL "partitions_logger.csv")
        list(APPEND logger_config_errors "select custom partition table partitions_logger.csv")
    endif()
    if(NOT CONFIG_BT_NIMBLE_ENABLED OR NOT CONFIG_BT_NIMBLE_ROLE_PERIPHERAL OR NOT CONFIG_BT_NIMBLE_GATT_SERVER)
        list(APPEND logger_config_errors "enable NimBLE peripheral and GATT server")
    endif()
    if(NOT CONFIG_BT_NIMBLE_MAX_CONNECTIONS EQUAL 1)
        list(APPEND logger_config_errors "CONFIG_BT_NIMBLE_MAX_CONNECTIONS must be 1")
    endif()
    if(logger_config_errors)
        list(JOIN logger_config_errors "\n  - " logger_config_errors_text)
        message(FATAL_ERROR
            "IN-GPS logger configuration mismatch:\n  - ${logger_config_errors_text}\n"
            "See LOGGER_README.md (ordinary ESP-IDF build settings). "
            "Run python tools/sync_logger_config.py before building, or use "
            "tools/build_logger.py for an isolated logger configuration. "
            "Editing sdkconfig.defaults alone will not replace an existing sdkconfig.")
    endif()
endif()
