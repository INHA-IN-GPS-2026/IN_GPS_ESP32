idf_component_register(
    SRCS
        "app_main.c"
        "ble/ble_adv.c"
        "sensor/sensor.c"
        "ulp/ulp_init.c"
    INCLUDE_DIRS "."
    REQUIRES ulp nvs_flash bt esp_adc hal
)

target_link_libraries(${COMPONENT_LIB} INTERFACE m)

set(ulp_app_name "ulp_main")
set(ulp_sources "ulp/ulp_main.c")
set(ulp_exp_dep_srcs "app_main.c")

ulp_embed_binary(${ulp_app_name} "${ulp_sources}" "${ulp_exp_dep_srcs}")
