idf_component_register(
    SRCS
        "app_main.c"
        "ble/ble_adv.c"
        "ble/adv_manager.c"
        "nvs_safe.c"
        "sensor/i2c_bus.c"
        "sensor/sensor.c"
        "sensor/as6221.c"
        "sensor/adxl345.c"
        # adc_cal.c: rev 4.0에는 ADC로 읽는 네트가 하나도 없다(ADXL335 X/Y/Z와
        # NTC 분압이 모두 사라짐). 배터리 모니터링 등 향후 아날로그 채널을 위해
        # 빌드에는 남겨 둔다 — 초기화하지 않으면 런타임 비용이 없다.
        "sensor/adc_cal.c"
        "watchdog/wdt_guard.c"
        "watchdog/wdt_test.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash bt esp_adc esp_driver_i2c hal led_strip esp_pm esp_hw_support
)

target_link_libraries(${COMPONENT_LIB} INTERFACE m)

# ★rev 4.0에서 ULP 서브시스템을 제거했다.
#   가속도계가 ADXL335(아날로그) -> ADXL345(I2C)로 바뀌면서 ULP RISC-V가
#   센서에 접근할 수 없게 됐기 때문이다(ULP는 I2C 마스터를 못 쓴다).
#   그 결과 ULP + SARADC + RTC_PERIPH 상시 ON이 통째로 사라졌고, 이것이
#   ADXL345 전환의 전력 실익 대부분이다(추정 300~900µA).
#   구 코드: main/ulp/{ulp_init.c,ulp_main.c}, main/ulp_shared.h
#           -> 커밋 5e8998c 또는 Analog_1.0.0_ver 브랜치 참조.
