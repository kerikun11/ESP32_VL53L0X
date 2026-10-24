idf_component_register(
    SRCS
        api/core/src/vl53l0x_api.c
        api/core/src/vl53l0x_api_calibration.c
        api/core/src/vl53l0x_api_core.c
        api/core/src/vl53l0x_api_ranging.c
        api/core/src/vl53l0x_api_strings.c
        api/platform/src/vl53l0x_platform.c
        api/platform/src/vl53l0x_platform_log.c
    INCLUDE_DIRS
        inc
        api/core/inc
        api/platform/inc
    REQUIRES
        esp_driver_i2c
        esp_driver_gpio
)

target_compile_definitions(${COMPONENT_LIB}
    PRIVATE USE_I2C_2V8=1
)

target_compile_options(${COMPONENT_LIB}
    PRIVATE -mlongcalls
)
