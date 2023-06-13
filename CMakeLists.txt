set(COMPONENT_SRCS
    api/core/src/vl53l0x_api.c
    api/core/src/vl53l0x_api_calibration.c
    api/core/src/vl53l0x_api_core.c
    api/core/src/vl53l0x_api_ranging.c
    api/core/src/vl53l0x_api_strings.c
    api/platform/src/vl53l0x_platform.c
    api/platform/src/vl53l0x_platform_log.c
)

set(COMPONENT_ADD_INCLUDEDIRS
    inc
    api/core/inc
    api/platform/inc
)

set(COMPONENT_REQUIRES driver)

register_component()

target_compile_definitions(${COMPONENT_TARGET}
    PRIVATE USE_I2C_2V8=1
)

target_compile_options(${COMPONENT_TARGET}
    PRIVATE -mlongcalls
)
