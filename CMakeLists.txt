set(COMPONENT_PRIV_REQUIRES log)

set(COMPONENT_SRCS 
  "core/src/vl53l0x_api.c"
  "core/src/vl53l0x_api_calibration.c"
  "core/src/vl53l0x_api_core.c"
  "core/src/vl53l0x_api_ranging.c"
  "core/src/vl53l0x_api_strings.c"
  "platform/src/vl53l0x_platform.c"
  "platform/src/vl53l0x_platform_log.c"
)

set(COMPONENT_ADD_INCLUDEDIRS "." "core/inc" "platform/inc")

register_component()
