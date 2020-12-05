set(srcs "src/vl53l0x_api.c"
        "src/vl53l0x_api_calibration.c"
        "src/vl53l0x_api_core.c"
        "src/vl53l0x_api_ranging.c"
	"src/vl53l0x_api_strings.c"
	"src/vl53l0x_platform.c"
	"src/vl53l0x_platform_log.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS include
                    REQUIRES driver)