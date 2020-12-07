#include "vl53l0x_platform_log.h"

uint32_t _trace_level = TRACE_LEVEL_ALL;
int _modules = TRACE_MODULE_ALL;

int32_t VL53L0X_trace_config(char *filename, uint32_t modules, uint32_t level, uint32_t functions)
{
    _trace_level = level;
    _modules = modules;
    return 0;
}
