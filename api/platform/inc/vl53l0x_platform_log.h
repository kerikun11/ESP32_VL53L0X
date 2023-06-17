/*******************************************************************************
Copyright � 2015, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/


#ifndef _VL53L0X_PLATFORM_LOG_H_
#define _VL53L0X_PLATFORM_LOG_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "esp_log.h"

/* LOG Functions */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file vl53l0x_platform_log.h
 *
 * @brief platform log function definition
 */

//#define VL53L0X_LOG_ENABLE 0

enum {
    TRACE_LEVEL_NONE = ESP_LOG_NONE,
    TRACE_LEVEL_ERRORS = ESP_LOG_ERROR,
    TRACE_LEVEL_WARNING = ESP_LOG_WARN,
    TRACE_LEVEL_INFO = ESP_LOG_INFO,
    TRACE_LEVEL_DEBUG = ESP_LOG_DEBUG,
    TRACE_LEVEL_ALL = ESP_LOG_VERBOSE,
    TRACE_LEVEL_IGNORE
};

enum {
    TRACE_FUNCTION_NONE = 0,
    TRACE_FUNCTION_I2C  = 1,
    TRACE_FUNCTION_ALL  = 0x7fffffff //all bits except sign
};

enum {
    TRACE_MODULE_NONE              = 0x0,
    TRACE_MODULE_API               = 0x1,
    TRACE_MODULE_PLATFORM          = 0x2,
    TRACE_MODULE_ALL               = 0x7fffffff //all bits except sign
};

#define VL53L0X_LOG_ENABLE

#ifdef VL53L0X_LOG_ENABLE

extern uint32_t _trace_level;
extern int _modules;

int32_t VL53L0X_trace_config(char *filename, uint32_t modules, uint32_t level, uint32_t functions);

#define trace_print_module_function(module, level, function, format, ...) \
        if (level <= LOG_LOCAL_LEVEL && (module & _modules) != 0) \
            ESP_LOG_LEVEL(level, "VL53L0X", format, ##__VA_ARGS__)

#define LOG_GET_TIME() esp_log_timestamp()

#define _LOG_FUNCTION_START(module, fmt, ... ) \
        trace_print_module_function(module, _trace_level, TRACE_FUNCTION_ALL, "%" PRIu32 " <START> %s "fmt"\n", LOG_GET_TIME(), __FUNCTION__, ##__VA_ARGS__);

#define _LOG_FUNCTION_END(module, status, ... )\
        trace_print_module_function(module, _trace_level, TRACE_FUNCTION_ALL, "%" PRIu32 " <END> %s %d\n", LOG_GET_TIME(), __FUNCTION__, (int)status, ##__VA_ARGS__)

#define _LOG_FUNCTION_END_FMT(module, status, fmt, ... )\
        trace_print_module_function(module, _trace_level, TRACE_FUNCTION_ALL, "%" PRIu32 " <END> %s %d "fmt"\n", LOG_GET_TIME(),  __FUNCTION__, (int)status,##__VA_ARGS__)

#else /* VL53L0X_LOG_ENABLE no logging */
    #define VL53L0X_ErrLog(...) (void)0
    #define _LOG_FUNCTION_START(module, fmt, ... ) (void)0
    #define _LOG_FUNCTION_END(module, status, ... ) (void)0
    #define _LOG_FUNCTION_END_FMT(module, status, fmt, ... ) (void)0
#endif /* else */

#define VL53L0X_COPYSTRING(str, ...) strcpy(str, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif  /* _VL53L0X_PLATFORM_LOG_H_ */



