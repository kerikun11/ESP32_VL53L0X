#include <FreeRTOS.h>
#include <nvs_flash.h>
#include <VL53L0X.h>
#include <iostream>

/* config */
#define I2C_PORT I2C_NUM_0
#define PIN_SDA GPIO_NUM_21
#define PIN_SCL GPIO_NUM_22

#include "app_log.h"

extern "C" void app_main() {
  /* Boot Message */
  logi << "VL53L0X" << std::endl;

  /* NVS flash initialization */
  nvs_flash_init();

  /* initialization */
  VL53L0X vl(I2C_PORT);
  vl.i2cMasterInit(PIN_SDA, PIN_SCL);
  if (!vl.init()) {
    loge << "Failed to initialize VL53L0X :(" << std::endl;
    vTaskDelay(portMAX_DELAY);
  }
  /* Main Loop */

  while (1) {
    /* measurement */
    uint16_t result_mm = 0;
    TickType_t tick_start = xTaskGetTickCount();
    bool res = vl.read(&result_mm);
    TickType_t tick_end = xTaskGetTickCount();
    int took_ms = ((int)tick_end - tick_start) / portTICK_PERIOD_MS;
    if (res)
      logi << "Range: " << (int)result_mm << " [mm]"
           << " took " << took_ms << " [ms]" << std::endl;
    else
      loge << "Failed to measure :(" << std::endl;
  }

  /* sleep forever */
  vTaskDelay(portMAX_DELAY);
}
