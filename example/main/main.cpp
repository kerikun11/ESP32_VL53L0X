#include "freertos/FreeRTOS.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "VL53L0X.h"

#include "esp_log.h"
#define TAG "app_main"

extern "C" void app_main() {
  /* initialization */
  VL53L0X vl;
  vl.i2cMasterInit();
  if (!vl.init()) {
    ESP_LOGE(TAG, "Failed to initialize VL53L0X :(");
    vTaskDelay(portMAX_DELAY);
  }

  while (1) {
    /* measurement */
    uint16_t result_mm = 0;
    TickType_t tick_start = xTaskGetTickCount();
    bool res = vl.read(&result_mm);
    TickType_t tick_end = xTaskGetTickCount();
    int took_ms = ((int)tick_end - tick_start) / portTICK_PERIOD_MS;
    if (res)
      ESP_LOGI(TAG, "Range: %d [mm] took %d [ms]", (int)result_mm, took_ms);
    else
      ESP_LOGE(TAG, "Failed to measure :(");
  }
}
