/**
 * @file app_main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief VL53L0X driver example
 * @date 2020-12-07
 * @copyright Copyright (c) 2020 Ryotaro Onuki
 */
#include "VL53L0X.h"

#include "esp_log.h"
#define TAG "app_main"

/* config */
#define I2C_PORT I2C_NUM_0
#define PIN_SDA GPIO_NUM_21
#define PIN_SCL GPIO_NUM_22

extern "C" void app_main() {
  /* initialization */
  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = I2C_PORT;
  bus_cfg.sda_io_num = PIN_SDA;
  bus_cfg.scl_io_num = PIN_SCL;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7;
  bus_cfg.flags.enable_internal_pullup = true;
  i2c_master_bus_handle_t bus = NULL;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

  VL53L0X vl(bus);
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
