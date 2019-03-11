/**
 * @file VL53L0X.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief C++ Library for VL53L0X as an ESP-IDF component
 * @version 0.1
 * @date 2018-11-07
 *
 * @copyright Copyright (c) 2018
 *
 */
#pragma once

#define TAG "VL53L0X"

namespace VL53L0X_config {
# ifdef CONFIG_I2C_PORT_NUM1
    static auto i2c_port = I2C_NUM1;
# else
    static auto i2c_port = I2C_NUM0;
# endif

  static auto sda_gpio = (gpio_num_t) CONFIG_I2C_SDA_GPIO;
  static auto scl_gpio = (gpio_num_t) CONFIG_I2C_SCL_GPIO;

# ifdef CONFIG_XSHUT_ENABLE
  static auto xshut_gpio = (gpio_num_t) CONFIG_XSHUT_GPIO;
# else
  static auto xshut_gpio = GPIO_NUM_MAX;
# endif

# ifdef CONFIG_INT_ENABLE
  static auto gpio1_gpio = (gpio_num_t) CONFIG_GPIO1_GPIO;
# else
  static auto gpio1_gpio = GPIO_NUM_MAX;
# endif
}

