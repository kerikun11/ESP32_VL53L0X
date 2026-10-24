# ESP32_VL53L0X

C++ VL53L0X driver as an ESP-IDF component

## Dependency

- [ESP-IDF](https://github.com/espressif/esp-idf)

## Usage

The driver uses the ESP-IDF I2C master driver (`driver/i2c_master.h`).
Create the bus in your application and pass its handle to the sensor:

```cpp
i2c_master_bus_config_t bus_cfg = {};
bus_cfg.i2c_port = I2C_NUM_0;
bus_cfg.sda_io_num = GPIO_NUM_21;
bus_cfg.scl_io_num = GPIO_NUM_22;
bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
bus_cfg.glitch_ignore_cnt = 7;
bus_cfg.flags.enable_internal_pullup = true;
i2c_master_bus_handle_t bus = NULL;
ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

VL53L0X vl(bus);
if (vl.init()) {
  uint16_t mm;
  vl.read(&mm);
}
```

## Example

```sh
git clone https://github.com/kerikun11/ESP32_VL53L0X.git
cd ESP32_VL53L0X/examples/polling
idf.py build flash monitor
```

or open `examples/polling` directory with [PlatformIO IDE](https://platformio.org/platformio-ide)

## API using

[STMicroelectronics official VL53L0X API](http://www.st.com/content/st_com/en/products/embedded-software/proximity-sensors-software/stsw-img005.html)

- core
- platform
