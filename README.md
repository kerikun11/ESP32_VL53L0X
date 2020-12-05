# ESP32_VL53L0X

VL53L0X driver as ESP-IDF 4.0 component

## Example

```sh
git clone https://github.com/TeschRenan/ESP32_VL53L0X.git
cd ESP32_VL53L0X
idf.py build

```

## Details


***Set the flag for use mode***

```sh
#define REAL_TIME_READ 1

#define DELAY_READ 0

```
Our

```sh
#define REAL_TIME_READ 0

#define DELAY_READ 1

```


## Dependency

- [ESP-IDF 4.0](https://github.com/espressif/esp-idf)

## using API

[STMicroelectronics official VL53L0X API](http://www.st.com/content/st_com/en/products/embedded-software/proximity-sensors-software/stsw-img005.html)

- core
- platform