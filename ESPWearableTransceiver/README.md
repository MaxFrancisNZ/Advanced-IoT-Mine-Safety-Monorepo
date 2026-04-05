# ESP Wearable Transceiver

An ESP-IDF firmware for an ESP32-based wearable sensor node used in an IoT mine safety system. The device reads environmental and motion data from multiple sensors and transmits JSON payloads to a base station receiver via ESP-NOW every 5 seconds.

## Overview

The wearable node collects data from:
- **MPU6050** — 3-axis accelerometer + 3-axis gyroscope (I2C, 0x68)
- **BMP180** — barometric pressure and temperature sensor (I2C, 0x77)
- **DHT11** — ambient temperature and relative humidity (1-wire)
- **Battery probe** — ADC-based battery voltage measurement via a resistor divider (GPIO35)

Data is transmitted to a paired receiver (base station) using ESP-NOW. Each transmission includes the node's MAC address as a unique identifier. Failed deliveries are counted and included in subsequent payloads. An LED provides a visual "alive" indicator.

Sensors are optional — if a sensor is unavailable, its fields are transmitted as `null` rather than causing a fatal error.

---

## Hardware Pin Mapping

| Signal          | GPIO  | Notes                              |
|-----------------|-------|------------------------------------|
| I2C SDA         | 21    | MPU6050 + BMP180                   |
| I2C SCL         | 22    | MPU6050 + BMP180                   |
| DHT11 data      | 4     | 1-wire, internal pull-up enabled   |
| LED             | 16    | Active-low                         |
| Battery sense   | 35    | ADC1 CH7, assumes 2:1 divider      |

---

## JSON Payload Format

A payload is transmitted every 5 seconds. Fields for unavailable sensors are `null`.

```json
{
  "node_id": "AA:BB:CC:DD:EE:FF",
  "transmission_attempts": 0,
  "environment": {
    "battery_voltage": 3.742,
    "temperature": 24.50,
    "humidity": 58.00,
    "accelerometer": { "x": 0.012, "y": -0.004, "z": 1.001 },
    "gyroscope": { "x": 0.015, "y": -0.008, "z": 0.002 },
    "barometric_pressure": 1013.25
  },
  "led_state": "#00FF00"
}
```

| Field                        | Unit / Type  | Source                              |
|------------------------------|--------------|-------------------------------------|
| `node_id`                    | MAC string   | Wi-Fi STA MAC address               |
| `transmission_attempts`      | integer      | Failed delivery count since last success |
| `battery_voltage`            | Volts        | Battery probe (ADC, 2:1 divider), `null` if uncalibrated |
| `temperature`                | °C           | DHT11 preferred; BMP180 or MPU6050 fallback |
| `humidity`                   | %RH          | DHT11                               |
| `accelerometer.x/y/z`        | g            | MPU6050 (±2g range, 16384 LSB/g)   |
| `gyroscope.x/y/z`            | °/s          | MPU6050 (±250°/s range, 131 LSB/°/s) |
| `barometric_pressure`        | hPa          | BMP180 (OSS=0)                      |
| `led_state`                  | hex color    | `#00FF00` = on, `#000000` = off     |

---

## Project Structure

```
ESPWearableTransceiver/
├── main/
│   └── main.c                  # App entry point, send task, payload formatting
└── components/
    ├── battery_probe/          # ADC battery voltage measurement
    ├── dht_manager/            # DHT11/DHT22 1-wire driver
    ├── espnow_manager/         # ESP-NOW transmit wrapper
    ├── i2c_manager/            # I2C bus + MPU6050 + BMP180 drivers
    └── led_manager/            # GPIO LED control
```

---

## Components

### `battery_probe`

ADC-based battery voltage sensor. Assumes a 2:1 resistor divider between the battery and the sense pin, so the battery voltage is calculated as `pin_voltage × 2`. Supports ESP32 ADC1 input-only pins (GPIO32–39).

Uses ESP-IDF's ADC oneshot driver with 12 dB attenuation (0–3.9V input range). Calibration is applied automatically using curve-fitting or line-fitting schemes if eFuse calibration data is available.

**Supported GPIOs:** 32, 33, 34, 35, 36, 39

#### API

```c
esp_err_t battery_probe_init(gpio_num_t gpio_num);
esp_err_t battery_probe_read(battery_probe_reading_t *out_reading);
```

#### `battery_probe_reading_t`

| Field               | Type    | Description                                      |
|---------------------|---------|--------------------------------------------------|
| `raw`               | `int`   | Raw ADC count                                    |
| `pin_millivolts`    | `int`   | Calibrated voltage at the sense pin (mV)         |
| `battery_millivolts`| `int`   | Estimated battery voltage (mV), `pin_mv × 2`    |
| `battery_voltage`   | `float` | Estimated battery voltage (V)                    |
| `calibrated`        | `bool`  | `true` if eFuse calibration was applied          |

If `calibrated` is `false`, only `raw` is valid; voltage fields are zero.

---

### `dht_manager`

Bit-banged single-wire driver for DHT11 and DHT22 humidity/temperature sensors. Uses open-drain GPIO with internal pull-up. Timing is measured with `esp_timer_get_time()` at microsecond resolution.

A bit is decoded as `1` if its high-pulse duration exceeds 40 µs (DHT protocol threshold). A 5-byte checksum is validated before data is returned.

**DHT11** returns integer-resolution values (±1°C, ±1%RH).  
**DHT22** returns 0.1-resolution values decoded from a 16-bit fixed-point word.

#### API

```c
esp_err_t dht_manager_init(gpio_num_t gpio_num, dht_manager_type_t sensor_type);
esp_err_t dht_manager_read(dht_manager_data_t *out_data);
```

#### `dht_manager_type_t`

| Value                      | Sensor  |
|----------------------------|---------|
| `DHT_MANAGER_TYPE_DHT11`   | DHT11   |
| `DHT_MANAGER_TYPE_DHT22`   | DHT22   |

#### `dht_manager_data_t`

| Field               | Type    | Description           |
|---------------------|---------|-----------------------|
| `temperature_c`     | `float` | Temperature in °C     |
| `humidity_percent`  | `float` | Relative humidity %RH |

Returns `ESP_ERR_INVALID_CRC` if the checksum fails.

---

### `espnow_manager`

Thin wrapper around the ESP-IDF ESP-NOW API for unicast transmission to a single peer. Initialises Wi-Fi in STA mode on the configured channel. Uses a binary semaphore to block on the send callback, allowing `send_and_wait` to confirm delivery before returning.

The send callback updates a module-level status variable and releases the semaphore. The caller inspects `*out_delivered` to distinguish between a transport error and a MAC-layer delivery failure (no ACK).

#### API

```c
esp_err_t espnow_manager_init(const espnow_manager_config_t *config);
esp_err_t espnow_manager_send(const uint8_t *data, size_t len);
esp_err_t espnow_manager_send_and_wait(const uint8_t *data, size_t len,
                                       uint32_t timeout_ms, bool *out_delivered);
```

#### `espnow_manager_config_t`

| Field        | Type           | Description                                  |
|--------------|----------------|----------------------------------------------|
| `peer_mac`   | `uint8_t[6]`   | MAC address of the receiver (base station)   |
| `channel`    | `uint8_t`      | Wi-Fi channel (must match receiver)          |
| `encrypt`    | `bool`         | Enable ESP-NOW encryption                    |
| `log_tag`    | `const char *` | Tag used for ESP log output                  |

#### `espnow_manager_send_and_wait`

Sends data and blocks until the send callback fires or `timeout_ms` elapses.

| `err` return      | `*out_delivered` | Meaning                         |
|-------------------|------------------|---------------------------------|
| `ESP_OK`          | `true`           | Delivery confirmed              |
| `ESP_OK`          | `false`          | Sent, but no ACK received       |
| `ESP_ERR_TIMEOUT` | `false`          | Callback did not fire in time   |
| Other             | `false`          | `esp_now_send` failed           |

---

### `i2c_manager`

I2C bus initialisation and low-level read/write helpers, plus higher-level drivers for the MPU6050 and BMP180.

#### Bus API

```c
esp_err_t i2c_manager_init(const i2c_manager_config_t *config, i2c_master_bus_handle_t *out_bus);
esp_err_t i2c_manager_add_device(i2c_master_bus_handle_t bus, uint16_t device_address,
                                  uint32_t scl_speed_hz, i2c_master_dev_handle_t *out_dev);
esp_err_t i2c_manager_probe(i2c_master_bus_handle_t bus, uint16_t device_address,
                             uint32_t scl_speed_hz);
void      i2c_manager_scan(i2c_master_bus_handle_t bus, uint32_t scl_speed_hz,
                            const char *log_tag);
esp_err_t i2c_manager_write_register(i2c_master_dev_handle_t dev, uint8_t reg_addr,
                                      uint8_t value);
esp_err_t i2c_manager_read_registers(i2c_master_dev_handle_t dev, uint8_t reg_addr,
                                      uint8_t *data, size_t len);
```

`i2c_manager_scan` logs all responding addresses between 0x01 and 0x7E. Useful for verifying sensor connections on startup.

#### `i2c_manager_config_t`

| Field                  | Type              | Description                            |
|------------------------|-------------------|----------------------------------------|
| `port`                 | `i2c_port_num_t`  | I2C peripheral number (`I2C_NUM_0`)    |
| `sda_io_num`           | `gpio_num_t`      | SDA GPIO                               |
| `scl_io_num`           | `gpio_num_t`      | SCL GPIO                               |
| `scl_speed_hz`         | `uint32_t`        | Clock frequency (e.g. 100000)          |
| `enable_internal_pullup` | `bool`          | Enable ESP32 internal pull-ups         |
| `glitch_ignore_cnt`    | `uint8_t`         | Glitch filter count (typical: 7)       |

#### MPU6050 API

```c
esp_err_t i2c_manager_mpu6050_init(i2c_master_bus_handle_t bus, uint16_t device_address,
                                    uint32_t scl_speed_hz, i2c_master_dev_handle_t *out_dev);
esp_err_t i2c_manager_mpu6050_read(i2c_master_dev_handle_t dev,
                                    i2c_manager_mpu6050_data_t *data);
```

Init wakes the device by clearing `PWR_MGMT_1`. Read fetches all 14 bytes from `ACCEL_XOUT_H` in a single burst.

**Conversion factors (default FS ranges):**
- Accelerometer: divide raw value by **16384** to get g (±2g range)
- Gyroscope: divide raw value by **131** to get °/s (±250°/s range)
- Temperature: `raw / 340.0 + 36.53` °C

#### `i2c_manager_mpu6050_data_t`

| Field             | Type      | Description                     |
|-------------------|-----------|---------------------------------|
| `accel_x/y/z`     | `int16_t` | Raw accelerometer counts        |
| `gyro_x/y/z`      | `int16_t` | Raw gyroscope counts            |
| `temperature_raw` | `int16_t` | Raw temperature register value  |
| `temperature_c`   | `float`   | Temperature in °C (converted)   |

#### BMP180 API

```c
esp_err_t i2c_manager_bmp180_init(i2c_master_bus_handle_t bus, uint16_t device_address,
                                   uint32_t scl_speed_hz, i2c_master_dev_handle_t *out_dev,
                                   i2c_manager_bmp180_calibration_t *out_calibration);
esp_err_t i2c_manager_bmp180_read(i2c_master_dev_handle_t dev,
                                   const i2c_manager_bmp180_calibration_t *calibration,
                                   uint8_t oversampling_setting,
                                   i2c_manager_bmp180_data_t *data);
```

Init reads all 11 factory calibration coefficients (22 bytes from `0xAA`). Read performs a temperature conversion followed by a pressure conversion using the Bosch datasheet compensation algorithm. The `oversampling_setting` (OSS) controls pressure measurement duration and resolution:

| OSS | Samples | Conversion time |
|-----|---------|-----------------|
| 0   | 1       | 5 ms            |
| 1   | 2       | 8 ms            |
| 2   | 4       | 14 ms           |
| 3   | 8       | 26 ms           |

The app uses OSS=0. Pressure is output in Pascal; the payload divides by 100 to convert to hPa.

#### `i2c_manager_bmp180_data_t`

| Field              | Type      | Description                        |
|--------------------|-----------|------------------------------------|
| `temperature_0_1c` | `int32_t` | Temperature in units of 0.1°C      |
| `temperature_c`    | `float`   | Temperature in °C                  |
| `pressure_pa`      | `int32_t` | Compensated pressure in Pascal     |

---

### `led_manager`

Simple GPIO output driver for a single LED. Supports active-high and active-low configurations. The LED is turned on during `init` and its logical state is tracked so it can be reported in the transmitted payload.

#### API

```c
esp_err_t led_manager_init(gpio_num_t gpio_num, bool active_low);
esp_err_t led_manager_set(bool on);
bool      led_manager_is_on(void);
```

| Parameter     | Description                                        |
|---------------|----------------------------------------------------|
| `active_low`  | `true` if the LED anode is tied to VCC (common in ESP32 dev boards) |

---

## Build & Flash

### Prerequisites

- [ESP-IDF v5.x](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/)
- Target: ESP32

### Build

```bash
idf.py build
```

### Flash

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

### Dev Container

A `.devcontainer` configuration is included for VS Code. It installs ESP-IDF and the Espressif extensions automatically. Open in a dev container and use the ESP-IDF extension commands or the terminal to build and flash.

---

## Configuration

All hardware assignments and application parameters are defined as macros at the top of [main/main.c](main/main.c):

| Macro              | Default         | Description                        |
|--------------------|-----------------|------------------------------------|
| `I2C_SDA_GPIO`     | 21              | I2C SDA pin                        |
| `I2C_SCL_GPIO`     | 22              | I2C SCL pin                        |
| `I2C_PORT_NUM`     | `I2C_NUM_0`     | I2C peripheral                     |
| `I2C_FREQ_HZ`      | 100000          | I2C clock speed                    |
| `ESPNOW_CHANNEL`   | 1               | Wi-Fi channel for ESP-NOW          |
| `LED_GPIO`         | 16              | Status LED GPIO                    |
| `BATTERY_SENSE_GPIO` | 35            | ADC sense pin for battery          |
| `MPU6050_ADDR`     | 0x68            | MPU6050 I2C address                |
| `BMP180_ADDR`      | 0x77            | BMP180 I2C address                 |
| `BMP180_OSS`       | 0               | Oversampling setting (0–3)         |
| `HUMITURE_GPIO`    | 4               | DHT11 data pin                     |
| `HUMITURE_TYPE`    | `DHT_MANAGER_TYPE_DHT11` | Sensor variant          |

The receiver MAC address is set in `app_main` inside the `espnow_config` initialiser.
