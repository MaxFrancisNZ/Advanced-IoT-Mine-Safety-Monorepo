#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_port_num_t port;
    gpio_num_t sda_io_num;
    gpio_num_t scl_io_num;
    uint32_t scl_speed_hz;
    bool enable_internal_pullup;
    uint8_t glitch_ignore_cnt;
} i2c_manager_config_t;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature_raw;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    float temperature_c;
} i2c_manager_mpu6050_data_t;

typedef struct {
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} i2c_manager_bmp180_calibration_t;

typedef struct {
    int32_t temperature_0_1c;
    int32_t pressure_pa;
    float temperature_c;
} i2c_manager_bmp180_data_t;

esp_err_t i2c_manager_init(const i2c_manager_config_t *config, i2c_master_bus_handle_t *out_bus);
esp_err_t i2c_manager_add_device(i2c_master_bus_handle_t bus, uint16_t device_address, uint32_t scl_speed_hz,
                                 i2c_master_dev_handle_t *out_dev);
esp_err_t i2c_manager_probe(i2c_master_bus_handle_t bus, uint16_t device_address, uint32_t scl_speed_hz);
void i2c_manager_scan(i2c_master_bus_handle_t bus, uint32_t scl_speed_hz, const char *log_tag);

esp_err_t i2c_manager_write_register(i2c_master_dev_handle_t dev, uint8_t reg_addr, uint8_t value);
esp_err_t i2c_manager_read_registers(i2c_master_dev_handle_t dev, uint8_t reg_addr, uint8_t *data, size_t len);

esp_err_t i2c_manager_mpu6050_init(i2c_master_bus_handle_t bus, uint16_t device_address, uint32_t scl_speed_hz,
                                   i2c_master_dev_handle_t *out_dev);
esp_err_t i2c_manager_mpu6050_read(i2c_master_dev_handle_t dev, i2c_manager_mpu6050_data_t *data);

esp_err_t i2c_manager_bmp180_init(i2c_master_bus_handle_t bus, uint16_t device_address, uint32_t scl_speed_hz,
                                  i2c_master_dev_handle_t *out_dev,
                                  i2c_manager_bmp180_calibration_t *out_calibration);
esp_err_t i2c_manager_bmp180_read(i2c_master_dev_handle_t dev,
                                  const i2c_manager_bmp180_calibration_t *calibration,
                                  uint8_t oversampling_setting,
                                  i2c_manager_bmp180_data_t *data);
