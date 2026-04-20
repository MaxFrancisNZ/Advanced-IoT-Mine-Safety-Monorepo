#include "i2c_manager.h"

#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define I2C_MANAGER_TIMEOUT_MS 100

#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

#define BMP180_REG_CALIB_START 0xAA
#define BMP180_REG_CHIP_ID 0xD0
#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT 0xF6
#define BMP180_CHIP_ID 0x55
#define BMP180_CMD_TEMP 0x2E
#define BMP180_CMD_PRESSURE_BASE 0x34
#define BMP180_PRESSURE_RETRIES 3
#define BMP180_PRESSURE_SAMPLE_COUNT 5

static void sort_int32_values(int32_t *values, size_t count)
{
    for (size_t i = 1; i < count; ++i) {
        int32_t key = values[i];
        size_t j = i;

        while (j > 0 && values[j - 1] > key) {
            values[j] = values[j - 1];
            --j;
        }
        values[j] = key;
    }
}

static esp_err_t bmp180_compensate_pressure(const i2c_manager_bmp180_calibration_t *calibration,
                                            uint8_t oversampling_setting,
                                            int32_t ut,
                                            int32_t up,
                                            int32_t *out_temperature,
                                            int32_t *out_pressure)
{
    int32_t x1 = ((ut - (int32_t)calibration->ac6) * (int32_t)calibration->ac5) >> 15;
    int32_t denominator = x1 + calibration->md;
    if (denominator == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t x2 = ((int32_t)calibration->mc << 11) / denominator;
    int32_t b5 = x1 + x2;
    int32_t temperature = (b5 + 8) >> 4;

    int32_t b6 = b5 - 4000;
    x1 = ((int32_t)calibration->b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = ((int32_t)calibration->ac2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = (((((int32_t)calibration->ac1 * 4) + x3) << oversampling_setting) + 2) >> 2;

    x1 = ((int32_t)calibration->ac3 * b6) >> 13;
    x2 = ((int32_t)calibration->b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = ((uint32_t)calibration->ac4 * (uint32_t)(x3 + 32768)) >> 15;
    if (b4 == 0U) {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t pressure_delta = up - b3;
    if (pressure_delta <= 0) {
        ESP_LOGW("i2c_manager", "BMP180 compensation produced invalid delta: UP=%ld B3=%ld",
                 (long)up, (long)b3);
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t b7 = (uint32_t)pressure_delta * (uint32_t)(50000U >> oversampling_setting);
    int32_t pressure = (b7 < 0x80000000U) ? (int32_t)((b7 << 1) / b4) : (int32_t)((b7 / b4) << 1);

    x1 = (pressure >> 8) * (pressure >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * pressure) >> 16;
    pressure += (x1 + x2 + 3791) >> 4;

    *out_temperature = temperature;
    *out_pressure = pressure;
    return ESP_OK;
}

esp_err_t i2c_manager_init(const i2c_manager_config_t *config, i2c_master_bus_handle_t *out_bus)
{
    if (config == NULL || out_bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = config->port,
        .sda_io_num = config->sda_io_num,
        .scl_io_num = config->scl_io_num,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = config->glitch_ignore_cnt,
        .flags.enable_internal_pullup = config->enable_internal_pullup,
    };

    return i2c_new_master_bus(&bus_config, out_bus);
}

esp_err_t i2c_manager_add_device(i2c_master_bus_handle_t bus, uint16_t device_address, uint32_t scl_speed_hz,
                                 i2c_master_dev_handle_t *out_dev)
{
    if (bus == NULL || out_dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = scl_speed_hz,
    };

    return i2c_master_bus_add_device(bus, &dev_cfg, out_dev);
}

esp_err_t i2c_manager_write_register(i2c_master_dev_handle_t dev, uint8_t reg_addr, uint8_t value)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[2] = {reg_addr, value};
    return i2c_master_transmit(dev, buffer, sizeof(buffer), I2C_MANAGER_TIMEOUT_MS);
}

esp_err_t i2c_manager_read_registers(i2c_master_dev_handle_t dev, uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (dev == NULL || data == NULL || len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(dev, &reg_addr, sizeof(reg_addr), data, len, I2C_MANAGER_TIMEOUT_MS);
}

esp_err_t i2c_manager_probe(i2c_master_bus_handle_t bus, uint16_t device_address, uint32_t scl_speed_hz)
{
    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = i2c_manager_add_device(bus, device_address, scl_speed_hz, &dev);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_master_probe(bus, device_address, 50);
    i2c_master_bus_rm_device(dev);

    return err;
}

void i2c_manager_scan(i2c_master_bus_handle_t bus, uint32_t scl_speed_hz, const char *log_tag)
{
    if (bus == NULL) {
        return;
    }

    for (uint8_t addr = 1; addr < 127; addr++) {
        if (i2c_manager_probe(bus, addr, scl_speed_hz) == ESP_OK) {
            ESP_LOGI(log_tag, "Found I2C device at 0x%02X", addr);
        }
    }
}

static int16_t read_be_i16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint16_t read_be_u16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

esp_err_t i2c_manager_mpu6050_init(i2c_master_bus_handle_t bus, uint16_t device_address, uint32_t scl_speed_hz,
                                   i2c_master_dev_handle_t *out_dev)
{
    if (bus == NULL || out_dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = i2c_manager_add_device(bus, device_address, scl_speed_hz, out_dev);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_manager_write_register(*out_dev, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(*out_dev);
        *out_dev = NULL;
    }

    return err;
}

esp_err_t i2c_manager_mpu6050_read(i2c_master_dev_handle_t dev, i2c_manager_mpu6050_data_t *data)
{
    if (dev == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw_data[14] = {0};
    esp_err_t err = i2c_manager_read_registers(dev, MPU6050_REG_ACCEL_XOUT_H, raw_data, sizeof(raw_data));
    if (err != ESP_OK) {
        return err;
    }

    data->accel_x = read_be_i16(&raw_data[0]);
    data->accel_y = read_be_i16(&raw_data[2]);
    data->accel_z = read_be_i16(&raw_data[4]);
    data->temperature_raw = read_be_i16(&raw_data[6]);
    data->gyro_x = read_be_i16(&raw_data[8]);
    data->gyro_y = read_be_i16(&raw_data[10]);
    data->gyro_z = read_be_i16(&raw_data[12]);
    data->temperature_c = ((float)data->temperature_raw / 340.0f) + 36.53f;

    return ESP_OK;
}

esp_err_t i2c_manager_bmp180_init(i2c_master_bus_handle_t bus, uint16_t device_address, uint32_t scl_speed_hz,
                                  i2c_master_dev_handle_t *out_dev,
                                  i2c_manager_bmp180_calibration_t *out_calibration)
{
    if (bus == NULL || out_dev == NULL || out_calibration == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = i2c_manager_add_device(bus, device_address, scl_speed_hz, out_dev);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t chip_id = 0;
    err = i2c_manager_read_registers(*out_dev, BMP180_REG_CHIP_ID, &chip_id, sizeof(chip_id));
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(*out_dev);
        *out_dev = NULL;
        return err;
    }

    if (chip_id != BMP180_CHIP_ID) {
        ESP_LOGW("i2c_manager", "Unexpected barometer chip ID 0x%02X at 0x%02X, expected BMP180 ID 0x%02X",
                 chip_id, device_address, BMP180_CHIP_ID);
        i2c_master_bus_rm_device(*out_dev);
        *out_dev = NULL;
        return ESP_ERR_NOT_SUPPORTED;
    }

    (void)i2c_manager_write_register(*out_dev, 0xE0, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t calib_raw[22] = {0};
    err = i2c_manager_read_registers(*out_dev, BMP180_REG_CALIB_START, calib_raw, sizeof(calib_raw));
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(*out_dev);
        *out_dev = NULL;
        return err;
    }

    out_calibration->ac1 = read_be_i16(&calib_raw[0]);
    out_calibration->ac2 = read_be_i16(&calib_raw[2]);
    out_calibration->ac3 = read_be_i16(&calib_raw[4]);
    out_calibration->ac4 = read_be_u16(&calib_raw[6]);
    out_calibration->ac5 = read_be_u16(&calib_raw[8]);
    out_calibration->ac6 = read_be_u16(&calib_raw[10]);
    out_calibration->b1 = read_be_i16(&calib_raw[12]);
    out_calibration->b2 = read_be_i16(&calib_raw[14]);
    out_calibration->mb = read_be_i16(&calib_raw[16]);
    out_calibration->mc = read_be_i16(&calib_raw[18]);
    out_calibration->md = read_be_i16(&calib_raw[20]);

    ESP_LOGI("i2c_manager",
             "BMP180 detected at 0x%02X (AC1=%d AC2=%d AC3=%d AC4=%u AC5=%u AC6=%u B1=%d B2=%d MB=%d MC=%d MD=%d)",
             device_address,
             out_calibration->ac1, out_calibration->ac2, out_calibration->ac3,
             out_calibration->ac4, out_calibration->ac5, out_calibration->ac6,
             out_calibration->b1, out_calibration->b2, out_calibration->mb,
             out_calibration->mc, out_calibration->md);

    return ESP_OK;
}

esp_err_t i2c_manager_bmp180_read(i2c_master_dev_handle_t dev,
                                  const i2c_manager_bmp180_calibration_t *calibration,
                                  uint8_t oversampling_setting,
                                  i2c_manager_bmp180_data_t *data)
{
    if (dev == NULL || calibration == NULL || data == NULL || oversampling_setting > 3U) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw_temp[2] = {0};
    uint8_t raw_pressure[3] = {0};
    uint8_t control_reg = 0;
    uint8_t pressure_command = (uint8_t)(BMP180_CMD_PRESSURE_BASE | (oversampling_setting << 6));
    int32_t ut;
    int32_t up;
    int32_t temperature = 0;
    int32_t pressure = 0;
    int32_t pressure_candidates[BMP180_PRESSURE_SAMPLE_COUNT] = {0};
    size_t valid_pressure_count = 0;

    esp_err_t err = i2c_manager_write_register(dev, BMP180_REG_CONTROL, BMP180_CMD_TEMP);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    err = i2c_manager_read_registers(dev, BMP180_REG_RESULT, raw_temp, sizeof(raw_temp));
    if (err != ESP_OK) {
        return err;
    }

    /* Use a more conservative delay than the datasheet minimums while debugging
     * pressure conversion issues on this board. */
    static const uint8_t pressure_delay_ms[] = {8, 12, 18, 30};
    ut = (int32_t)read_be_u16(raw_temp);
    up = ut;

    for (int sample = 0; sample < BMP180_PRESSURE_SAMPLE_COUNT; ++sample) {
        bool sample_valid = false;

        for (int attempt = 0; attempt < BMP180_PRESSURE_RETRIES; ++attempt) {
            err = i2c_manager_write_register(dev, BMP180_REG_CONTROL, pressure_command);
            if (err != ESP_OK) {
                return err;
            }

            err = i2c_manager_read_registers(dev, BMP180_REG_CONTROL, &control_reg, sizeof(control_reg));
            if (err != ESP_OK) {
                return err;
            }

            vTaskDelay(pdMS_TO_TICKS(pressure_delay_ms[oversampling_setting]));

            err = i2c_manager_read_registers(dev, BMP180_REG_RESULT, raw_pressure, sizeof(raw_pressure));
            if (err != ESP_OK) {
                return err;
            }

            up = ((((int32_t)raw_pressure[0] << 16) | ((int32_t)raw_pressure[1] << 8) | raw_pressure[2]) >>
                  (8 - oversampling_setting));

            ESP_LOGI("i2c_manager",
                     "BMP180 raw read sample %d/%d attempt %d/%d: UT=0x%04X (%ld) CTRL=0x%02X UP bytes=[0x%02X 0x%02X 0x%02X] -> %ld (OSS=%u, delay=%ums)",
                     sample + 1, BMP180_PRESSURE_SAMPLE_COUNT,
                     attempt + 1, BMP180_PRESSURE_RETRIES,
                     (unsigned int)ut, (long)ut,
                     control_reg,
                     raw_pressure[0], raw_pressure[1], raw_pressure[2],
                     (long)up, oversampling_setting, pressure_delay_ms[oversampling_setting]);

            if (up == ut) {
                ESP_LOGW("i2c_manager",
                         "BMP180 pressure conversion still suspicious on sample %d/%d attempt %d/%d: UP equals UT (%ld)",
                         sample + 1, BMP180_PRESSURE_SAMPLE_COUNT,
                         attempt + 1, BMP180_PRESSURE_RETRIES, (long)up);
                continue;
            }

            err = bmp180_compensate_pressure(calibration, oversampling_setting, ut, up, &temperature, &pressure);
            if (err != ESP_OK) {
                continue;
            }

            if (pressure < 30000 || pressure > 110000) {
                ESP_LOGW("i2c_manager",
                         "BMP180 rejected out-of-spec pressure candidate on sample %d/%d: %ldPa",
                         sample + 1, BMP180_PRESSURE_SAMPLE_COUNT, (long)pressure);
                continue;
            }

            pressure_candidates[valid_pressure_count++] = pressure;
            sample_valid = true;
            break;
        }

        if (!sample_valid) {
            ESP_LOGW("i2c_manager",
                     "BMP180 failed to collect a valid pressure candidate for sample %d/%d",
                     sample + 1, BMP180_PRESSURE_SAMPLE_COUNT);
        }
    }

    if (valid_pressure_count == 0U) {
        ESP_LOGW("i2c_manager", "BMP180 failed to produce any valid pressure samples.");
        return ESP_ERR_INVALID_STATE;
    }

    err = bmp180_compensate_pressure(calibration, oversampling_setting, ut, up, &temperature, &pressure);
    if (err != ESP_OK) {
        return err;
    }

    sort_int32_values(pressure_candidates, valid_pressure_count);
    pressure = pressure_candidates[valid_pressure_count / 2U];

    if (valid_pressure_count >= 2U) {
        int32_t spread = pressure_candidates[valid_pressure_count - 1U] - pressure_candidates[0];
        if (spread > 8000) {
            ESP_LOGW("i2c_manager",
                     "BMP180 pressure samples are still unstable: min=%ldPa median=%ldPa max=%ldPa spread=%ldPa",
                     (long)pressure_candidates[0],
                     (long)pressure,
                     (long)pressure_candidates[valid_pressure_count - 1U],
                     (long)spread);
        }
    }

    if (pressure < 50000 || pressure > 120000) {
        ESP_LOGW("i2c_manager",
                 "BMP180 produced suspicious pressure: UT=%ld UP=%ld T=%ld.%ldC P=%ldPa",
                 (long)ut, (long)up,
                 (long)(temperature / 10), (long)(temperature >= 0 ? temperature % 10 : -(temperature % 10)),
                 (long)pressure);
    }

    data->temperature_0_1c = temperature;
    data->pressure_pa = pressure;
    data->temperature_c = (float)temperature / 10.0f;

    return ESP_OK;
}
