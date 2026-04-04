#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "i2c_manager.h"
#include "espnow_manager.h"
#include "led_manager.h"
#include "dht_manager.h"

/* Hardware configuration */
#define I2C_SDA_GPIO 21
#define I2C_SCL_GPIO 22
#define I2C_PORT_NUM I2C_NUM_0
#define I2C_FREQ_HZ 100000
#define ESPNOW_CHANNEL 1
#define LED_GPIO GPIO_NUM_16
#define MPU6050_ADDR 0x68
#define BMP180_ADDR 0x77
#define BMP180_OSS 0

#define HUMITURE_GPIO GPIO_NUM_4
#define HUMITURE_TYPE DHT_MANAGER_TYPE_DHT11

/* Application configuration */
static const char *TAG = "ESPNOW_TX";

/* Global state */
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t mpu6050_dev = NULL;
static i2c_master_dev_handle_t bmp180_dev = NULL;
static i2c_manager_bmp180_calibration_t bmp180_calibration = {0};
static char node_id[18] = {0};
static bool dht_ready = false;


/* Forward declarations */
static void send_task(void *pvParameters);
static void log_sensor_error(const char *sensor_name, esp_err_t err);
static void format_optional_float(char *buffer, size_t buffer_len, bool has_value, float value, uint8_t decimals);
static void format_payload(char *json, size_t json_len, uint32_t transmission_attempts);
static void initialize_sensors(void);

static void log_sensor_error(const char *sensor_name, esp_err_t err)
{
    ESP_LOGW(TAG, "%s unavailable: %s", sensor_name, esp_err_to_name(err));
}

static void format_optional_float(char *buffer, size_t buffer_len, bool has_value, float value, uint8_t decimals)
{
    if (!has_value) {
        snprintf(buffer, buffer_len, "null");
        return;
    }

    snprintf(buffer, buffer_len, "%.*f", decimals, value);
}

static void initialize_sensors(void)
{
    esp_err_t err = ESP_OK;

    if (mpu6050_dev == NULL) {
        err = i2c_manager_mpu6050_init(i2c_bus, MPU6050_ADDR, I2C_FREQ_HZ, &mpu6050_dev);
        if (err != ESP_OK) {
            log_sensor_error("MPU6050", err);
        }
    }

    if (bmp180_dev == NULL) {
        err = i2c_manager_bmp180_init(i2c_bus, BMP180_ADDR, I2C_FREQ_HZ, &bmp180_dev, &bmp180_calibration);
        if (err != ESP_OK) {
            log_sensor_error("BMP180", err);
        }
    }
}

static void format_payload(char *json, size_t json_len, uint32_t transmission_attempts)
{
    i2c_manager_mpu6050_data_t mpu_data = {0};
    i2c_manager_bmp180_data_t bmp_data = {0};
    dht_manager_data_t dht_data = {0};
    esp_err_t dht_err = ESP_FAIL;
    esp_err_t mpu_err = ESP_FAIL;
    esp_err_t bmp_err = ESP_FAIL;

    bool has_temperature = false;
    bool has_humidity = false;
    bool has_accel = false;
    bool has_gyro = false;
    bool has_pressure = false;

    char temperature[16];
    char humidity[16];
    char accel_x[16];
    char accel_y[16];
    char accel_z[16];
    char gyro_x[16];
    char gyro_y[16];
    char gyro_z[16];
    char pressure[16];

    initialize_sensors();

    if (dht_ready) {
        dht_err = dht_manager_read(&dht_data);
        if (dht_err == ESP_OK) {
            has_temperature = true;
            has_humidity = true;
        } else {
            log_sensor_error("DHT", dht_err);
        }
    }

    if (mpu6050_dev != NULL) {
        mpu_err = i2c_manager_mpu6050_read(mpu6050_dev, &mpu_data);
        if (mpu_err == ESP_OK) {
            has_temperature = true;
            has_accel = true;
            has_gyro = true;
        } else {
            log_sensor_error("MPU6050", mpu_err);
        }
    }

    if (bmp180_dev != NULL) {
        bmp_err = i2c_manager_bmp180_read(bmp180_dev, &bmp180_calibration, BMP180_OSS, &bmp_data);
        if (bmp_err == ESP_OK) {
            has_temperature = true;
            has_pressure = true;
        } else {
            log_sensor_error("BMP180", bmp_err);
        }
    }

    format_optional_float(temperature, sizeof(temperature), has_temperature,
                          dht_err == ESP_OK ? dht_data.temperature_c :
                          (bmp_err == ESP_OK ? bmp_data.temperature_c : mpu_data.temperature_c), 2);
    format_optional_float(humidity, sizeof(humidity), has_humidity, dht_data.humidity_percent, 2);
    format_optional_float(accel_x, sizeof(accel_x), has_accel, (float)mpu_data.accel_x / 16384.0f, 3);
    format_optional_float(accel_y, sizeof(accel_y), has_accel, (float)mpu_data.accel_y / 16384.0f, 3);
    format_optional_float(accel_z, sizeof(accel_z), has_accel, (float)mpu_data.accel_z / 16384.0f, 3);
    format_optional_float(gyro_x, sizeof(gyro_x), has_gyro, (float)mpu_data.gyro_x / 131.0f, 3);
    format_optional_float(gyro_y, sizeof(gyro_y), has_gyro, (float)mpu_data.gyro_y / 131.0f, 3);
    format_optional_float(gyro_z, sizeof(gyro_z), has_gyro, (float)mpu_data.gyro_z / 131.0f, 3);
    format_optional_float(pressure, sizeof(pressure), has_pressure, (float)bmp_data.pressure_pa / 100.0f, 2);

    snprintf(json, json_len,
             "{"
             "\"node_id\":\"%s\","
             "\"transmission_attempts\":%" PRIu32 ","
             "\"environment\":{"
             "\"temperature\":%s,"
             "\"humidity\":%s,"
             "\"accelerometer\":{\"x\":%s,\"y\":%s,\"z\":%s},"
             "\"gyroscope\":{\"x\":%s,\"y\":%s,\"z\":%s},"
             "\"barometric_pressure\":%s"
             "},"
             "\"led_state\":\"%s\""
             "}",
             node_id,
             transmission_attempts,
             temperature,
             humidity,
             accel_x,
             accel_y,
             accel_z,
             gyro_x,
             gyro_y,
             gyro_z,
             pressure,
             led_manager_is_on() ? "#00FF00" : "#000000");
}

/* Periodic transmit task */
static void send_task(void *pvParameters)
{
    (void)pvParameters;

    uint32_t failed_transmission_attempts = 0;
    char json[512];

    while (1)
    {
        format_payload(json, sizeof(json), failed_transmission_attempts);

        bool delivered = false;
        esp_err_t err = espnow_manager_send_and_wait((const uint8_t *)json, strlen(json), 1000, &delivered);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "espnow_manager_send failed: %s", esp_err_to_name(err));
            failed_transmission_attempts++;
        }
        else if (!delivered)
        {
            ESP_LOGW(TAG, "ESP-NOW delivery failed");
            failed_transmission_attempts++;
        }
        else
        {
            ESP_LOGI(TAG, "Sent JSON: %s", json);
            failed_transmission_attempts = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* Application entry point */
void app_main(void)
{
    const i2c_manager_config_t i2c_config = {
        .port = I2C_PORT_NUM,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .scl_speed_hz = I2C_FREQ_HZ,
        .enable_internal_pullup = true,
        .glitch_ignore_cnt = 7,
    };
    const espnow_manager_config_t espnow_config = {
        .peer_mac = {0x5c, 0x01, 0x3b, 0x8a, 0xdb, 0xbc},
        .channel = ESPNOW_CHANNEL,
        .encrypt = false,
        .log_tag = TAG,
    };

    ESP_ERROR_CHECK(led_manager_init(LED_GPIO, true));
    esp_err_t dht_err = dht_manager_init(HUMITURE_GPIO, HUMITURE_TYPE);
    if (dht_err != ESP_OK) {
        log_sensor_error("DHT", dht_err);
    } else {
        dht_ready = true;
    }

    ESP_ERROR_CHECK(i2c_manager_init(&i2c_config, &i2c_bus));
    i2c_manager_scan(i2c_bus, i2c_config.scl_speed_hz, TAG);
    initialize_sensors();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(espnow_manager_init(&espnow_config));
    uint8_t wifi_sta_mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(wifi_sta_mac, ESP_MAC_WIFI_STA));
    snprintf(node_id, sizeof(node_id), "%02X:%02X:%02X:%02X:%02X:%02X",
             wifi_sta_mac[0], wifi_sta_mac[1], wifi_sta_mac[2],
             wifi_sta_mac[3], wifi_sta_mac[4], wifi_sta_mac[5]);

    xTaskCreate(send_task, "send_task", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "Transmitter ready");
}
