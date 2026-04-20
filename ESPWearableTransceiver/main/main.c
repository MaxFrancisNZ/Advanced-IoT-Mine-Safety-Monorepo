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
#include "battery_probe.h"
#include "mic_manager.h"
#include "adpcm.h"
#include "audio_protocol.h"
#include "ws2812_manager.h"
#include "status_led_manager.h"
#include "web_monitor_manager.h"

/* Hardware configuration */
#define I2C_SDA_GPIO 18
#define I2C_SCL_GPIO 5
#define I2C_PORT_NUM I2C_NUM_0
#define I2C_FREQ_HZ 50000
#define ESPNOW_CHANNEL 1
#define LED_GPIO GPIO_NUM_16
#define BATTERY_SENSE_GPIO GPIO_NUM_35
#define MPU6050_ADDR 0x68
#define BMP180_ADDR 0x77
#define BMP180_OSS 3
#define BMP180_PRESSURE_OFFSET_HPA (-3.0f)

#define HUMITURE_GPIO GPIO_NUM_4
#define HUMITURE_TYPE DHT_MANAGER_TYPE_DHT11

#define MIC_I2S_BCK_GPIO  GPIO_NUM_33
#define MIC_I2S_WS_GPIO   GPIO_NUM_26
#define MIC_I2S_DIN_GPIO  GPIO_NUM_25
#define BUTTON_GPIO       GPIO_NUM_14
#define WS2812_GPIO       GPIO_NUM_12
#define WEB_MONITOR_AP_SSID     "ESPWearableMonitor"
#define WEB_MONITOR_AP_PASSWORD "wearable123"

#define AUDIO_SAMPLE_RATE    8000
#define AUDIO_MAX_SECONDS    5
#define AUDIO_MAX_SAMPLES    (AUDIO_SAMPLE_RATE * AUDIO_MAX_SECONDS)
#define MIC_READ_CHUNK       512

/* Application configuration */
static const char *TAG = "ESPNOW_TX";

/* Global state */
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t mpu6050_dev = NULL;
static i2c_master_dev_handle_t bmp180_dev = NULL;
static i2c_manager_bmp180_calibration_t bmp180_calibration = {0};
static char node_id[18] = {0};
static bool dht_ready = false;
static bool battery_probe_ready = false;
static battery_probe_reading_t last_battery_reading = {0};
static bool battery_reading_valid = false;
static volatile bool audio_active = false;
static uint8_t base_station_mac[6] = {0};


/* Forward declarations */
static void send_task(void *pvParameters);
static void button_task(void *pvParameters);
static void log_sensor_error(const char *sensor_name, esp_err_t err);
static void format_optional_float(char *buffer, size_t buffer_len, bool has_value, float value, uint8_t decimals);
static void format_payload(char *json, size_t json_len, uint32_t transmission_attempts);
static void initialize_sensors(void);
static void log_battery_probe(void);
static void refresh_battery_probe(void);
static void send_audio_chunks(const uint8_t *adpcm_data, size_t adpcm_len, uint32_t total_samples);
static void handle_base_station_message(const uint8_t *mac_addr, const uint8_t *data, int len, void *context);
static bool matches_json_field(const char *json, const char *field_name, const char *field_value);
static void normalize_json_string(const uint8_t *data, int len, char *buffer, size_t buffer_len);

static void log_sensor_error(const char *sensor_name, esp_err_t err)
{
    ESP_LOGW(TAG, "%s unavailable: %s", sensor_name, esp_err_to_name(err));
}

static void normalize_json_string(const uint8_t *data, int len, char *buffer, size_t buffer_len)
{
    size_t out_index = 0;

    if (buffer_len == 0U) {
        return;
    }

    for (int i = 0; i < len && out_index < (buffer_len - 1U); ++i) {
        char ch = (char)data[i];
        if (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') {
            continue;
        }
        buffer[out_index++] = ch;
    }

    buffer[out_index] = '\0';
}

static bool matches_json_field(const char *json, const char *field_name, const char *field_value)
{
    char token[64];

    snprintf(token, sizeof(token), "\"%s\":\"%s\"", field_name, field_value);
    return strstr(json, token) != NULL;
}

static void handle_base_station_message(const uint8_t *mac_addr, const uint8_t *data, int len, void *context)
{
    (void)context;

    char normalized_json[128];

    if (mac_addr == NULL || data == NULL || len <= 0) {
        return;
    }

    if (memcmp(mac_addr, base_station_mac, sizeof(base_station_mac)) != 0) {
        ESP_LOGW(TAG,
                 "Ignoring command from unknown peer %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
        return;
    }

    normalize_json_string(data, len, normalized_json, sizeof(normalized_json));
    if (!matches_json_field(normalized_json, "type", "led_override")) {
        return;
    }

    if (matches_json_field(normalized_json, "mode", "alert_on")) {
        ESP_LOGI(TAG, "Received remote LED alert enable");
        status_led_manager_set_remote_override(true);
        return;
    }

    if (matches_json_field(normalized_json, "mode", "alert_off")) {
        ESP_LOGI(TAG, "Received remote LED alert clear");
        status_led_manager_set_remote_override(false);
        return;
    }

    ESP_LOGW(TAG, "Ignoring unknown LED override command: %s", normalized_json);
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

static void refresh_battery_probe(void)
{
    if (!battery_probe_ready) {
        battery_reading_valid = false;
        return;
    }

    esp_err_t err = battery_probe_read(&last_battery_reading);
    if (err != ESP_OK) {
        battery_reading_valid = false;
        ESP_LOGW(TAG, "Battery probe unavailable: %s", esp_err_to_name(err));
        return;
    }

    battery_reading_valid = last_battery_reading.calibrated;
}

static void log_battery_probe(void)
{
    if (!battery_probe_ready) {
        return;
    }

    refresh_battery_probe();
    if (!battery_reading_valid) {
        ESP_LOGI(TAG, "GPIO35 battery probe: raw=%d battery_voltage=uncalibrated", last_battery_reading.raw);
        return;
    }

    if (last_battery_reading.calibrated) {
        ESP_LOGI(TAG,
                 "GPIO35 battery probe: raw=%d pin_voltage=%dmV battery_voltage=%.3fV heuristic=%s",
                 last_battery_reading.raw,
                 last_battery_reading.pin_millivolts,
                 last_battery_reading.battery_voltage,
                 last_battery_reading.pin_millivolts > 1500 ? "divider_or_battery_present" : "likely_not_divided_or_not_connected");
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
    status_led_inputs_t led_inputs = {0};

    char temperature[16];
    char humidity[16];
    char accel_x[16];
    char accel_y[16];
    char accel_z[16];
    char gyro_x[16];
    char gyro_y[16];
    char gyro_z[16];
    char pressure[16];
    char battery_voltage[16];

    initialize_sensors();
    refresh_battery_probe();

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
    format_optional_float(pressure, sizeof(pressure), has_pressure, ((float)bmp_data.pressure_pa / 100.0f) + BMP180_PRESSURE_OFFSET_HPA, 2);
    format_optional_float(battery_voltage, sizeof(battery_voltage), battery_reading_valid, last_battery_reading.battery_voltage, 3);

    led_inputs.temperature_available = has_temperature;
    led_inputs.temperature_c = dht_err == ESP_OK ? dht_data.temperature_c :
                               (bmp_err == ESP_OK ? bmp_data.temperature_c : mpu_data.temperature_c);
    led_inputs.humidity_available = has_humidity;
    led_inputs.humidity_percent = dht_data.humidity_percent;
    led_inputs.battery_available = battery_reading_valid;
    led_inputs.battery_voltage = last_battery_reading.battery_voltage;
    led_inputs.extreme_acceleration_detected = false;
    led_inputs.prolonged_lying_down_detected = false;
    status_led_manager_update_inputs(&led_inputs);

    snprintf(json, json_len,
             "{"
             "\"node_id\":\"%s\","
             "\"transmission_attempts\":%" PRIu32 ","
             "\"environment\":{"
             "\"battery_voltage\":%s,"
             "\"temperature\":%s,"
             "\"humidity\":%s,"
             "\"accelerometer\":{\"x\":%s,\"y\":%s,\"z\":%s},"
             "\"gyroscope\":{\"x\":%s,\"y\":%s,\"z\":%s},"
             "\"barometric_pressure\":%s"
             "},"
             "\"led_state\":\"%s\","
             "\"led_override_active\":%s"
             "}",
             node_id,
             transmission_attempts,
             battery_voltage,
             temperature,
             humidity,
             accel_x,
             accel_y,
             accel_z,
             gyro_x,
             gyro_y,
             gyro_z,
             pressure,
             status_led_manager_get_render_state_name(),
             status_led_manager_is_remote_override_active() ? "true" : "false");
}

/* Send ADPCM audio as chunked ESP-NOW packets */
static void send_audio_chunks(const uint8_t *adpcm_data, size_t adpcm_len, uint32_t total_samples)
{
    uint8_t wifi_mac[6];
    esp_read_mac(wifi_mac, ESP_MAC_WIFI_STA);

    uint16_t total_chunks = (uint16_t)((adpcm_len + AUDIO_CHUNK_DATA_MAX - 1) / AUDIO_CHUNK_DATA_MAX);

    ESP_LOGI(TAG, "Sending audio: %zu bytes ADPCM, %"PRIu32" samples, %u chunks",
             adpcm_len, total_samples, total_chunks);

    for (uint16_t i = 0; i < total_chunks; i++) {
        uint8_t pkt[ESPNOW_MAX_PAYLOAD];
        audio_chunk_header_t *hdr = (audio_chunk_header_t *)pkt;

        hdr->msg_type      = AUDIO_MSG_TYPE;
        memcpy(hdr->node_mac, wifi_mac, 6);
        hdr->seq_num       = i;
        hdr->total_chunks  = total_chunks;
        hdr->sample_rate   = AUDIO_SAMPLE_RATE;
        hdr->total_samples = total_samples;

        size_t offset    = (size_t)i * AUDIO_CHUNK_DATA_MAX;
        size_t chunk_len = adpcm_len - offset;
        if (chunk_len > AUDIO_CHUNK_DATA_MAX) chunk_len = AUDIO_CHUNK_DATA_MAX;
        hdr->data_len = (uint16_t)chunk_len;

        memcpy(pkt + AUDIO_CHUNK_HDR_SIZE, adpcm_data + offset, chunk_len);

        size_t pkt_len = AUDIO_CHUNK_HDR_SIZE + chunk_len;
        bool delivered = false;
        esp_err_t err = espnow_manager_send_and_wait(pkt, pkt_len, 200, &delivered);
        if (err != ESP_OK || !delivered) {
            ESP_LOGW(TAG, "Audio chunk %u/%u failed", i + 1, total_chunks);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_LOGI(TAG, "Audio transmission complete");
}

/* Push-to-talk button task */
static void button_task(void *pvParameters)
{
    (void)pvParameters;

    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

    int16_t read_buf[MIC_READ_CHUNK];

    while (1) {
        /* Wait for button press (active-low) */
        if (gpio_get_level(BUTTON_GPIO) != 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /* Debounce */
        vTaskDelay(pdMS_TO_TICKS(50));
        if (gpio_get_level(BUTTON_GPIO) != 0) continue;

        /* --- Begin recording --- */
        audio_active = true;
        ESP_LOGI(TAG, "Recording started (hold button, max %d sec)", AUDIO_MAX_SECONDS);

        size_t adpcm_buf_size = (AUDIO_MAX_SAMPLES + 1) / 2;
        uint8_t *adpcm_buf = malloc(adpcm_buf_size);
        if (adpcm_buf == NULL) {
            ESP_LOGE(TAG, "Failed to allocate ADPCM buffer (%zu bytes)", adpcm_buf_size);
            audio_active = false;
            continue;
        }

        adpcm_state_t adpcm_state = {0, 0};
        size_t adpcm_offset = 0;
        size_t total_samples = 0;

        esp_err_t err = mic_manager_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "mic_manager_start failed: %s", esp_err_to_name(err));
            free(adpcm_buf);
            audio_active = false;
            continue;
        }

        /* Record while button is held, up to max duration */
        while (gpio_get_level(BUTTON_GPIO) == 0 && total_samples < AUDIO_MAX_SAMPLES) {
            size_t remaining = AUDIO_MAX_SAMPLES - total_samples;
            size_t to_read = remaining < MIC_READ_CHUNK ? remaining : MIC_READ_CHUNK;
            size_t samples_read = 0;

            err = mic_manager_read(read_buf, to_read, &samples_read, 500);
            if (err != ESP_OK || samples_read == 0) continue;

            size_t encoded = adpcm_encode(read_buf, samples_read,
                                          adpcm_buf + adpcm_offset, &adpcm_state);
            adpcm_offset += encoded;
            total_samples += samples_read;
        }

        mic_manager_stop();

        ESP_LOGI(TAG, "Recording stopped: %zu samples (%.1f sec)",
                 total_samples, (float)total_samples / AUDIO_SAMPLE_RATE);

        /* --- Send compressed audio --- */
        if (total_samples > 0) {
            send_audio_chunks(adpcm_buf, adpcm_offset, (uint32_t)total_samples);
        }

        free(adpcm_buf);
        audio_active = false;

        /* Wait for button release before allowing another recording */
        while (gpio_get_level(BUTTON_GPIO) == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

/* Periodic transmit task */
static void send_task(void *pvParameters)
{
    (void)pvParameters;

    uint32_t failed_transmission_attempts = 0;
    char json[512];

    while (1)
    {
        if (audio_active) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        log_battery_probe();
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
    esp_err_t early_web_monitor_err = web_monitor_manager_early_init();
    if (early_web_monitor_err != ESP_OK) {
        ESP_LOGW(TAG, "Early web monitor capture init failed: %s", esp_err_to_name(early_web_monitor_err));
    }

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
    esp_err_t battery_err = battery_probe_init(BATTERY_SENSE_GPIO);
    if (battery_err != ESP_OK) {
        ESP_LOGW(TAG, "Battery probe init failed on GPIO35: %s", esp_err_to_name(battery_err));
    } else {
        battery_probe_ready = true;
        log_battery_probe();
    }

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
    memcpy(base_station_mac, espnow_config.peer_mac, sizeof(base_station_mac));
    ESP_ERROR_CHECK(espnow_manager_register_recv_cb(handle_base_station_message, NULL));

    const web_monitor_manager_config_t web_monitor_config = {
        .ap_ssid = WEB_MONITOR_AP_SSID,
        .ap_password = WEB_MONITOR_AP_PASSWORD,
        .channel = ESPNOW_CHANNEL,
        .max_connections = 1,
    };
    esp_err_t web_monitor_err = web_monitor_manager_init(&web_monitor_config);
    if (web_monitor_err != ESP_OK) {
        ESP_LOGW(TAG, "Web monitor init failed: %s", esp_err_to_name(web_monitor_err));
    }

    uint8_t wifi_sta_mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(wifi_sta_mac, ESP_MAC_WIFI_STA));
    snprintf(node_id, sizeof(node_id), "%02X:%02X:%02X:%02X:%02X:%02X",
             wifi_sta_mac[0], wifi_sta_mac[1], wifi_sta_mac[2],
             wifi_sta_mac[3], wifi_sta_mac[4], wifi_sta_mac[5]);

    /* Initialise MEMS microphone (I2S) */
    const mic_manager_config_t mic_config = {
        .bck_gpio    = MIC_I2S_BCK_GPIO,
        .ws_gpio     = MIC_I2S_WS_GPIO,
        .din_gpio    = MIC_I2S_DIN_GPIO,
        .sample_rate = AUDIO_SAMPLE_RATE,
    };
    esp_err_t mic_err = mic_manager_init(&mic_config);
    if (mic_err != ESP_OK) {
        ESP_LOGE(TAG, "Microphone init failed: %s", esp_err_to_name(mic_err));
    }

    esp_err_t ws_err = ws2812_manager_init(WS2812_GPIO);
    if (ws_err != ESP_OK) {
        ESP_LOGW(TAG, "WS2812 init failed during boot test: %s", esp_err_to_name(ws_err));
    } else {
        esp_err_t ws_color_err = ws2812_manager_set_color(0, 0, 30);
        if (ws_color_err != ESP_OK) {
            ESP_LOGW(TAG, "WS2812 blue boot test failed: %s", esp_err_to_name(ws_color_err));
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    esp_err_t status_led_err = status_led_manager_init(WS2812_GPIO);
    if (status_led_err != ESP_OK) {
        ESP_LOGW(TAG, "Status LED init failed: %s", esp_err_to_name(status_led_err));
    }

    xTaskCreate(send_task, "send_task", 4096, NULL, 4, NULL);
    xTaskCreate(button_task, "button_task", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "Transmitter ready");
    web_monitor_manager_finalize_startup_history();
}
