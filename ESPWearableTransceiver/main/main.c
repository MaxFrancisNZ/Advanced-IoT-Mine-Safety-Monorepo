#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_spiffs.h"

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
#define JSON_PAYLOAD_MAX_LEN 512
#define STORAGE_BASE_PATH    "/spiffs"
#define STORAGE_SCAN_PATH    STORAGE_BASE_PATH "/"
#define STORAGE_PATH_MAX     96
#define STORAGE_READ_BUFFER_MAX ESPNOW_MAX_PAYLOAD
#define QUEUE_FLUSH_DELAY_MS 50
#define QUEUE_RETRY_DELAY_MS 250

/* Application configuration */
#define SEND_INTERVAL_MS 5000
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
static uint32_t s_next_sensor_seq = 0;
static uint32_t s_next_audio_seq = 0;
static uint32_t s_sensor_sample_sequence = 0;

static volatile bool       s_time_synced     = false;
static volatile int64_t    s_sync_epoch_ms   = 0;
static volatile TickType_t s_sync_local_tick = 0;
static portMUX_TYPE        s_time_sync_mux   = portMUX_INITIALIZER_UNLOCKED;

static volatile bool s_offline_mode = false;

#define OFFLINE_MAX_AUDIO_RECORDINGS 5U
#define OFFLINE_NVS_NAMESPACE "wearable"
#define OFFLINE_NVS_KEY       "offline"

typedef enum {
    QUEUE_KIND_SENSOR = 0,
    QUEUE_KIND_AUDIO,
} queue_kind_t;


/* Forward declarations */
static void send_task(void *pvParameters);
static void button_task(void *pvParameters);
static void log_sensor_error(const char *sensor_name, esp_err_t err);
static void format_optional_float(char *buffer, size_t buffer_len, bool has_value, float value, uint8_t decimals);
static void format_payload(char *json, size_t json_len, uint32_t transmission_attempts, uint32_t sample_sequence);
static void initialize_sensors(void);
static void log_battery_probe(void);
static void refresh_battery_probe(void);
static void send_audio_chunks(const uint8_t *adpcm_data, size_t adpcm_len, uint32_t total_samples);
static void handle_base_station_message(const uint8_t *mac_addr, const uint8_t *data, int len, void *context);
static bool matches_json_field(const char *json, const char *field_name, const char *field_value);
static bool parse_json_int64_field(const char *json, const char *field_name, int64_t *out_value);
static bool get_synced_epoch_ms(int64_t *out_ms);
static void normalize_json_string(const uint8_t *data, int len, char *buffer, size_t buffer_len);
static const char *queue_kind_prefix(queue_kind_t kind);
static uint32_t *queue_kind_next_seq(queue_kind_t kind);
static esp_err_t init_persistent_queue(void);
static esp_err_t refresh_queue_sequence_counters(void);
static esp_err_t enqueue_packet(queue_kind_t kind, const uint8_t *data, size_t len);
static bool find_oldest_packet_path(queue_kind_t kind, char *path, size_t path_len, uint32_t *out_seq);
static size_t count_queued_packets(queue_kind_t kind);
static bool flush_persistent_queues(uint32_t *failed_transmission_attempts);
static bool nvs_load_offline_flag(void);
static void nvs_store_offline_flag(bool offline);
static void button_gpio_init(void);
static bool button_held_at_boot(void);
static bool probe_basestation_for(uint32_t timeout_ms);
static size_t count_queued_audio_recordings(void);
static esp_err_t drop_oldest_audio_recording(void);

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

static const char *queue_kind_prefix(queue_kind_t kind)
{
    return kind == QUEUE_KIND_AUDIO ? "audio_" : "sensor_";
}

static uint32_t *queue_kind_next_seq(queue_kind_t kind)
{
    return kind == QUEUE_KIND_AUDIO ? &s_next_audio_seq : &s_next_sensor_seq;
}

static esp_err_t init_persistent_queue(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = STORAGE_BASE_PATH,
        .partition_label = "storage",
        .max_files = 6,
        .format_if_mount_failed = true,
    };

    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS queue storage: %s", esp_err_to_name(err));
        return err;
    }

    size_t total_bytes = 0;
    size_t used_bytes = 0;
    err = esp_spiffs_info(conf.partition_label, &total_bytes, &used_bytes);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Queue storage mounted: %u / %u bytes used",
                 (unsigned)used_bytes, (unsigned)total_bytes);
    } else {
        ESP_LOGW(TAG, "Failed to query queue storage usage: %s", esp_err_to_name(err));
    }

    return refresh_queue_sequence_counters();
}

static esp_err_t refresh_queue_sequence_counters(void)
{
    DIR *dir = opendir(STORAGE_SCAN_PATH);
    struct dirent *entry = NULL;
    uint32_t max_sensor_seq = 0;
    uint32_t max_audio_seq = 0;
    bool sensor_found = false;
    bool audio_found = false;

    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open queue storage directory: errno=%d", errno);
        return ESP_FAIL;
    }

    while ((entry = readdir(dir)) != NULL) {
        uint32_t seq = 0;

        if (sscanf(entry->d_name, "sensor_%" SCNu32 ".bin", &seq) == 1) {
            if (!sensor_found || seq > max_sensor_seq) {
                max_sensor_seq = seq;
                sensor_found = true;
            }
            continue;
        }

        if (sscanf(entry->d_name, "audio_%" SCNu32 ".bin", &seq) == 1) {
            if (!audio_found || seq > max_audio_seq) {
                max_audio_seq = seq;
                audio_found = true;
            }
        }
    }

    closedir(dir);
    s_next_sensor_seq = sensor_found ? (max_sensor_seq + 1U) : 0U;
    s_next_audio_seq = audio_found ? (max_audio_seq + 1U) : 0U;

    ESP_LOGI(TAG, "Recovered queued packets: %u audio, %u sensor",
             (unsigned)count_queued_packets(QUEUE_KIND_AUDIO),
             (unsigned)count_queued_packets(QUEUE_KIND_SENSOR));
    return ESP_OK;
}

static esp_err_t enqueue_packet(queue_kind_t kind, const uint8_t *data, size_t len)
{
    char path[STORAGE_PATH_MAX];
    FILE *file = NULL;
    uint32_t seq = 0;
    size_t written = 0;

    if (data == NULL || len == 0U || len > STORAGE_READ_BUFFER_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    seq = *queue_kind_next_seq(kind);
    snprintf(path, sizeof(path), STORAGE_BASE_PATH "/%s%" PRIu32 ".bin", queue_kind_prefix(kind), seq);

    file = fopen(path, "wb");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to create queue file %s: errno=%d", path, errno);
        return ESP_FAIL;
    }

    written = fwrite(data, 1, len, file);
    if (written != len) {
        ESP_LOGE(TAG, "Short write to %s (%u/%u bytes)", path, (unsigned)written, (unsigned)len);
        fclose(file);
        unlink(path);
        return ESP_FAIL;
    }

    if (fclose(file) != 0) {
        ESP_LOGE(TAG, "Failed to close queue file %s: errno=%d", path, errno);
        unlink(path);
        return ESP_FAIL;
    }

    (*queue_kind_next_seq(kind))++;
    ESP_LOGI(TAG, "Queued %s packet %" PRIu32 " (%u bytes)",
             kind == QUEUE_KIND_AUDIO ? "audio" : "sensor",
             seq,
             (unsigned)len);
    return ESP_OK;
}

static bool find_oldest_packet_path(queue_kind_t kind, char *path, size_t path_len, uint32_t *out_seq)
{
    DIR *dir = NULL;
    struct dirent *entry = NULL;
    uint32_t oldest_seq = UINT32_MAX;
    bool found = false;
    char pattern[32];

    if (path == NULL || path_len == 0U) {
        return false;
    }

    snprintf(pattern, sizeof(pattern), "%s%%" SCNu32 ".bin", queue_kind_prefix(kind));
    dir = opendir(STORAGE_SCAN_PATH);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open queue directory: errno=%d", errno);
        return false;
    }

    while ((entry = readdir(dir)) != NULL) {
        uint32_t seq = 0;

        if (sscanf(entry->d_name, pattern, &seq) == 1 && seq < oldest_seq) {
            oldest_seq = seq;
            found = true;
        }
    }

    closedir(dir);
    if (!found) {
        return false;
    }

    snprintf(path, path_len, STORAGE_BASE_PATH "/%s%" PRIu32 ".bin", queue_kind_prefix(kind), oldest_seq);
    if (out_seq != NULL) {
        *out_seq = oldest_seq;
    }
    return true;
}

static size_t count_queued_packets(queue_kind_t kind)
{
    DIR *dir = opendir(STORAGE_SCAN_PATH);
    struct dirent *entry = NULL;
    size_t count = 0;
    char pattern[32];

    if (dir == NULL) {
        return 0U;
    }

    snprintf(pattern, sizeof(pattern), "%s%%" SCNu32 ".bin", queue_kind_prefix(kind));
    while ((entry = readdir(dir)) != NULL) {
        uint32_t seq = 0;
        if (sscanf(entry->d_name, pattern, &seq) == 1) {
            count++;
        }
    }

    closedir(dir);
    return count;
}

static bool flush_persistent_queues(uint32_t *failed_transmission_attempts)
{
    uint8_t packet_buffer[STORAGE_READ_BUFFER_MAX];

    while (true) {
        queue_kind_t kind = QUEUE_KIND_AUDIO;
        char path[STORAGE_PATH_MAX];
        uint32_t seq = 0;
        FILE *file = NULL;
        size_t payload_len = 0;
        bool delivered = false;
        esp_err_t err = ESP_OK;

        if (!find_oldest_packet_path(QUEUE_KIND_AUDIO, path, sizeof(path), &seq)) {
            kind = QUEUE_KIND_SENSOR;
            if (!find_oldest_packet_path(kind, path, sizeof(path), &seq)) {
                return true;
            }
        }

        file = fopen(path, "rb");
        if (file == NULL) {
            ESP_LOGW(TAG, "Queued packet disappeared before send: %s", path);
            continue;
        }

        payload_len = fread(packet_buffer, 1, sizeof(packet_buffer), file);
        if (ferror(file) != 0) {
            ESP_LOGE(TAG, "Failed reading queued packet %s", path);
            fclose(file);
            if (failed_transmission_attempts != NULL) {
                (*failed_transmission_attempts)++;
            }
            return false;
        }
        fclose(file);

        if (payload_len == 0U) {
            ESP_LOGW(TAG, "Removing empty queued packet %s", path);
            unlink(path);
            continue;
        }

        err = espnow_manager_send_and_wait(packet_buffer, payload_len, 1000, &delivered);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "espnow_manager_send failed while flushing %s queue: %s",
                     kind == QUEUE_KIND_AUDIO ? "audio" : "sensor",
                     esp_err_to_name(err));
            if (failed_transmission_attempts != NULL) {
                (*failed_transmission_attempts)++;
            }
            return false;
        }

        if (!delivered) {
            ESP_LOGW(TAG, "ESP-NOW delivery failed while flushing %s queue",
                     kind == QUEUE_KIND_AUDIO ? "audio" : "sensor");
            if (failed_transmission_attempts != NULL) {
                (*failed_transmission_attempts)++;
            }
            return false;
        }

        if (unlink(path) != 0) {
            ESP_LOGE(TAG, "Failed to delete sent queue file %s: errno=%d", path, errno);
            if (failed_transmission_attempts != NULL) {
                (*failed_transmission_attempts)++;
            }
            return false;
        }

        ESP_LOGI(TAG, "Sent queued %s packet %" PRIu32 " (%u bytes, %u audio pending, %u sensor pending)",
                 kind == QUEUE_KIND_AUDIO ? "audio" : "sensor",
                 seq,
                 (unsigned)payload_len,
                 (unsigned)count_queued_packets(QUEUE_KIND_AUDIO),
                 (unsigned)count_queued_packets(QUEUE_KIND_SENSOR));
        if (failed_transmission_attempts != NULL) {
            *failed_transmission_attempts = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(QUEUE_FLUSH_DELAY_MS));
    }

    return true;
}

static bool matches_json_field(const char *json, const char *field_name, const char *field_value)
{
    char token[64];

    snprintf(token, sizeof(token), "\"%s\":\"%s\"", field_name, field_value);
    return strstr(json, token) != NULL;
}

static bool parse_json_int64_field(const char *json, const char *field_name, int64_t *out_value)
{
    char token[40];
    if (json == NULL || field_name == NULL || out_value == NULL) {
        return false;
    }

    snprintf(token, sizeof(token), "\"%s\":", field_name);
    const char *cursor = strstr(json, token);
    if (cursor == NULL) {
        return false;
    }

    cursor += strlen(token);
    while (*cursor == ' ' || *cursor == '\t') {
        cursor++;
    }

    char *end = NULL;
    long long parsed = strtoll(cursor, &end, 10);
    if (end == cursor) {
        return false;
    }

    *out_value = (int64_t)parsed;
    return true;
}

static bool get_synced_epoch_ms(int64_t *out_ms)
{
    bool synced;
    int64_t base;
    TickType_t base_tick;

    portENTER_CRITICAL(&s_time_sync_mux);
    synced = s_time_synced;
    base = s_sync_epoch_ms;
    base_tick = s_sync_local_tick;
    portEXIT_CRITICAL(&s_time_sync_mux);

    if (!synced) {
        return false;
    }

    TickType_t now = xTaskGetTickCount();
    int64_t elapsed_ms = (int64_t)((uint32_t)(now - base_tick)) * (int64_t)portTICK_PERIOD_MS;
    *out_ms = base + elapsed_ms;
    return true;
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

    if (matches_json_field(normalized_json, "type", "led_override")) {
        ESP_LOGI(TAG, "Ignoring LED override command while WS2812 feature is disabled: %s", normalized_json);
        return;
    }

    if (matches_json_field(normalized_json, "type", "time_sync")) {
        int64_t epoch_ms = 0;
        if (!parse_json_int64_field(normalized_json, "epoch_ms", &epoch_ms)) {
            ESP_LOGW(TAG, "time_sync packet missing epoch_ms: %s", normalized_json);
            return;
        }

        TickType_t local_tick = xTaskGetTickCount();
        portENTER_CRITICAL(&s_time_sync_mux);
        s_sync_epoch_ms = epoch_ms;
        s_sync_local_tick = local_tick;
        s_time_synced = true;
        portEXIT_CRITICAL(&s_time_sync_mux);

        ESP_LOGI(TAG, "Time sync received: epoch_ms=%lld", (long long)epoch_ms);
        return;
    }

    ESP_LOGI(TAG, "Ignoring unknown base station message: %s", normalized_json);
}

static void format_optional_float(char *buffer, size_t buffer_len, bool has_value, float value, uint8_t decimals)
{
    if (!has_value) {
        snprintf(buffer, buffer_len, "null");
        return;
    }

    snprintf(buffer, buffer_len, "%.*f", decimals, value);
}

static bool nvs_load_offline_flag(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(OFFLINE_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return false;
    }

    uint8_t value = 0;
    err = nvs_get_u8(handle, OFFLINE_NVS_KEY, &value);
    nvs_close(handle);
    return err == ESP_OK && value != 0;
}

static void nvs_store_offline_flag(bool offline)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(OFFLINE_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open(%s) failed: %s", OFFLINE_NVS_NAMESPACE, esp_err_to_name(err));
        return;
    }

    err = nvs_set_u8(handle, OFFLINE_NVS_KEY, offline ? 1 : 0);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to persist offline flag: %s", esp_err_to_name(err));
    }
    nvs_close(handle);
}

static void button_gpio_init(void)
{
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);
}

static bool button_held_at_boot(void)
{
    if (gpio_get_level(BUTTON_GPIO) != 0) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    return gpio_get_level(BUTTON_GPIO) == 0;
}

static bool probe_basestation_for(uint32_t timeout_ms)
{
    const uint8_t probe[1] = { PROBE_MSG_TYPE };
    const uint32_t per_attempt_ms = 1000;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while ((int32_t)(deadline - xTaskGetTickCount()) > 0) {
        bool delivered = false;
        esp_err_t err = espnow_manager_send_and_wait(probe, sizeof(probe), per_attempt_ms, &delivered);
        if (err == ESP_OK && delivered) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

static bool read_audio_chunk_header_from_path(const char *path, audio_chunk_header_t *out_hdr)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        return false;
    }
    size_t read_bytes = fread(out_hdr, 1, sizeof(*out_hdr), f);
    fclose(f);
    return read_bytes == sizeof(*out_hdr);
}

static size_t count_queued_audio_recordings(void)
{
    DIR *dir = opendir(STORAGE_SCAN_PATH);
    if (dir == NULL) {
        return 0;
    }

    size_t recordings = 0;
    struct dirent *entry = NULL;
    char path[STORAGE_PATH_MAX];

    while ((entry = readdir(dir)) != NULL) {
        uint32_t seq = 0;
        if (sscanf(entry->d_name, "audio_%" SCNu32 ".bin", &seq) != 1) {
            continue;
        }

        snprintf(path, sizeof(path), STORAGE_BASE_PATH "/%s", entry->d_name);
        audio_chunk_header_t hdr = {0};
        if (!read_audio_chunk_header_from_path(path, &hdr)) {
            continue;
        }
        if (hdr.msg_type == AUDIO_MSG_TYPE && hdr.seq_num == 0U) {
            recordings++;
        }
    }

    closedir(dir);
    return recordings;
}

static esp_err_t drop_oldest_audio_recording(void)
{
    DIR *dir = opendir(STORAGE_SCAN_PATH);
    if (dir == NULL) {
        return ESP_FAIL;
    }

    uint32_t oldest_start_seq = UINT32_MAX;
    uint16_t oldest_total_chunks = 0;
    struct dirent *entry = NULL;
    char path[STORAGE_PATH_MAX];

    while ((entry = readdir(dir)) != NULL) {
        uint32_t seq = 0;
        if (sscanf(entry->d_name, "audio_%" SCNu32 ".bin", &seq) != 1) {
            continue;
        }

        snprintf(path, sizeof(path), STORAGE_BASE_PATH "/%s", entry->d_name);
        audio_chunk_header_t hdr = {0};
        if (!read_audio_chunk_header_from_path(path, &hdr)) {
            continue;
        }
        if (hdr.msg_type != AUDIO_MSG_TYPE || hdr.seq_num != 0U) {
            continue;
        }
        if (seq < oldest_start_seq) {
            oldest_start_seq = seq;
            oldest_total_chunks = hdr.total_chunks;
        }
    }
    closedir(dir);

    if (oldest_start_seq == UINT32_MAX || oldest_total_chunks == 0U) {
        return ESP_ERR_NOT_FOUND;
    }

    size_t deleted = 0;
    for (uint16_t i = 0; i < oldest_total_chunks; i++) {
        snprintf(path, sizeof(path), STORAGE_BASE_PATH "/audio_%" PRIu32 ".bin",
                 oldest_start_seq + (uint32_t)i);
        if (unlink(path) == 0) {
            deleted++;
        } else if (errno != ENOENT) {
            ESP_LOGW(TAG, "Failed to drop %s: errno=%d", path, errno);
        }
    }

    ESP_LOGI(TAG, "Dropped oldest audio recording: start_seq=%" PRIu32
                  ", expected_chunks=%u, removed=%u",
             oldest_start_seq, (unsigned)oldest_total_chunks, (unsigned)deleted);
    return ESP_OK;
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

static void format_payload(char *json, size_t json_len, uint32_t transmission_attempts, uint32_t sample_sequence)
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
    char battery_voltage[16];
    char measurement_epoch_str[24];

    int64_t measurement_epoch_ms = 0;
    if (get_synced_epoch_ms(&measurement_epoch_ms)) {
        snprintf(measurement_epoch_str, sizeof(measurement_epoch_str),
                 "%lld", (long long)measurement_epoch_ms);
    } else {
        snprintf(measurement_epoch_str, sizeof(measurement_epoch_str), "null");
    }

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

    snprintf(json, json_len,
             "{"
             "\"node_id\":\"%s\","
             "\"sample_sequence\":%" PRIu32 ","
             "\"transmission_attempts\":%" PRIu32 ","
             "\"measurement_epoch_ms\":%s,"
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
             sample_sequence,
             transmission_attempts,
             measurement_epoch_str,
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
             "disabled",
             "false");
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
        esp_err_t err = enqueue_packet(QUEUE_KIND_AUDIO, pkt, pkt_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to persist audio chunk %u/%u", i + 1, total_chunks);
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (s_offline_mode) {
        ESP_LOGI(TAG, "Audio recording stored offline (%u audio pending)",
                 (unsigned)count_queued_packets(QUEUE_KIND_AUDIO));
    } else if (!flush_persistent_queues(NULL)) {
        ESP_LOGW(TAG, "Audio queued for later delivery (%u audio pending)",
                 (unsigned)count_queued_packets(QUEUE_KIND_AUDIO));
    } else {
        ESP_LOGI(TAG, "Audio transmission complete");
    }
}

/* Push-to-talk button task */
static void button_task(void *pvParameters)
{
    (void)pvParameters;

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

        while (count_queued_audio_recordings() >= OFFLINE_MAX_AUDIO_RECORDINGS) {
            ESP_LOGW(TAG, "Audio queue at cap (%u recordings); dropping oldest",
                     (unsigned)OFFLINE_MAX_AUDIO_RECORDINGS);
            if (drop_oldest_audio_recording() != ESP_OK) {
                break;
            }
        }

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
    char json[JSON_PAYLOAD_MAX_LEN];
    TickType_t next_measurement_tick = xTaskGetTickCount();

    while (1)
    {
        TickType_t now = xTaskGetTickCount();
        bool measurement_due = now >= next_measurement_tick;
        bool has_pending_packets = count_queued_packets(QUEUE_KIND_AUDIO) > 0U ||
                                   count_queued_packets(QUEUE_KIND_SENSOR) > 0U;

        if (audio_active) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (measurement_due) {
            log_battery_probe();
            format_payload(json, sizeof(json), failed_transmission_attempts, s_sensor_sample_sequence++);

            size_t json_len = strlen(json);
            if (enqueue_packet(QUEUE_KIND_SENSOR, (const uint8_t *)json, json_len) != ESP_OK) {
                failed_transmission_attempts++;
                next_measurement_tick = now + pdMS_TO_TICKS(SEND_INTERVAL_MS);
                vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
                continue;
            }

            has_pending_packets = true;
            next_measurement_tick = now + pdMS_TO_TICKS(SEND_INTERVAL_MS);
        }

        if (!s_offline_mode && has_pending_packets &&
            (count_queued_packets(QUEUE_KIND_SENSOR) > 1U || count_queued_packets(QUEUE_KIND_AUDIO) > 0U || measurement_due)) {
            ESP_LOGI(TAG, "Queued payloads pending (%u audio, %u sensor)",
                     (unsigned)count_queued_packets(QUEUE_KIND_AUDIO),
                     (unsigned)count_queued_packets(QUEUE_KIND_SENSOR));
        }

        if (!s_offline_mode && has_pending_packets &&
            !flush_persistent_queues(&failed_transmission_attempts)) {
            ESP_LOGI(TAG, "Queue flush paused with %u audio and %u sensor payload(s) pending",
                     (unsigned)count_queued_packets(QUEUE_KIND_AUDIO),
                     (unsigned)count_queued_packets(QUEUE_KIND_SENSOR));
            vTaskDelay(pdMS_TO_TICKS(QUEUE_RETRY_DELAY_MS));
            continue;
        }

        now = xTaskGetTickCount();
        if (now < next_measurement_tick) {
            TickType_t sleep_ticks = next_measurement_tick - now;
            TickType_t retry_ticks = pdMS_TO_TICKS(QUEUE_RETRY_DELAY_MS);
            if (sleep_ticks > retry_ticks) {
                sleep_ticks = retry_ticks;
            }
            vTaskDelay(sleep_ticks);
        }
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
        .peer_mac = {0x24, 0x6f, 0x28, 0xb9, 0xb4, 0xb0},
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
    ESP_ERROR_CHECK(init_persistent_queue());

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

    ESP_LOGI(TAG, "WS2812/status LED feature disabled in firmware");

    button_gpio_init();
    s_offline_mode = nvs_load_offline_flag();

    if (button_held_at_boot()) {
        ESP_LOGI(TAG, "Boot button held — probing basestation for up to 10s");
        bool reachable = probe_basestation_for(10000);
        s_offline_mode = !reachable;
        nvs_store_offline_flag(s_offline_mode);
        ESP_LOGI(TAG, "Boot decision: %s",
                 s_offline_mode ? "OFFLINE (no basestation ack)" : "ONLINE (basestation reachable)");
    } else {
        ESP_LOGI(TAG, "Boot decision (button not held): %s",
                 s_offline_mode ? "OFFLINE (resumed)" : "ONLINE");
    }

    ESP_LOGI(TAG, "Queued at startup: %u audio chunk(s), %u sensor payload(s)",
             (unsigned)count_queued_packets(QUEUE_KIND_AUDIO),
             (unsigned)count_queued_packets(QUEUE_KIND_SENSOR));

    xTaskCreate(send_task, "send_task", 4096, NULL, 4, NULL);
    xTaskCreate(button_task, "button_task", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "Transmitter ready");
    web_monitor_manager_finalize_startup_history();
}
