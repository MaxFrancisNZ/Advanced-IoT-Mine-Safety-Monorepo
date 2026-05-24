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
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_wifi.h"

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

#define AUDIO_SAMPLE_RATE    8000
#define AUDIO_MAX_SECONDS    5
#define AUDIO_MAX_SAMPLES    (AUDIO_SAMPLE_RATE * AUDIO_MAX_SECONDS)
#define MIC_READ_CHUNK       512
#define BUTTON_DEBOUNCE_MS   50
#define AUDIO_BOOT_LOCKOUT_MS 5000
#define JSON_PAYLOAD_MAX_LEN 512
#define STORAGE_BASE_PATH    "/spiffs"
#define STORAGE_SCAN_PATH    STORAGE_BASE_PATH "/"
#define STORAGE_PATH_MAX     96
#define STORAGE_READ_BUFFER_MAX ESPNOW_MAX_PAYLOAD
#define AUDIO_QUEUE_FLUSH_DELAY_MS 5
#define AUDIO_SEND_TIMEOUT_MS 100
#define SENSOR_SEND_TIMEOUT_MS 500
#define QUEUE_RETRY_DELAY_MS 250

#define AUDIO_CHUNK_FILE_PREFIX     "aud_"
#define SENSOR_FILE_PREFIX          "sensor_"
#define AUDIO_SESSION_MAX_PENDING   4
#define AUDIO_SESSION_DEADLINE_MS   30000
#define AUDIO_SESSION_QUIET_MS      5000
#define AUDIO_INTER_CHUNK_DELAY_MS  5
#define SESSION_NVS_BOOT_CTR_KEY    "boot_ctr"
#define DUMP_SEND_TIMEOUT_MS        75

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
static uint32_t s_sensor_sample_sequence = 0;
static uint32_t s_sensor_queue_next_seq = 0;
static bool s_diagnostic_mode = false;
static volatile TickType_t s_sensor_resume_tick = 0;
static TickType_t s_boot_tick = 0;
static TickType_t s_audio_enable_tick = 0;

static volatile bool       s_time_synced     = false;
static volatile int64_t    s_sync_epoch_ms   = 0;
static volatile TickType_t s_sync_local_tick = 0;
static portMUX_TYPE        s_time_sync_mux   = portMUX_INITIALIZER_UNLOCKED;

static volatile bool s_offline_mode = false;
static volatile bool s_dump_in_progress = false;
static volatile bool s_dump_task_started = false;
static portMUX_TYPE s_dump_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_send_task_handle = NULL;
static TaskHandle_t s_button_task_handle = NULL;
static TaskHandle_t s_audio_monitor_task_handle = NULL;
static TaskHandle_t s_dump_task_handle = NULL;

typedef struct {
    bool       in_use;
    uint32_t   session_id;
    uint16_t   total_chunks;
    TickType_t started_tick;
    TickType_t last_activity_tick;
} pending_audio_session_t;

static pending_audio_session_t s_pending_sessions[AUDIO_SESSION_MAX_PENDING] = {0};
static portMUX_TYPE s_pending_sessions_mux = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    uint32_t seq;
    char name[32];
} sensor_queue_entry_t;

static uint16_t s_session_id_boot_ctr = 0;
static uint16_t s_session_id_next_low = 0;

#define OFFLINE_MAX_AUDIO_RECORDINGS 5U
#define OFFLINE_NVS_NAMESPACE "wearable"

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
static esp_err_t init_persistent_queue(void);
static esp_err_t refresh_queue_sequence_counters(void);
static esp_err_t clear_sensor_queue(void);
static esp_err_t enqueue_packet(queue_kind_t kind, const uint8_t *data, size_t len);
static size_t count_queued_packets(queue_kind_t kind);
static bool send_sensor_payload(const char *json, size_t json_len, uint32_t *failed_transmission_attempts);
static void button_gpio_init(void);
static bool button_pressed(void);
static bool build_storage_entry_path(char *path, size_t path_len, const char *entry_name);
static bool button_held_at_boot(void);
static size_t count_queued_audio_recordings(void);
static esp_err_t drop_oldest_audio_recording(void);
static void audio_chunk_path(char *buf, size_t buf_len, uint32_t session_id, uint16_t seq_num);
static bool parse_audio_chunk_name(const char *name, uint32_t *out_sid, uint16_t *out_seq);
static esp_err_t init_session_id_generator(void);
static uint32_t next_session_id(void);
static bool pending_session_register(uint32_t session_id, uint16_t total_chunks);
static bool pending_session_touch(uint32_t session_id);
static bool pending_session_remove(uint32_t session_id);
static size_t pending_session_count(void);
static void delete_session_files(uint32_t session_id);
static void retransmit_session_chunk(uint32_t session_id, uint16_t seq_num);
static void retransmit_session_all(uint32_t session_id);
static void handle_audio_nack(const uint8_t *data, int len);
static void handle_audio_ack(const uint8_t *data, int len);
static void audio_session_monitor_task(void *pvParameters);
static void audio_recovery_scan_on_boot(void);
static void arm_sensor_dump(void);
static void sensor_dump_task(void *pvParameters);

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

static void audio_chunk_path(char *buf, size_t buf_len, uint32_t session_id, uint16_t seq_num)
{
    snprintf(buf, buf_len, STORAGE_BASE_PATH "/" AUDIO_CHUNK_FILE_PREFIX "%08" PRIX32 "_%05u.bin",
             session_id, (unsigned)seq_num);
}

static bool parse_audio_chunk_name(const char *name, uint32_t *out_sid, uint16_t *out_seq)
{
    uint32_t sid = 0;
    unsigned seq = 0;
    if (sscanf(name, AUDIO_CHUNK_FILE_PREFIX "%08" SCNx32 "_%05u.bin", &sid, &seq) != 2) {
        return false;
    }
    if (seq > UINT16_MAX) {
        return false;
    }
    if (out_sid != NULL) *out_sid = sid;
    if (out_seq != NULL) *out_seq = (uint16_t)seq;
    return true;
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
    bool saw_sensor = false;

    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open queue storage directory: errno=%d", errno);
        return ESP_FAIL;
    }

    while ((entry = readdir(dir)) != NULL) {
        uint32_t seq = 0;

        if (sscanf(entry->d_name, "sensor_%" SCNu32 ".bin", &seq) == 1) {
            if (!saw_sensor || seq > max_sensor_seq) {
                max_sensor_seq = seq;
            }
            saw_sensor = true;
        }
        /* Audio chunks are addressed by (session_id, seq_num) embedded in their
         * filenames and do not use a monotonic queue counter. */
    }

    closedir(dir);
    s_sensor_queue_next_seq = saw_sensor ? (max_sensor_seq + 1U) : 0U;

    ESP_LOGI(TAG, "Recovered queued packets: %u audio, %u sensor",
             (unsigned)count_queued_packets(QUEUE_KIND_AUDIO),
             (unsigned)count_queued_packets(QUEUE_KIND_SENSOR));
    return ESP_OK;
}

static esp_err_t clear_sensor_queue(void)
{
    DIR *dir = opendir(STORAGE_SCAN_PATH);
    struct dirent *entry = NULL;
    unsigned removed_sensor_packets = 0;

    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open queue storage directory: errno=%d", errno);
        return ESP_FAIL;
    }

    while ((entry = readdir(dir)) != NULL) {
        uint32_t seq = 0;
        if (sscanf(entry->d_name, SENSOR_FILE_PREFIX "%" SCNu32 ".bin", &seq) == 1) {
            char path[STORAGE_PATH_MAX];
            if (build_storage_entry_path(path, sizeof(path), entry->d_name) && unlink(path) == 0) {
                removed_sensor_packets++;
            }
        }
    }

    closedir(dir);
    s_sensor_queue_next_seq = 0;
    if (removed_sensor_packets > 0U) {
        ESP_LOGI(TAG, "Removed %u queued offline sensor payload(s)", removed_sensor_packets);
    }
    return ESP_OK;
}

static esp_err_t enqueue_packet(queue_kind_t kind, const uint8_t *data, size_t len)
{
    char path[STORAGE_PATH_MAX];
    FILE *file = NULL;
    size_t written = 0;

    if (data == NULL || len == 0U || len > STORAGE_READ_BUFFER_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    if (kind == QUEUE_KIND_AUDIO) {
        if (len < AUDIO_CHUNK_HDR_SIZE) {
            return ESP_ERR_INVALID_ARG;
        }
        const audio_chunk_header_t *hdr = (const audio_chunk_header_t *)data;
        if (hdr->msg_type != AUDIO_MSG_TYPE) {
            return ESP_ERR_INVALID_ARG;
        }
        audio_chunk_path(path, sizeof(path), hdr->session_id, hdr->seq_num);
    } else if (kind == QUEUE_KIND_SENSOR) {
        int written_len = snprintf(path, sizeof(path), STORAGE_BASE_PATH "/" SENSOR_FILE_PREFIX "%" PRIu32 ".bin",
                                   s_sensor_queue_next_seq++);
        if (written_len < 0 || (size_t)written_len >= sizeof(path)) {
            return ESP_ERR_INVALID_SIZE;
        }
    } else {
        return ESP_ERR_INVALID_ARG;
    }

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

    if (kind == QUEUE_KIND_AUDIO) {
        const audio_chunk_header_t *hdr = (const audio_chunk_header_t *)data;
        ESP_LOGI(TAG, "Queued audio chunk sid=%08" PRIX32 " seq=%u (%u bytes)",
                 hdr->session_id, (unsigned)hdr->seq_num, (unsigned)len);
    } else {
        ESP_LOGI(TAG, "Queued offline sensor payload %s (%u bytes)", path, (unsigned)len);
    }
    return ESP_OK;
}

static size_t count_queued_packets(queue_kind_t kind)
{
    DIR *dir = opendir(STORAGE_SCAN_PATH);
    struct dirent *entry = NULL;
    size_t count = 0;

    if (dir == NULL) {
        return 0U;
    }

    if (kind == QUEUE_KIND_AUDIO) {
        while ((entry = readdir(dir)) != NULL) {
            if (parse_audio_chunk_name(entry->d_name, NULL, NULL)) {
                count++;
            }
        }
        closedir(dir);
        return count;
    }

    while ((entry = readdir(dir)) != NULL) {
        uint32_t seq = 0;
        if (sscanf(entry->d_name, "sensor_%" SCNu32 ".bin", &seq) == 1) {
            count++;
        }
    }

    closedir(dir);
    return count;
}

static bool send_sensor_payload(const char *json, size_t json_len, uint32_t *failed_transmission_attempts)
{
    bool delivered = false;
    esp_err_t err;

    if (json == NULL || json_len == 0U) {
        return false;
    }

    if (s_dump_in_progress) {
        ESP_LOGW(TAG, "Suppressing sensor sample while dump is active");
        return false;
    }

    if (s_offline_mode) {
        err = enqueue_packet(QUEUE_KIND_SENSOR, (const uint8_t *)json, json_len);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to persist offline sensor sample: %s", esp_err_to_name(err));
            if (failed_transmission_attempts != NULL) {
                (*failed_transmission_attempts)++;
            }
            return false;
        }
        ESP_LOGI(TAG, "Sensor sample persisted offline: %s", json);
        if (failed_transmission_attempts != NULL) {
            *failed_transmission_attempts = 0;
        }
        return true;
    }

    err = espnow_manager_send_and_wait((const uint8_t *)json, json_len,
                                       SENSOR_SEND_TIMEOUT_MS, &delivered);
    if (err != ESP_OK || !delivered) {
        ESP_LOGW(TAG, "Sensor sample send failed, dropping payload: %s",
                 err != ESP_OK ? esp_err_to_name(err) : "no MAC ACK");
        if (failed_transmission_attempts != NULL) {
            (*failed_transmission_attempts)++;
        }
        return false;
    }

    ESP_LOGI(TAG, "Sensor sample sent: %s", json);
    if (failed_transmission_attempts != NULL) {
        *failed_transmission_attempts = 0;
    }
    return true;
}

static int compare_sensor_queue_entries(const void *a, const void *b)
{
    const sensor_queue_entry_t *entry_a = (const sensor_queue_entry_t *)a;
    const sensor_queue_entry_t *entry_b = (const sensor_queue_entry_t *)b;

    if (entry_a->seq < entry_b->seq) {
        return -1;
    }
    if (entry_a->seq > entry_b->seq) {
        return 1;
    }
    return 0;
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

    /* Binary control messages from the basestation are dispatched on first
     * byte (msg_type). They are sent as broadcasts; ignore the peer-MAC
     * check that JSON commands honour. */
    switch (data[0]) {
    case AUDIO_NACK_MSG_TYPE:
        handle_audio_nack(data, len);
        return;
    case AUDIO_ACK_MSG_TYPE:
        handle_audio_ack(data, len);
        return;
    case DUMP_REQUEST_MSG_TYPE:
        arm_sensor_dump();
        return;
    default:
        break;
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

static void button_gpio_init(void)
{
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);
}

static bool button_pressed(void)
{
    return gpio_get_level(BUTTON_GPIO) == 0;
}

static bool build_storage_entry_path(char *path, size_t path_len, const char *entry_name)
{
    int written;

    if (path == NULL || path_len == 0U || entry_name == NULL) {
        return false;
    }

    written = snprintf(path, path_len, STORAGE_BASE_PATH "/%s", entry_name);
    if (written < 0 || (size_t)written >= path_len) {
        ESP_LOGW(TAG, "Skipping overlong storage entry path: %s", entry_name);
        return false;
    }

    return true;
}

static bool button_held_at_boot(void)
{
    if (!button_pressed()) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
    return button_pressed();
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
    /* A "recording" is one session_id. Count distinct session_ids on disk. */
    DIR *dir = opendir(STORAGE_SCAN_PATH);
    if (dir == NULL) {
        return 0;
    }

    uint32_t seen[AUDIO_SESSION_MAX_PENDING * 2] = {0};
    size_t seen_count = 0;
    struct dirent *entry = NULL;

    while ((entry = readdir(dir)) != NULL) {
        uint32_t sid = 0;
        if (!parse_audio_chunk_name(entry->d_name, &sid, NULL)) {
            continue;
        }

        bool already = false;
        for (size_t i = 0; i < seen_count; i++) {
            if (seen[i] == sid) { already = true; break; }
        }
        if (!already && seen_count < (sizeof(seen) / sizeof(seen[0]))) {
            seen[seen_count++] = sid;
        }
    }

    closedir(dir);
    return seen_count;
}

static esp_err_t drop_oldest_audio_recording(void)
{
    /* Find the lexicographically smallest session_id present on disk and
     * delete every chunk belonging to it. Also clear it from the pending
     * session table if present. */
    DIR *dir = opendir(STORAGE_SCAN_PATH);
    if (dir == NULL) {
        return ESP_FAIL;
    }

    uint32_t oldest_sid = 0;
    bool found = false;
    struct dirent *entry = NULL;

    while ((entry = readdir(dir)) != NULL) {
        uint32_t sid = 0;
        if (!parse_audio_chunk_name(entry->d_name, &sid, NULL)) {
            continue;
        }
        if (!found || sid < oldest_sid) {
            oldest_sid = sid;
            found = true;
        }
    }
    closedir(dir);

    if (!found) {
        return ESP_ERR_NOT_FOUND;
    }

    delete_session_files(oldest_sid);
    pending_session_remove(oldest_sid);
    ESP_LOGI(TAG, "Dropped oldest audio recording sid=%08" PRIX32, oldest_sid);
    return ESP_OK;
}

static bool s_session_id_armed = false;

static esp_err_t init_session_id_generator(void)
{
    if (s_session_id_armed) {
        return ESP_OK;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(OFFLINE_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "session_id NVS open failed: %s", esp_err_to_name(err));
        s_session_id_boot_ctr = (uint16_t)(xTaskGetTickCount() & 0xFFFFu);
        s_session_id_next_low = 0;
        s_session_id_armed = true;
        return err;
    }

    uint16_t boot_ctr = 0;
    err = nvs_get_u16(handle, SESSION_NVS_BOOT_CTR_KEY, &boot_ctr);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        boot_ctr = 0;
        err = ESP_OK;
    }
    if (err != ESP_OK) {
        nvs_close(handle);
        ESP_LOGW(TAG, "session_id NVS read failed: %s", esp_err_to_name(err));
        s_session_id_boot_ctr = (uint16_t)(xTaskGetTickCount() & 0xFFFFu);
        s_session_id_next_low = 0;
        s_session_id_armed = true;
        return err;
    }

    boot_ctr = (uint16_t)(boot_ctr + 1U);
    err = nvs_set_u16(handle, SESSION_NVS_BOOT_CTR_KEY, boot_ctr);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    s_session_id_boot_ctr = boot_ctr;
    s_session_id_next_low = 0;
    s_session_id_armed = true;
    ESP_LOGI(TAG, "Session id generator armed: boot_ctr=%u", (unsigned)boot_ctr);
    return err;
}

static uint32_t next_session_id(void)
{
    if (!s_session_id_armed) {
        init_session_id_generator();
    }
    uint32_t high = (uint32_t)s_session_id_boot_ctr;
    uint16_t low = s_session_id_next_low++;
    return (high << 16) | (uint32_t)low;
}

static bool pending_session_register(uint32_t session_id, uint16_t total_chunks)
{
    bool ok = false;
    TickType_t now = xTaskGetTickCount();
    portENTER_CRITICAL(&s_pending_sessions_mux);
    for (size_t i = 0; i < AUDIO_SESSION_MAX_PENDING; i++) {
        if (s_pending_sessions[i].in_use && s_pending_sessions[i].session_id == session_id) {
            s_pending_sessions[i].total_chunks = total_chunks;
            s_pending_sessions[i].last_activity_tick = now;
            ok = true;
            break;
        }
    }
    if (!ok) {
        for (size_t i = 0; i < AUDIO_SESSION_MAX_PENDING; i++) {
            if (!s_pending_sessions[i].in_use) {
                s_pending_sessions[i].in_use = true;
                s_pending_sessions[i].session_id = session_id;
                s_pending_sessions[i].total_chunks = total_chunks;
                s_pending_sessions[i].started_tick = now;
                s_pending_sessions[i].last_activity_tick = now;
                ok = true;
                break;
            }
        }
    }
    portEXIT_CRITICAL(&s_pending_sessions_mux);
    if (!ok) {
        ESP_LOGW(TAG, "pending_session_register: table full, sid=%08" PRIX32, session_id);
    }
    return ok;
}

static bool pending_session_touch(uint32_t session_id)
{
    bool ok = false;
    TickType_t now = xTaskGetTickCount();
    portENTER_CRITICAL(&s_pending_sessions_mux);
    for (size_t i = 0; i < AUDIO_SESSION_MAX_PENDING; i++) {
        if (s_pending_sessions[i].in_use && s_pending_sessions[i].session_id == session_id) {
            s_pending_sessions[i].last_activity_tick = now;
            ok = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_pending_sessions_mux);
    return ok;
}

static bool pending_session_remove(uint32_t session_id)
{
    bool removed = false;
    portENTER_CRITICAL(&s_pending_sessions_mux);
    for (size_t i = 0; i < AUDIO_SESSION_MAX_PENDING; i++) {
        if (s_pending_sessions[i].in_use && s_pending_sessions[i].session_id == session_id) {
            s_pending_sessions[i] = (pending_audio_session_t){0};
            removed = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_pending_sessions_mux);
    return removed;
}

static size_t pending_session_count(void)
{
    size_t count = 0;
    portENTER_CRITICAL(&s_pending_sessions_mux);
    for (size_t i = 0; i < AUDIO_SESSION_MAX_PENDING; i++) {
        if (s_pending_sessions[i].in_use) count++;
    }
    portEXIT_CRITICAL(&s_pending_sessions_mux);
    return count;
}

static void delete_session_files(uint32_t session_id)
{
    /* Heap-allocate the filename list so we don't blow the task stack. SPIFFS
     * readdir+unlink mid-iteration is undefined, so we collect first, then
     * close the dir, then unlink. */
    #define DELETE_NAMES_MAX 64
    #define DELETE_NAME_LEN  32
    char (*names)[DELETE_NAME_LEN] = calloc(DELETE_NAMES_MAX, DELETE_NAME_LEN);
    if (names == NULL) {
        ESP_LOGW(TAG, "delete_session_files: out of heap, skipping sid=%08" PRIX32, session_id);
        return;
    }

    DIR *dir = opendir(STORAGE_SCAN_PATH);
    if (dir == NULL) {
        free(names);
        return;
    }

    size_t n_names = 0;
    struct dirent *entry = NULL;
    while ((entry = readdir(dir)) != NULL && n_names < DELETE_NAMES_MAX) {
        uint32_t sid = 0;
        if (!parse_audio_chunk_name(entry->d_name, &sid, NULL)) continue;
        if (sid != session_id) continue;
        strncpy(names[n_names], entry->d_name, DELETE_NAME_LEN - 1U);
        names[n_names][DELETE_NAME_LEN - 1U] = '\0';
        n_names++;
    }
    closedir(dir);

    size_t deleted = 0;
    char path[STORAGE_PATH_MAX];
    for (size_t i = 0; i < n_names; i++) {
        if (!build_storage_entry_path(path, sizeof(path), names[i])) continue;
        if (unlink(path) == 0) {
            deleted++;
        } else if (errno != ENOENT) {
            ESP_LOGW(TAG, "Failed to unlink %s: errno=%d", path, errno);
        }
    }
    free(names);
    ESP_LOGI(TAG, "Cleared %u file(s) for session sid=%08" PRIX32, (unsigned)deleted, session_id);
    #undef DELETE_NAMES_MAX
    #undef DELETE_NAME_LEN
}

static void retransmit_session_chunk(uint32_t session_id, uint16_t seq_num)
{
    char path[STORAGE_PATH_MAX];
    audio_chunk_path(path, sizeof(path), session_id, seq_num);

    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        ESP_LOGW(TAG, "Retransmit: missing %s", path);
        return;
    }

    /* Heap-allocate the 1470-byte read buffer so we stay friendly to small
     * task stacks (e.g. the ESP-NOW recv callback that invokes the NACK
     * handler). */
    uint8_t *buffer = malloc(STORAGE_READ_BUFFER_MAX);
    if (buffer == NULL) {
        ESP_LOGW(TAG, "Retransmit: out of heap for sid=%08" PRIX32 " seq=%u",
                 session_id, (unsigned)seq_num);
        fclose(file);
        return;
    }

    size_t read_len = fread(buffer, 1, STORAGE_READ_BUFFER_MAX, file);
    fclose(file);
    if (read_len < AUDIO_CHUNK_HDR_SIZE) {
        ESP_LOGW(TAG, "Retransmit: truncated chunk %s (%u bytes)", path, (unsigned)read_len);
        free(buffer);
        return;
    }

    bool delivered = false;
    esp_err_t err = espnow_manager_send_and_wait(buffer, read_len, AUDIO_SEND_TIMEOUT_MS, &delivered);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Retransmit send failed sid=%08" PRIX32 " seq=%u: %s",
                 session_id, (unsigned)seq_num, esp_err_to_name(err));
    } else if (!delivered) {
        ESP_LOGW(TAG, "Retransmit no MAC-ACK sid=%08" PRIX32 " seq=%u",
                 session_id, (unsigned)seq_num);
    }
    free(buffer);
}

static void retransmit_session_all(uint32_t session_id)
{
    #define RETRANSMIT_MAX_SEQS 256
    uint16_t *seqs = malloc(sizeof(uint16_t) * RETRANSMIT_MAX_SEQS);
    if (seqs == NULL) {
        ESP_LOGW(TAG, "Self-retransmit out of heap for sid=%08" PRIX32, session_id);
        return;
    }

    DIR *dir = opendir(STORAGE_SCAN_PATH);
    if (dir == NULL) {
        free(seqs);
        return;
    }

    size_t n_seqs = 0;
    struct dirent *entry = NULL;
    while ((entry = readdir(dir)) != NULL && n_seqs < RETRANSMIT_MAX_SEQS) {
        uint32_t sid = 0;
        uint16_t seq = 0;
        if (!parse_audio_chunk_name(entry->d_name, &sid, &seq)) continue;
        if (sid != session_id) continue;
        seqs[n_seqs++] = seq;
    }
    closedir(dir);

    ESP_LOGI(TAG, "Self-retransmit sid=%08" PRIX32 ": %u chunk(s)",
             session_id, (unsigned)n_seqs);
    for (size_t i = 0; i < n_seqs; i++) {
        retransmit_session_chunk(session_id, seqs[i]);
        vTaskDelay(pdMS_TO_TICKS(AUDIO_INTER_CHUNK_DELAY_MS));
    }
    free(seqs);
    #undef RETRANSMIT_MAX_SEQS
}

static void handle_audio_nack(const uint8_t *data, int len)
{
    if (s_dump_in_progress) {
        return;
    }

    if (len < (int)sizeof(audio_nack_msg_t)) {
        ESP_LOGW(TAG, "NACK packet too short (%d bytes)", len);
        return;
    }
    const audio_nack_msg_t *hdr = (const audio_nack_msg_t *)data;

    uint8_t own_mac[6];
    esp_read_mac(own_mac, ESP_MAC_WIFI_STA);
    if (memcmp(hdr->target_mac, own_mac, 6) != 0) {
        return;  /* not for us */
    }

    size_t expected = sizeof(audio_nack_msg_t) + (size_t)hdr->missing_count * sizeof(uint16_t);
    if ((size_t)len < expected) {
        ESP_LOGW(TAG, "NACK truncated: header says %u seqs, only %d bytes",
                 (unsigned)hdr->missing_count, len);
        return;
    }

    /* The seqs array sits at an odd byte offset (13). Use memcpy to read each
     * value to avoid unaligned uint16_t loads. */
    const uint8_t *seqs_bytes = data + sizeof(audio_nack_msg_t);
    uint32_t session_id = hdr->session_id;
    uint16_t missing_count = hdr->missing_count;
    ESP_LOGI(TAG, "NACK received sid=%08" PRIX32 ", %u missing seqs",
             session_id, (unsigned)missing_count);

    pending_session_touch(session_id);
    for (uint16_t i = 0; i < missing_count; i++) {
        uint16_t seq = 0;
        memcpy(&seq, seqs_bytes + (size_t)i * sizeof(uint16_t), sizeof(seq));
        retransmit_session_chunk(session_id, seq);
        vTaskDelay(pdMS_TO_TICKS(AUDIO_INTER_CHUNK_DELAY_MS));
    }
}

static void handle_audio_ack(const uint8_t *data, int len)
{
    if (s_dump_in_progress) {
        return;
    }

    if (len < (int)sizeof(audio_ack_msg_t)) {
        ESP_LOGW(TAG, "ACK packet too short (%d bytes)", len);
        return;
    }
    const audio_ack_msg_t *hdr = (const audio_ack_msg_t *)data;

    uint8_t own_mac[6];
    esp_read_mac(own_mac, ESP_MAC_WIFI_STA);
    if (memcmp(hdr->target_mac, own_mac, 6) != 0) {
        return;
    }

    ESP_LOGI(TAG, "SESSION_ACK sid=%08" PRIX32, hdr->session_id);
    delete_session_files(hdr->session_id);
    pending_session_remove(hdr->session_id);

    if (pending_session_count() == 0U) {
        audio_active = false;
        s_sensor_resume_tick = xTaskGetTickCount() + pdMS_TO_TICKS(SEND_INTERVAL_MS);
        ESP_LOGI(TAG, "All sessions cleared, sensors resume in %u ms",
                 (unsigned)SEND_INTERVAL_MS);
    }
}

static void audio_session_monitor_task(void *pvParameters)
{
    (void)pvParameters;

    /* Deferred boot-time recovery: scan SPIFFS for orphan session files and
     * register them. Runs once, after the system is stable and on this task's
     * roomy stack. */
    audio_recovery_scan_on_boot();

    while (1) {
        if (s_dump_in_progress) {
            ESP_LOGI(TAG, "Audio session monitor stopping for sensor dump");
            s_audio_monitor_task_handle = NULL;
            vTaskDelete(NULL);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

        TickType_t now = xTaskGetTickCount();
        bool any_in_use = false;

        for (size_t i = 0; i < AUDIO_SESSION_MAX_PENDING; i++) {
            uint32_t session_id = 0;
            TickType_t started = 0;
            TickType_t last_act = 0;
            bool in_use = false;

            portENTER_CRITICAL(&s_pending_sessions_mux);
            in_use = s_pending_sessions[i].in_use;
            if (in_use) {
                session_id = s_pending_sessions[i].session_id;
                started = s_pending_sessions[i].started_tick;
                last_act = s_pending_sessions[i].last_activity_tick;
            }
            portEXIT_CRITICAL(&s_pending_sessions_mux);

            if (!in_use) continue;
            any_in_use = true;

            TickType_t elapsed_since_start = now - started;
            TickType_t elapsed_since_act = now - last_act;

            if (elapsed_since_start >= pdMS_TO_TICKS(AUDIO_SESSION_DEADLINE_MS)) {
                ESP_LOGW(TAG, "Session sid=%08" PRIX32 " expired (%u ms), dropping",
                         session_id, (unsigned)AUDIO_SESSION_DEADLINE_MS);
                delete_session_files(session_id);
                pending_session_remove(session_id);
                continue;
            }

            if (elapsed_since_act >= pdMS_TO_TICKS(AUDIO_SESSION_QUIET_MS)) {
                ESP_LOGI(TAG, "Session sid=%08" PRIX32 " quiet for %u ms, self-retransmitting",
                         session_id, (unsigned)(elapsed_since_act * portTICK_PERIOD_MS));
                pending_session_touch(session_id);
                retransmit_session_all(session_id);
            }
        }

        if (!any_in_use && audio_active) {
            audio_active = false;
            s_sensor_resume_tick = xTaskGetTickCount() + pdMS_TO_TICKS(SEND_INTERVAL_MS);
            ESP_LOGI(TAG, "Pending session table empty, sensors resume in %u ms",
                     (unsigned)SEND_INTERVAL_MS);
        }
    }
}

static void audio_recovery_scan_on_boot(void)
{
    DIR *dir = opendir(STORAGE_SCAN_PATH);
    if (dir == NULL) {
        return;
    }

    /* Discover all distinct session_ids and tally chunks per session. */
    struct {
        uint32_t sid;
        uint16_t chunks;
    } sessions[AUDIO_SESSION_MAX_PENDING * 4];
    size_t n_sessions = 0;

    struct dirent *entry = NULL;
    while ((entry = readdir(dir)) != NULL) {
        uint32_t sid = 0;
        if (!parse_audio_chunk_name(entry->d_name, &sid, NULL)) continue;

        bool found = false;
        for (size_t i = 0; i < n_sessions; i++) {
            if (sessions[i].sid == sid) {
                sessions[i].chunks++;
                found = true;
                break;
            }
        }
        if (!found && n_sessions < (sizeof(sessions) / sizeof(sessions[0]))) {
            sessions[n_sessions].sid = sid;
            sessions[n_sessions].chunks = 1;
            n_sessions++;
        }
    }
    closedir(dir);

    if (n_sessions == 0U) {
        return;
    }

    /* Only the first AUDIO_SESSION_MAX_PENDING fit in the tracker. Drop any
     * surplus from disk so we don't accumulate dead files forever. */
    for (size_t i = 0; i < n_sessions; i++) {
        if (i < AUDIO_SESSION_MAX_PENDING) {
            pending_session_register(sessions[i].sid, sessions[i].chunks);
            ESP_LOGI(TAG, "Recovered pending audio session sid=%08" PRIX32 ", %u chunk(s)",
                     sessions[i].sid, (unsigned)sessions[i].chunks);
        } else {
            ESP_LOGW(TAG, "Boot recovery overflow: dropping session sid=%08" PRIX32,
                     sessions[i].sid);
            delete_session_files(sessions[i].sid);
        }
    }

    if (pending_session_count() > 0U) {
        audio_active = true;
    }
}

static void arm_sensor_dump(void)
{
    bool should_start = false;

    portENTER_CRITICAL(&s_dump_mux);
    if (!s_dump_task_started) {
        s_dump_task_started = true;
        s_dump_in_progress = true;
        should_start = true;
    }
    portEXIT_CRITICAL(&s_dump_mux);

    if (!should_start) {
        ESP_LOGI(TAG, "Dump request ignored; dump already in progress");
        return;
    }

    BaseType_t created = xTaskCreate(sensor_dump_task, "sensor_dump", 8192, NULL, 6, &s_dump_task_handle);
    if (created != pdPASS) {
        portENTER_CRITICAL(&s_dump_mux);
        s_dump_task_started = false;
        s_dump_in_progress = false;
        portEXIT_CRITICAL(&s_dump_mux);
        ESP_LOGE(TAG, "Failed to start sensor dump task");
    }
}

static void sensor_dump_task(void *pvParameters)
{
    (void)pvParameters;

    audio_active = false;
    s_sensor_resume_tick = 0;
    (void)led_manager_set(false);
    (void)mic_manager_stop();

    ESP_LOGI(TAG, "Offline sensor dump started");

    vTaskDelay(pdMS_TO_TICKS(100));

    DIR *dir = opendir(STORAGE_SCAN_PATH);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open queue storage for dump: errno=%d", errno);
        goto done;
    }

    size_t capacity = 32;
    size_t count = 0;
    sensor_queue_entry_t *entries = calloc(capacity, sizeof(sensor_queue_entry_t));
    if (entries == NULL) {
        closedir(dir);
        ESP_LOGE(TAG, "Out of heap while preparing sensor dump");
        goto done;
    }

    struct dirent *entry = NULL;
    while ((entry = readdir(dir)) != NULL) {
        uint32_t seq = 0;
        if (sscanf(entry->d_name, SENSOR_FILE_PREFIX "%" SCNu32 ".bin", &seq) != 1) {
            continue;
        }

        if (count == capacity) {
            size_t new_capacity = capacity * 2U;
            sensor_queue_entry_t *new_entries = realloc(entries, new_capacity * sizeof(sensor_queue_entry_t));
            if (new_entries == NULL) {
                ESP_LOGW(TAG, "Sensor dump list truncated at %u entries", (unsigned)count);
                break;
            }
            entries = new_entries;
            capacity = new_capacity;
        }

        entries[count].seq = seq;
        strncpy(entries[count].name, entry->d_name, sizeof(entries[count].name) - 1U);
        entries[count].name[sizeof(entries[count].name) - 1U] = '\0';
        count++;
    }
    closedir(dir);

    qsort(entries, count, sizeof(entries[0]), compare_sensor_queue_entries);

    uint8_t *buffer = malloc(STORAGE_READ_BUFFER_MAX);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Out of heap for sensor dump read buffer");
        free(entries);
        goto done;
    }

    size_t sent = 0;
    size_t failed = 0;
    for (size_t i = 0; i < count; i++) {
        char path[STORAGE_PATH_MAX];
        if (!build_storage_entry_path(path, sizeof(path), entries[i].name)) {
            failed++;
            continue;
        }

        FILE *file = fopen(path, "rb");
        if (file == NULL) {
            if (errno != ENOENT) {
                ESP_LOGW(TAG, "Dump read open failed for %s: errno=%d", path, errno);
                failed++;
            }
            continue;
        }

        size_t read_len = fread(buffer, 1, STORAGE_READ_BUFFER_MAX, file);
        bool read_error = ferror(file) != 0;
        fclose(file);
        if (read_error || read_len == 0U) {
            ESP_LOGW(TAG, "Dump read failed for %s", path);
            failed++;
            continue;
        }

        bool delivered = false;
        esp_err_t err = espnow_manager_send_and_wait(buffer, read_len, DUMP_SEND_TIMEOUT_MS, &delivered);
        if (err == ESP_OK && delivered) {
            if (unlink(path) != 0 && errno != ENOENT) {
                ESP_LOGW(TAG, "Dump sent but unlink failed for %s: errno=%d", path, errno);
            }
            sent++;
        } else {
            ESP_LOGW(TAG, "Dump send failed for %s: %s%s",
                     path, esp_err_to_name(err), delivered ? "" : " no-ack");
            failed++;
        }
    }

    free(buffer);
    free(entries);
    ESP_LOGI(TAG, "Offline sensor dump complete: sent=%u failed=%u",
             (unsigned)sent, (unsigned)failed);

done:
    portENTER_CRITICAL(&s_dump_mux);
    s_dump_task_started = false;
    portEXIT_CRITICAL(&s_dump_mux);
    s_dump_task_handle = NULL;
    vTaskDelete(NULL);
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
    if (s_dump_in_progress) {
        ESP_LOGW(TAG, "Skipping audio send because sensor dump is active");
        return;
    }

    uint8_t wifi_mac[6];
    esp_read_mac(wifi_mac, ESP_MAC_WIFI_STA);

    uint16_t total_chunks = (uint16_t)((adpcm_len + AUDIO_CHUNK_DATA_MAX - 1) / AUDIO_CHUNK_DATA_MAX);
    uint32_t session_id = next_session_id();

    ESP_LOGI(TAG, "Sending audio: sid=%08" PRIX32 ", %zu bytes ADPCM, %"PRIu32" samples, %u chunks",
             session_id, adpcm_len, total_samples, total_chunks);

    while (count_queued_audio_recordings() >= OFFLINE_MAX_AUDIO_RECORDINGS) {
        ESP_LOGW(TAG, "Audio queue at cap (%u recordings); dropping oldest",
                 (unsigned)OFFLINE_MAX_AUDIO_RECORDINGS);
        if (drop_oldest_audio_recording() != ESP_OK) {
            break;
        }
    }

    /* Register the session up-front so monitor task tracks it from the start
     * and sensor activity stays suspended even if MAC-level sends fail.
     * Re-assert audio_active in case an unrelated SESSION_ACK cleared it
     * between the button press and this call. */
    audio_active = true;
    pending_session_register(session_id, total_chunks);

    size_t persisted = 0;
    size_t sent_ok = 0;
    for (uint16_t i = 0; i < total_chunks; i++) {
        uint8_t pkt[ESPNOW_MAX_PAYLOAD];
        audio_chunk_header_t *hdr = (audio_chunk_header_t *)pkt;

        hdr->msg_type      = AUDIO_MSG_TYPE;
        memcpy(hdr->node_mac, wifi_mac, 6);
        hdr->session_id    = session_id;
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
        persisted++;

        if (!s_offline_mode) {
            esp_err_t send_err = espnow_manager_send(pkt, pkt_len);
            if (send_err == ESP_OK) {
                sent_ok++;
            } else {
                ESP_LOGW(TAG, "Initial send failed sid=%08" PRIX32 " seq=%u: %s",
                         session_id, (unsigned)i, esp_err_to_name(send_err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(AUDIO_INTER_CHUNK_DELAY_MS));
    }

    /* Touch the session at end-of-burst so the quiet timer resets. */
    pending_session_touch(session_id);

    if (s_offline_mode) {
        ESP_LOGI(TAG, "Audio session sid=%08" PRIX32 " persisted offline (%u/%u chunks on disk)",
                 session_id, (unsigned)persisted, (unsigned)total_chunks);
    } else {
        ESP_LOGI(TAG, "Audio session sid=%08" PRIX32 " transmitted: persisted=%u/%u, send_ok=%u, awaiting ACK",
                 session_id, (unsigned)persisted, (unsigned)total_chunks, (unsigned)sent_ok);
    }
}

/* Push-to-talk button task */
static void button_task(void *pvParameters)
{
    (void)pvParameters;

    bool was_pressed = false;
    int16_t read_buf[MIC_READ_CHUNK];

    if (led_manager_set(false) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to force LED off at button task start");
    }

    while (1) {
        if (s_dump_in_progress) {
            ESP_LOGI(TAG, "Button task stopping for sensor dump");
            s_button_task_handle = NULL;
            vTaskDelete(NULL);
        }

        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(s_audio_enable_tick - now) > 0) {
            if (was_pressed) {
                was_pressed = false;
                audio_active = false;
                (void)led_manager_set(false);
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        bool pressed = button_pressed();
        if (pressed != was_pressed) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
            pressed = button_pressed();
            if (pressed != was_pressed) {
                was_pressed = pressed;
                audio_active = pressed;
                if (led_manager_set(pressed) != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to set LED state for button event");
                }
                ESP_LOGI(TAG, "Button %s", pressed ? "pressed" : "released");

                if (pressed && s_diagnostic_mode) {
                    size_t adpcm_buf_size = (AUDIO_MAX_SAMPLES + 1) / 2;
                    uint8_t *adpcm_buf = malloc(adpcm_buf_size);
                    size_t total_samples = 0;
                    size_t adpcm_offset = 0;
                    adpcm_state_t adpcm_state = {0, 0};

                    if (adpcm_buf == NULL) {
                        ESP_LOGE(TAG, "Failed to allocate diagnostic ADPCM buffer (%zu bytes)", adpcm_buf_size);
                        audio_active = false;
                        was_pressed = false;
                        if (led_manager_set(false) != ESP_OK) {
                            ESP_LOGW(TAG, "Failed to clear LED after diagnostic audio allocation failure");
                        }
                        while (!s_dump_in_progress && button_pressed()) {
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }
                        s_sensor_resume_tick = xTaskGetTickCount() + pdMS_TO_TICKS(SEND_INTERVAL_MS);
                        continue;
                    }

                    esp_err_t err = mic_manager_start();
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Diagnostic mic start failed: %s", esp_err_to_name(err));
                        free(adpcm_buf);
                        audio_active = false;
                        was_pressed = false;
                        if (led_manager_set(false) != ESP_OK) {
                            ESP_LOGW(TAG, "Failed to clear LED after diagnostic mic start failure");
                        }
                        while (!s_dump_in_progress && button_pressed()) {
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }
                        s_sensor_resume_tick = xTaskGetTickCount() + pdMS_TO_TICKS(SEND_INTERVAL_MS);
                    } else {
                        ESP_LOGI(TAG, "Diagnostic audio capture started");

                        while (!s_dump_in_progress && button_pressed() && total_samples < AUDIO_MAX_SAMPLES) {
                            size_t remaining = AUDIO_MAX_SAMPLES - total_samples;
                            size_t to_read = remaining < MIC_READ_CHUNK ? remaining : MIC_READ_CHUNK;
                            size_t samples_read = 0;

                            err = mic_manager_read(read_buf, to_read, &samples_read, 250);
                            if (err != ESP_OK) {
                                ESP_LOGW(TAG, "Diagnostic mic read failed: %s", esp_err_to_name(err));
                                continue;
                            }

                            size_t encoded = adpcm_encode(read_buf, samples_read,
                                                          adpcm_buf + adpcm_offset, &adpcm_state);
                            adpcm_offset += encoded;
                            total_samples += samples_read;
                        }

                        err = mic_manager_stop();
                        if (err != ESP_OK) {
                            ESP_LOGW(TAG, "Diagnostic mic stop failed: %s", esp_err_to_name(err));
                        }

                        ESP_LOGI(TAG, "Diagnostic audio capture stopped: %zu samples (%.1f sec, %zu bytes ADPCM)",
                                 total_samples, (float)total_samples / AUDIO_SAMPLE_RATE, adpcm_offset);

                        bool session_started = false;
                        if (total_samples > 0) {
                            send_audio_chunks(adpcm_buf, adpcm_offset, (uint32_t)total_samples);
                            session_started = (pending_session_count() > 0U);
                        }

                        was_pressed = false;
                        if (led_manager_set(false) != ESP_OK) {
                            ESP_LOGW(TAG, "Failed to clear LED after diagnostic mic capture");
                        }
                        while (!s_dump_in_progress && button_pressed()) {
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }
                        /* If a session was registered, audio_active stays true
                         * until SESSION_ACK or the 30 s deadline — the monitor
                         * task arms s_sensor_resume_tick at that point. */
                        if (!session_started) {
                            audio_active = false;
                            s_sensor_resume_tick = xTaskGetTickCount() + pdMS_TO_TICKS(SEND_INTERVAL_MS);
                        }
                        free(adpcm_buf);
                    }
                } else if (!pressed) {
                    s_sensor_resume_tick = xTaskGetTickCount() + pdMS_TO_TICKS(SEND_INTERVAL_MS);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
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
        if (s_dump_in_progress) {
            ESP_LOGI(TAG, "Send task stopping for sensor dump");
            s_send_task_handle = NULL;
            vTaskDelete(NULL);
        }

        TickType_t now = xTaskGetTickCount();
        bool measurement_due = now >= next_measurement_tick;
        TickType_t resume_tick = s_sensor_resume_tick;

        if (resume_tick != 0 && (int32_t)(resume_tick - next_measurement_tick) > 0) {
            next_measurement_tick = resume_tick;
            measurement_due = now >= next_measurement_tick;
        }

        if (audio_active) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (s_diagnostic_mode) {
            if (measurement_due) {
                log_battery_probe();
                format_payload(json, sizeof(json), failed_transmission_attempts, s_sensor_sample_sequence++);

                size_t json_len = strlen(json);
                (void)send_sensor_payload(json, json_len, &failed_transmission_attempts);
                next_measurement_tick = now + pdMS_TO_TICKS(SEND_INTERVAL_MS);
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
            continue;
        }

        if (measurement_due) {
            if (audio_active) {
                next_measurement_tick = xTaskGetTickCount() + pdMS_TO_TICKS(SEND_INTERVAL_MS);
                continue;
            }

            log_battery_probe();
            format_payload(json, sizeof(json), failed_transmission_attempts, s_sensor_sample_sequence++);

            size_t json_len = strlen(json);
            (void)send_sensor_payload(json, json_len, &failed_transmission_attempts);
            next_measurement_tick = now + pdMS_TO_TICKS(SEND_INTERVAL_MS);
        }

        if (audio_active) {
            vTaskDelay(pdMS_TO_TICKS(50));
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
    bool diagnostic_mode = true;
    s_diagnostic_mode = diagnostic_mode;
    s_boot_tick = xTaskGetTickCount();
    s_audio_enable_tick = s_boot_tick + pdMS_TO_TICKS(AUDIO_BOOT_LOCKOUT_MS);

    ESP_ERROR_CHECK(led_manager_init(LED_GPIO, true));
    ESP_ERROR_CHECK(led_manager_set(false));

    button_gpio_init();

    if (diagnostic_mode) {
        const i2c_manager_config_t i2c_config = {
            .port = I2C_PORT_NUM,
            .sda_io_num = I2C_SDA_GPIO,
            .scl_io_num = I2C_SCL_GPIO,
            .scl_speed_hz = I2C_FREQ_HZ,
            .enable_internal_pullup = true,
            .glitch_ignore_cnt = 7,
        };
        const espnow_manager_config_t espnow_config = {
            .peer_mac = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
            .channel = ESPNOW_CHANNEL,
            .encrypt = false,
            .log_tag = TAG,
        };

        esp_err_t battery_err = battery_probe_init(BATTERY_SENSE_GPIO);
        if (battery_err != ESP_OK) {
            ESP_LOGW(TAG, "Battery probe init failed on GPIO35: %s", esp_err_to_name(battery_err));
        } else {
            battery_probe_ready = true;
        }

        esp_err_t dht_err = dht_manager_init(HUMITURE_GPIO, HUMITURE_TYPE);
        if (dht_err != ESP_OK) {
            log_sensor_error("DHT", dht_err);
        } else {
            dht_ready = true;
        }

        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
        ESP_ERROR_CHECK(init_persistent_queue());
        s_offline_mode = button_held_at_boot();
        if (s_offline_mode) {
            portENTER_CRITICAL(&s_time_sync_mux);
            s_sync_epoch_ms = 0;
            s_sync_local_tick = s_boot_tick;
            s_time_synced = true;
            portEXIT_CRITICAL(&s_time_sync_mux);
            ESP_LOGI(TAG, "Boot button held: OFFLINE-ALONE mode active");
        } else {
            ESP_ERROR_CHECK(clear_sensor_queue());
            ESP_LOGI(TAG, "Boot button not held: ONLINE mode active");
        }
        /* Session ID generator now self-initializes lazily on first
         * next_session_id() call so a flaky NVS write can't brick the boot. */
        /* Boot-time orphan-session recovery deferred to the monitor task —
         * see audio_session_monitor_task. Keeps app_main's stack lean. */

        ESP_ERROR_CHECK(i2c_manager_init(&i2c_config, &i2c_bus));
        i2c_manager_scan(i2c_bus, i2c_config.scl_speed_hz, TAG);
        initialize_sensors();

        esp_err_t espnow_err = espnow_manager_init(&espnow_config);
        if (espnow_err != ESP_OK) {
            ESP_LOGE(TAG, "Diagnostic ESPNOW init failed: %s", esp_err_to_name(espnow_err));
            s_offline_mode = true;
        } else {
            memcpy(base_station_mac, espnow_config.peer_mac, sizeof(base_station_mac));
            esp_err_t recv_cb_err = espnow_manager_register_recv_cb(handle_base_station_message, NULL);
            if (recv_cb_err != ESP_OK) {
                ESP_LOGW(TAG, "Diagnostic ESPNOW recv callback registration failed: %s",
                         esp_err_to_name(recv_cb_err));
            }
        }

        uint8_t wifi_sta_mac[6] = {0};
        ESP_ERROR_CHECK(esp_read_mac(wifi_sta_mac, ESP_MAC_WIFI_STA));
        snprintf(node_id, sizeof(node_id), "%02X:%02X:%02X:%02X:%02X:%02X",
                 wifi_sta_mac[0], wifi_sta_mac[1], wifi_sta_mac[2],
                 wifi_sta_mac[3], wifi_sta_mac[4], wifi_sta_mac[5]);

        const mic_manager_config_t mic_config = {
            .bck_gpio    = MIC_I2S_BCK_GPIO,
            .ws_gpio     = MIC_I2S_WS_GPIO,
            .din_gpio    = MIC_I2S_DIN_GPIO,
            .sample_rate = AUDIO_SAMPLE_RATE,
        };
        esp_err_t mic_err = mic_manager_init(&mic_config);
        if (mic_err != ESP_OK) {
            ESP_LOGE(TAG, "Diagnostic microphone init failed: %s", esp_err_to_name(mic_err));
        }

        ESP_LOGI(TAG, "Diagnostic mode: web monitor disabled, ESPNOW path active");
        xTaskCreate(send_task, "send_task", 4096, NULL, 4, &s_send_task_handle);
        xTaskCreate(audio_session_monitor_task, "audio_session_mon", 8192, NULL, 4, &s_audio_monitor_task_handle);
    } else {
        const i2c_manager_config_t i2c_config = {
            .port = I2C_PORT_NUM,
            .sda_io_num = I2C_SDA_GPIO,
            .scl_io_num = I2C_SCL_GPIO,
            .scl_speed_hz = I2C_FREQ_HZ,
            .enable_internal_pullup = true,
            .glitch_ignore_cnt = 7,
        };
        const espnow_manager_config_t espnow_config = {
            .peer_mac = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
            .channel = ESPNOW_CHANNEL,
            .encrypt = false,
            .log_tag = TAG,
        };

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
        s_offline_mode = button_held_at_boot();
        if (s_offline_mode) {
            portENTER_CRITICAL(&s_time_sync_mux);
            s_sync_epoch_ms = 0;
            s_sync_local_tick = s_boot_tick;
            s_time_synced = true;
            portEXIT_CRITICAL(&s_time_sync_mux);
            ESP_LOGI(TAG, "Boot button held: OFFLINE-ALONE mode active");
        } else {
            ESP_ERROR_CHECK(clear_sensor_queue());
            ESP_LOGI(TAG, "Boot button not held: ONLINE mode active");
        }
        /* Session ID generator now self-initializes lazily on first
         * next_session_id() call so a flaky NVS write can't brick the boot. */
        /* Boot-time orphan-session recovery deferred to the monitor task —
         * see audio_session_monitor_task. Keeps app_main's stack lean. */

        ESP_ERROR_CHECK(espnow_manager_init(&espnow_config));
        memcpy(base_station_mac, espnow_config.peer_mac, sizeof(base_station_mac));
        ESP_ERROR_CHECK(espnow_manager_register_recv_cb(handle_base_station_message, NULL));

        uint8_t wifi_sta_mac[6] = {0};
        ESP_ERROR_CHECK(esp_read_mac(wifi_sta_mac, ESP_MAC_WIFI_STA));
        snprintf(node_id, sizeof(node_id), "%02X:%02X:%02X:%02X:%02X:%02X",
                 wifi_sta_mac[0], wifi_sta_mac[1], wifi_sta_mac[2],
                 wifi_sta_mac[3], wifi_sta_mac[4], wifi_sta_mac[5]);

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

        ESP_LOGI(TAG, "Queued at startup: %u audio chunk(s), %u sensor payload(s)",
                 (unsigned)count_queued_packets(QUEUE_KIND_AUDIO),
                 (unsigned)count_queued_packets(QUEUE_KIND_SENSOR));

        xTaskCreate(send_task, "send_task", 4096, NULL, 4, &s_send_task_handle);
        xTaskCreate(audio_session_monitor_task, "audio_session_mon", 8192, NULL, 4, &s_audio_monitor_task_handle);
    }

    xTaskCreate(button_task, "button_task", 8192, NULL, 5, &s_button_task_handle);

    ESP_LOGI(TAG, "Diagnostic firmware ready");
}
