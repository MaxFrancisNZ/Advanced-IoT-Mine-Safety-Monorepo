#include "dht_manager.h"

#include <stdint.h>

#include "esp_check.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#define DHT_MANAGER_START_SIGNAL_MS 20
#define DHT_MANAGER_TIMEOUT_US 120
#define DHT_MANAGER_BIT_THRESHOLD_US 40

static const char *TAG = "DHT_MANAGER";
static gpio_num_t s_dht_gpio = GPIO_NUM_NC;
static dht_manager_type_t s_dht_type = DHT_MANAGER_TYPE_DHT11;

static void dht_set_output(void)
{
    gpio_set_direction(s_dht_gpio, GPIO_MODE_OUTPUT_OD);
}

static void dht_set_input(void)
{
    gpio_set_direction(s_dht_gpio, GPIO_MODE_INPUT);
}

static esp_err_t wait_for_level(int expected_level, int timeout_us, int64_t *duration_us)
{
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(s_dht_gpio) == expected_level) {
        int64_t now = esp_timer_get_time();
        if ((now - start) > timeout_us) {
            return ESP_ERR_TIMEOUT;
        }
    }

    if (duration_us != NULL) {
        *duration_us = esp_timer_get_time() - start;
    }

    return ESP_OK;
}

static esp_err_t wait_until_level(int target_level, int timeout_us)
{
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(s_dht_gpio) != target_level) {
        int64_t now = esp_timer_get_time();
        if ((now - start) > timeout_us) {
            return ESP_ERR_TIMEOUT;
        }
    }

    return ESP_OK;
}

esp_err_t dht_manager_init(gpio_num_t gpio_num, dht_manager_type_t sensor_type)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        return err;
    }

    s_dht_gpio = gpio_num;
    s_dht_type = sensor_type;
    gpio_set_level(s_dht_gpio, 1);
    dht_set_input();

    return ESP_OK;
}

esp_err_t dht_manager_read(dht_manager_data_t *out_data)
{
    if (out_data == NULL || s_dht_gpio == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[5] = {0};

    dht_set_output();
    gpio_set_level(s_dht_gpio, 0);
    esp_rom_delay_us(DHT_MANAGER_START_SIGNAL_MS * 1000);
    gpio_set_level(s_dht_gpio, 1);
    esp_rom_delay_us(40);
    dht_set_input();

    ESP_RETURN_ON_ERROR(wait_for_level(1, DHT_MANAGER_TIMEOUT_US, NULL), TAG, "DHT response low timeout");
    ESP_RETURN_ON_ERROR(wait_for_level(0, DHT_MANAGER_TIMEOUT_US, NULL), TAG, "DHT response high timeout");

    for (int bit = 0; bit < 40; bit++) {
        int64_t high_duration_us = 0;

        ESP_RETURN_ON_ERROR(wait_until_level(1, DHT_MANAGER_TIMEOUT_US), TAG, "DHT bit start timeout");
        ESP_RETURN_ON_ERROR(wait_for_level(1, DHT_MANAGER_TIMEOUT_US, &high_duration_us), TAG, "DHT bit high timeout");

        data[bit / 8] <<= 1;
        if (high_duration_us > DHT_MANAGER_BIT_THRESHOLD_US) {
            data[bit / 8] |= 1U;
        }
    }

    uint8_t checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (checksum != data[4]) {
        return ESP_ERR_INVALID_CRC;
    }

    if (s_dht_type == DHT_MANAGER_TYPE_DHT11) {
        out_data->humidity_percent = (float)data[0];
        out_data->temperature_c = (float)data[2];
        return ESP_OK;
    }

    uint16_t raw_humidity = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_temperature = ((uint16_t)data[2] << 8) | data[3];
    bool negative = (raw_temperature & 0x8000U) != 0U;
    raw_temperature &= 0x7FFFU;

    out_data->humidity_percent = (float)raw_humidity / 10.0f;
    out_data->temperature_c = (float)raw_temperature / 10.0f;
    if (negative) {
        out_data->temperature_c = -out_data->temperature_c;
    }

    return ESP_OK;
}
