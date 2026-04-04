#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

typedef enum {
    DHT_MANAGER_TYPE_DHT11 = 0,
    DHT_MANAGER_TYPE_DHT22,
} dht_manager_type_t;

typedef struct {
    float temperature_c;
    float humidity_percent;
} dht_manager_data_t;

esp_err_t dht_manager_init(gpio_num_t gpio_num, dht_manager_type_t sensor_type);
esp_err_t dht_manager_read(dht_manager_data_t *out_data);
