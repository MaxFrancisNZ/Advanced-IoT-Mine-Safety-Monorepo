#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    int raw;
    int pin_millivolts;
    int battery_millivolts;
    float battery_voltage;
    bool calibrated;
} battery_probe_reading_t;

esp_err_t battery_probe_init(gpio_num_t gpio_num);
esp_err_t battery_probe_read(battery_probe_reading_t *out_reading);
