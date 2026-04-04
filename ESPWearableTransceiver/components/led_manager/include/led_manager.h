#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

esp_err_t led_manager_init(gpio_num_t gpio_num, bool active_low);
esp_err_t led_manager_set(bool on);
bool led_manager_is_on(void);
