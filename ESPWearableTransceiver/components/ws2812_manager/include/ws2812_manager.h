#pragma once

#include "esp_err.h"
#include "driver/gpio.h"

/**
 * Initialise a single WS2812 LED on the given GPIO using the RMT peripheral.
 */
esp_err_t ws2812_manager_init(gpio_num_t gpio_num);

/**
 * Set the LED colour (0-255 per channel).
 */
esp_err_t ws2812_manager_set_color(uint8_t red, uint8_t green, uint8_t blue);
