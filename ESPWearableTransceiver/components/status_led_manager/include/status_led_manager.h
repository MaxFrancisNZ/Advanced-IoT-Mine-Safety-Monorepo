#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

typedef enum {
    STATUS_LED_STATE_SAFE = 0,
    STATUS_LED_STATE_WARNING,
    STATUS_LED_STATE_DANGER,
    STATUS_LED_STATE_FLASHING_RED_OVERRIDE,
} status_led_render_state_t;

typedef struct {
    bool temperature_available;
    float temperature_c;
    bool humidity_available;
    float humidity_percent;
    bool battery_available;
    float battery_voltage;
    bool extreme_acceleration_detected;
    bool prolonged_lying_down_detected;
} status_led_inputs_t;

esp_err_t status_led_manager_init(gpio_num_t gpio_num);
esp_err_t status_led_manager_update_inputs(const status_led_inputs_t *inputs);
esp_err_t status_led_manager_set_remote_override(bool enabled);
status_led_render_state_t status_led_manager_get_render_state(void);
const char *status_led_manager_get_render_state_name(void);
bool status_led_manager_is_remote_override_active(void);
