#include "led_manager.h"

static gpio_num_t s_led_gpio = GPIO_NUM_NC;
static bool s_active_low = false;
static bool s_is_on = false;

esp_err_t led_manager_init(gpio_num_t gpio_num, bool active_low)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        return err;
    }

    s_led_gpio = gpio_num;
    s_active_low = active_low;

    return led_manager_set(true);
}

esp_err_t led_manager_set(bool on)
{
    if (s_led_gpio == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_STATE;
    }

    int level = on ? (s_active_low ? 0 : 1) : (s_active_low ? 1 : 0);
    esp_err_t err = gpio_set_level(s_led_gpio, level);
    if (err == ESP_OK) {
        s_is_on = on;
    }

    return err;
}

bool led_manager_is_on(void)
{
    return s_is_on;
}
