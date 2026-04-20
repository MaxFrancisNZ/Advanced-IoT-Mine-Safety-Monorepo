#include "status_led_manager.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ws2812_manager.h"

typedef enum {
    SIGNAL_STATUS_SAFE = 0,
    SIGNAL_STATUS_WARNING,
    SIGNAL_STATUS_DANGER,
} signal_status_t;

typedef struct {
    float safe_min;
    float safe_max;
    float warning_min;
    float warning_max;
} range_threshold_t;

static const char *TAG = "status_led";

static const range_threshold_t TEMPERATURE_THRESHOLDS = {
    .safe_min = 10.0f,
    .safe_max = 35.0f,
    .warning_min = 0.0f,
    .warning_max = 45.0f,
};

static const range_threshold_t HUMIDITY_THRESHOLDS = {
    .safe_min = 30.0f,
    .safe_max = 80.0f,
    .warning_min = 20.0f,
    .warning_max = 90.0f,
};

static const struct {
    float safe_min_voltage;
    float warning_min_voltage;
} BATTERY_THRESHOLDS = {
    .safe_min_voltage = 3.7f,
    .warning_min_voltage = 3.4f,
};

static const struct {
    float warning_g;
    float danger_g;
} EXTREME_ACCELERATION_THRESHOLDS = {
    .warning_g = 2.5f,
    .danger_g = 4.0f,
};

static const uint32_t PROLONGED_LYING_DOWN_DURATION_MS = 30000;
static const TickType_t FLASH_PERIOD_TICKS = pdMS_TO_TICKS(500);
static const TickType_t TASK_LOOP_TICKS = pdMS_TO_TICKS(100);

static SemaphoreHandle_t s_state_lock = NULL;
static status_led_inputs_t s_inputs = {0};
static bool s_remote_override_active = false;
static status_led_render_state_t s_render_state = STATUS_LED_STATE_SAFE;
static bool s_flash_led_on = false;
static bool s_initialized = false;

static signal_status_t evaluate_range_signal(float value, const range_threshold_t *thresholds)
{
    if (value < thresholds->warning_min || value > thresholds->warning_max) {
        return SIGNAL_STATUS_DANGER;
    }

    if (value < thresholds->safe_min || value > thresholds->safe_max) {
        return SIGNAL_STATUS_WARNING;
    }

    return SIGNAL_STATUS_SAFE;
}

static signal_status_t evaluate_battery_signal(float voltage)
{
    if (voltage < BATTERY_THRESHOLDS.warning_min_voltage) {
        return SIGNAL_STATUS_DANGER;
    }

    if (voltage < BATTERY_THRESHOLDS.safe_min_voltage) {
        return SIGNAL_STATUS_WARNING;
    }

    return SIGNAL_STATUS_SAFE;
}

static status_led_render_state_t evaluate_automatic_state(const status_led_inputs_t *inputs)
{
    bool warning_present = false;

    if (inputs->temperature_available) {
        signal_status_t status = evaluate_range_signal(inputs->temperature_c, &TEMPERATURE_THRESHOLDS);
        if (status == SIGNAL_STATUS_DANGER) {
            return STATUS_LED_STATE_DANGER;
        }
        warning_present = warning_present || (status == SIGNAL_STATUS_WARNING);
    }

    if (inputs->humidity_available) {
        signal_status_t status = evaluate_range_signal(inputs->humidity_percent, &HUMIDITY_THRESHOLDS);
        if (status == SIGNAL_STATUS_DANGER) {
            return STATUS_LED_STATE_DANGER;
        }
        warning_present = warning_present || (status == SIGNAL_STATUS_WARNING);
    }

    if (inputs->battery_available) {
        signal_status_t status = evaluate_battery_signal(inputs->battery_voltage);
        if (status == SIGNAL_STATUS_DANGER) {
            return STATUS_LED_STATE_DANGER;
        }
        warning_present = warning_present || (status == SIGNAL_STATUS_WARNING);
    }

    /* Hook placeholders for later motion/posture detectors. Threshold constants
     * are defined now, but these flags only become active when upstream logic
     * starts setting them based on real detection. */
    if (inputs->extreme_acceleration_detected || inputs->prolonged_lying_down_detected) {
        return STATUS_LED_STATE_DANGER;
    }

    return warning_present ? STATUS_LED_STATE_WARNING : STATUS_LED_STATE_SAFE;
}

static esp_err_t set_ws2812_color_for_state(status_led_render_state_t state, bool flash_on)
{
    switch (state) {
    case STATUS_LED_STATE_SAFE:
        return ws2812_manager_set_color(0, 30, 0);
    case STATUS_LED_STATE_WARNING:
        return ws2812_manager_set_color(30, 12, 0);
    case STATUS_LED_STATE_DANGER:
        return ws2812_manager_set_color(30, 0, 0);
    case STATUS_LED_STATE_FLASHING_RED_OVERRIDE:
        return flash_on ? ws2812_manager_set_color(30, 0, 0) : ws2812_manager_set_color(0, 0, 0);
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

static void status_led_task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_flash_toggle = xTaskGetTickCount();
    status_led_render_state_t last_applied_state = (status_led_render_state_t)(-1);
    bool last_applied_flash_on = false;

    while (1) {
        status_led_inputs_t inputs_snapshot = {0};
        bool override_active = false;
        status_led_render_state_t render_state;
        bool flash_on = true;

        xSemaphoreTake(s_state_lock, portMAX_DELAY);
        inputs_snapshot = s_inputs;
        override_active = s_remote_override_active;
        xSemaphoreGive(s_state_lock);

        if (override_active) {
            render_state = STATUS_LED_STATE_FLASHING_RED_OVERRIDE;
            if ((xTaskGetTickCount() - last_flash_toggle) >= FLASH_PERIOD_TICKS) {
                s_flash_led_on = !s_flash_led_on;
                last_flash_toggle = xTaskGetTickCount();
            }
            flash_on = s_flash_led_on;
        } else {
            render_state = evaluate_automatic_state(&inputs_snapshot);
            s_flash_led_on = true;
            last_flash_toggle = xTaskGetTickCount();
            flash_on = true;
        }

        xSemaphoreTake(s_state_lock, portMAX_DELAY);
        s_render_state = render_state;
        xSemaphoreGive(s_state_lock);

        if (render_state != last_applied_state || flash_on != last_applied_flash_on) {
            esp_err_t err = set_ws2812_color_for_state(render_state, flash_on);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to update WS2812 state: %s", esp_err_to_name(err));
            }
            last_applied_state = render_state;
            last_applied_flash_on = flash_on;
        }

        vTaskDelay(TASK_LOOP_TICKS);
    }
}

esp_err_t status_led_manager_init(gpio_num_t gpio_num)
{
    if (s_initialized) {
        return ESP_OK;
    }

    s_state_lock = xSemaphoreCreateMutex();
    if (s_state_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = ws2812_manager_init(gpio_num);
    if (err != ESP_OK) {
        return err;
    }

    s_inputs = (status_led_inputs_t){0};
    s_remote_override_active = false;
    s_render_state = STATUS_LED_STATE_SAFE;
    s_flash_led_on = true;

    err = ws2812_manager_set_color(0, 30, 0);
    if (err != ESP_OK) {
        return err;
    }

    BaseType_t task_ok = xTaskCreate(status_led_task, "status_led_task", 4096, NULL, 4, NULL);
    if (task_ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG,
             "Status LED ready (static safe green; temp %.1f/%.1f/%.1f/%.1f, humidity %.1f/%.1f/%.1f/%.1f, battery %.1f/%.1f, accel %.1fg/%.1fg, posture %lums)",
             TEMPERATURE_THRESHOLDS.warning_min, TEMPERATURE_THRESHOLDS.safe_min,
             TEMPERATURE_THRESHOLDS.safe_max, TEMPERATURE_THRESHOLDS.warning_max,
             HUMIDITY_THRESHOLDS.warning_min, HUMIDITY_THRESHOLDS.safe_min,
             HUMIDITY_THRESHOLDS.safe_max, HUMIDITY_THRESHOLDS.warning_max,
             BATTERY_THRESHOLDS.warning_min_voltage, BATTERY_THRESHOLDS.safe_min_voltage,
             EXTREME_ACCELERATION_THRESHOLDS.warning_g, EXTREME_ACCELERATION_THRESHOLDS.danger_g,
             (unsigned long)PROLONGED_LYING_DOWN_DURATION_MS);
    return ESP_OK;
}

esp_err_t status_led_manager_update_inputs(const status_led_inputs_t *inputs)
{
    if (!s_initialized || inputs == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_state_lock, portMAX_DELAY);
    s_inputs = *inputs;
    xSemaphoreGive(s_state_lock);
    return ESP_OK;
}

esp_err_t status_led_manager_set_remote_override(bool enabled)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_state_lock, portMAX_DELAY);
    s_remote_override_active = enabled;
    if (!enabled) {
        s_flash_led_on = true;
    }
    xSemaphoreGive(s_state_lock);

    return ESP_OK;
}

status_led_render_state_t status_led_manager_get_render_state(void)
{
    status_led_render_state_t state;

    if (!s_initialized) {
        return STATUS_LED_STATE_SAFE;
    }

    xSemaphoreTake(s_state_lock, portMAX_DELAY);
    state = s_render_state;
    xSemaphoreGive(s_state_lock);
    return state;
}

const char *status_led_manager_get_render_state_name(void)
{
    switch (status_led_manager_get_render_state()) {
    case STATUS_LED_STATE_SAFE:
        return "green";
    case STATUS_LED_STATE_WARNING:
        return "orange";
    case STATUS_LED_STATE_DANGER:
        return "red";
    case STATUS_LED_STATE_FLASHING_RED_OVERRIDE:
        return "flashing_red_override";
    default:
        return "unknown";
    }
}

bool status_led_manager_is_remote_override_active(void)
{
    bool active = false;

    if (!s_initialized) {
        return false;
    }

    xSemaphoreTake(s_state_lock, portMAX_DELAY);
    active = s_remote_override_active;
    xSemaphoreGive(s_state_lock);
    return active;
}
