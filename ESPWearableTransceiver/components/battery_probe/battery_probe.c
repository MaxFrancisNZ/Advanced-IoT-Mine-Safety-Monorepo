#include "battery_probe.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#define BATTERY_PROBE_DIVIDER_RATIO 2.0f

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_cali_handle = NULL;
static adc_channel_t s_channel = ADC_CHANNEL_7;
static bool s_is_initialized = false;

static esp_err_t gpio_to_adc_channel(gpio_num_t gpio_num, adc_channel_t *out_channel)
{
    if (out_channel == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (gpio_num) {
        case GPIO_NUM_32:
            *out_channel = ADC_CHANNEL_4;
            return ESP_OK;
        case GPIO_NUM_33:
            *out_channel = ADC_CHANNEL_5;
            return ESP_OK;
        case GPIO_NUM_34:
            *out_channel = ADC_CHANNEL_6;
            return ESP_OK;
        case GPIO_NUM_35:
            *out_channel = ADC_CHANNEL_7;
            return ESP_OK;
        case GPIO_NUM_36:
            *out_channel = ADC_CHANNEL_0;
            return ESP_OK;
        case GPIO_NUM_39:
            *out_channel = ADC_CHANNEL_3;
            return ESP_OK;
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

esp_err_t battery_probe_init(gpio_num_t gpio_num)
{
    esp_err_t err = gpio_to_adc_channel(gpio_num, &s_channel);
    if (err != ESP_OK) {
        return err;
    }

    if (s_adc_handle == NULL) {
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
        };
        err = adc_oneshot_new_unit(&init_config, &s_adc_handle);
        if (err != ESP_OK) {
            return err;
        }
    }

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc_handle, s_channel, &channel_config);
    if (err != ESP_OK) {
        return err;
    }

    if (s_cali_handle == NULL) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .chan = s_channel,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_curve_fitting(&cali_config, &s_cali_handle) != ESP_OK) {
            s_cali_handle = NULL;
        }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_line_fitting(&cali_config, &s_cali_handle) != ESP_OK) {
            s_cali_handle = NULL;
        }
#endif
    }

    s_is_initialized = true;
    return ESP_OK;
}

esp_err_t battery_probe_read(battery_probe_reading_t *out_reading)
{
    if (!s_is_initialized || out_reading == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw = 0;
    esp_err_t err = adc_oneshot_read(s_adc_handle, s_channel, &raw);
    if (err != ESP_OK) {
        return err;
    }

    out_reading->raw = raw;
    out_reading->pin_millivolts = 0;
    out_reading->battery_millivolts = 0;
    out_reading->battery_voltage = 0.0f;
    out_reading->calibrated = false;

    if (s_cali_handle != NULL) {
        int mv = 0;
        err = adc_cali_raw_to_voltage(s_cali_handle, raw, &mv);
        if (err == ESP_OK) {
            out_reading->pin_millivolts = mv;
            out_reading->battery_millivolts = (int)(mv * BATTERY_PROBE_DIVIDER_RATIO);
            out_reading->battery_voltage = (float)out_reading->battery_millivolts / 1000.0f;
            out_reading->calibrated = true;
        }
    }

    return ESP_OK;
}
