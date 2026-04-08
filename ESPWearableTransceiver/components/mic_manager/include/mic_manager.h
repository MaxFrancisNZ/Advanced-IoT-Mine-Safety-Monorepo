#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    gpio_num_t bck_gpio;
    gpio_num_t ws_gpio;
    gpio_num_t din_gpio;
    uint32_t sample_rate;
} mic_manager_config_t;

/**
 * Initialise the I2S peripheral for an SPH0645 MEMS microphone.
 * Call once at startup.
 */
esp_err_t mic_manager_init(const mic_manager_config_t *config);

/**
 * Enable the I2S RX channel and begin clocking the microphone.
 */
esp_err_t mic_manager_start(void);

/**
 * Read 16-bit mono PCM samples from the microphone.
 *
 * @param buffer       Output buffer for int16_t samples.
 * @param max_samples  Maximum number of samples to read.
 * @param samples_read Actual number of samples placed in buffer.
 * @param timeout_ms   Read timeout in milliseconds.
 * @return ESP_OK on success.
 */
esp_err_t mic_manager_read(int16_t *buffer, size_t max_samples,
                           size_t *samples_read, uint32_t timeout_ms);

/**
 * Disable the I2S RX channel (stops clocking the microphone).
 */
esp_err_t mic_manager_stop(void);

/**
 * Delete the I2S channel and free resources.
 */
esp_err_t mic_manager_deinit(void);
