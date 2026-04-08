#include "mic_manager.h"

#include <string.h>

#include "driver/i2s_std.h"
#include "esp_log.h"

static const char *TAG = "MIC_MGR";

static i2s_chan_handle_t rx_chan = NULL;

esp_err_t mic_manager_init(const mic_manager_config_t *config)
{
    if (rx_chan != NULL) {
        ESP_LOGW(TAG, "Already initialised");
        return ESP_ERR_INVALID_STATE;
    }

    /* Allocate an RX-only channel on I2S port 0 */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = 256;

    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(err));
        return err;
    }

    /*
     * SPH0645 configuration:
     *  - Philips (I2S) standard
     *  - 32-bit slot width (mic outputs 18-bit data MSB-justified in 32-bit frame)
     *  - Mono, left channel (SEL pin low / floating)
     */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config->sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                         I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = config->bck_gpio,
            .ws   = config->ws_gpio,
            .din  = config->din_gpio,
            .dout = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    err = i2s_channel_init_std_mode(rx_chan, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_rx failed: %s", esp_err_to_name(err));
        i2s_del_channel(rx_chan);
        rx_chan = NULL;
        return err;
    }

    ESP_LOGI(TAG, "Initialised: BCK=%d WS=%d DIN=%d rate=%"PRIu32,
             config->bck_gpio, config->ws_gpio, config->din_gpio, config->sample_rate);
    return ESP_OK;
}

esp_err_t mic_manager_start(void)
{
    if (rx_chan == NULL) return ESP_ERR_INVALID_STATE;
    return i2s_channel_enable(rx_chan);
}

esp_err_t mic_manager_read(int16_t *buffer, size_t max_samples,
                           size_t *samples_read, uint32_t timeout_ms)
{
    if (rx_chan == NULL) return ESP_ERR_INVALID_STATE;

    /*
     * The SPH0645 delivers 32-bit words per sample.
     * Read into a temporary 32-bit buffer, then convert to 16-bit.
     */
    size_t read_bytes = 0;
    size_t buf_bytes  = max_samples * sizeof(int32_t);

    /* Use stack for small reads, heap for large */
    int32_t stack_buf[64];
    int32_t *raw = stack_buf;
    bool heap_alloc = false;

    if (max_samples > 64) {
        raw = malloc(buf_bytes);
        if (raw == NULL) return ESP_ERR_NO_MEM;
        heap_alloc = true;
    }

    esp_err_t err = i2s_channel_read(rx_chan, raw, buf_bytes, &read_bytes, timeout_ms);
    if (err != ESP_OK) {
        if (heap_alloc) free(raw);
        *samples_read = 0;
        return err;
    }

    size_t n = read_bytes / sizeof(int32_t);

    /* Convert 32-bit left-justified data to 16-bit signed PCM.
     * SPH0645 places 18-bit data in the upper bits of the 32-bit word.
     * Shifting right by 16 keeps the top 16 bits, which is sufficient quality. */
    for (size_t i = 0; i < n; i++) {
        buffer[i] = (int16_t)(raw[i] >> 16);
    }

    *samples_read = n;
    if (heap_alloc) free(raw);
    return ESP_OK;
}

esp_err_t mic_manager_stop(void)
{
    if (rx_chan == NULL) return ESP_ERR_INVALID_STATE;
    return i2s_channel_disable(rx_chan);
}

esp_err_t mic_manager_deinit(void)
{
    if (rx_chan == NULL) return ESP_ERR_INVALID_STATE;
    esp_err_t err = i2s_del_channel(rx_chan);
    rx_chan = NULL;
    return err;
}
