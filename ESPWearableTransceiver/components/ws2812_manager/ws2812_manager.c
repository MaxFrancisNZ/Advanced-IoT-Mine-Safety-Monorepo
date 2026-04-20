#include "ws2812_manager.h"

#include <string.h>
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_log.h"

static const char *TAG = "ws2812";

/* WS2812 timing (nanoseconds) */
#define WS2812_T0H_NS 350
#define WS2812_T0L_NS 800
#define WS2812_T1H_NS 700
#define WS2812_T1L_NS 600
#define WS2812_RESET_US 280

/* RMT resolution – 10 MHz gives 100 ns per tick */
#define RMT_RESOLUTION_HZ 10000000

static rmt_channel_handle_t s_rmt_channel = NULL;
static rmt_encoder_handle_t s_encoder = NULL;
static gpio_num_t s_gpio_num = GPIO_NUM_NC;

/* ---- Simple bytes encoder using copy + raw encoding ---- */

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_handle_t bytes_encoder;
    rmt_encoder_handle_t copy_encoder;
    int state;
} ws2812_encoder_t;

static const rmt_symbol_word_t ws2812_reset_symbol = {
    .duration0 = WS2812_RESET_US * (RMT_RESOLUTION_HZ / 1000000) / 2,
    .level0 = 0,
    .duration1 = WS2812_RESET_US * (RMT_RESOLUTION_HZ / 1000000) / 2,
    .level1 = 0,
};

static size_t ws2812_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                            const void *primary_data, size_t data_size,
                            rmt_encode_state_t *ret_state)
{
    ws2812_encoder_t *ws = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (ws->state) {
    case 0: /* encode pixel data */
        encoded_symbols += ws->bytes_encoder->encode(ws->bytes_encoder, channel,
                                                     primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            *ret_state = (rmt_encode_state_t)RMT_ENCODING_MEM_FULL;
            return encoded_symbols;
        }
        /* fall through */
    case 1: /* send reset code */
        encoded_symbols += ws->copy_encoder->encode(ws->copy_encoder, channel,
                                                    &ws2812_reset_symbol,
                                                    sizeof(ws2812_reset_symbol),
                                                    &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws->state = RMT_ENCODING_RESET;
            *ret_state = (rmt_encode_state_t)RMT_ENCODING_COMPLETE;
        } else {
            *ret_state = (rmt_encode_state_t)RMT_ENCODING_MEM_FULL;
        }
        return encoded_symbols;
    }

    *ret_state = (rmt_encode_state_t)RMT_ENCODING_RESET;
    return encoded_symbols;
}

static esp_err_t ws2812_encoder_del(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *ws = __containerof(encoder, ws2812_encoder_t, base);
    rmt_del_encoder(ws->bytes_encoder);
    rmt_del_encoder(ws->copy_encoder);
    free(ws);
    return ESP_OK;
}

static esp_err_t ws2812_encoder_reset(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *ws = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encoder_reset(ws->bytes_encoder);
    rmt_encoder_reset(ws->copy_encoder);
    ws->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t ws2812_encoder_create(rmt_encoder_handle_t *ret_encoder)
{
    ws2812_encoder_t *ws = calloc(1, sizeof(ws2812_encoder_t));
    if (!ws) return ESP_ERR_NO_MEM;

    ws->base.encode = ws2812_encode;
    ws->base.del    = ws2812_encoder_del;
    ws->base.reset  = ws2812_encoder_reset;

    rmt_bytes_encoder_config_t bytes_config = {
        .bit0 = {
            .duration0 = WS2812_T0H_NS * RMT_RESOLUTION_HZ / 1000000000,
            .level0 = 1,
            .duration1 = WS2812_T0L_NS * RMT_RESOLUTION_HZ / 1000000000,
            .level1 = 0,
        },
        .bit1 = {
            .duration0 = WS2812_T1H_NS * RMT_RESOLUTION_HZ / 1000000000,
            .level0 = 1,
            .duration1 = WS2812_T1L_NS * RMT_RESOLUTION_HZ / 1000000000,
            .level1 = 0,
        },
        .flags.msb_first = true,
    };

    esp_err_t err = rmt_new_bytes_encoder(&bytes_config, &ws->bytes_encoder);
    if (err != ESP_OK) { free(ws); return err; }

    rmt_copy_encoder_config_t copy_config = {};
    err = rmt_new_copy_encoder(&copy_config, &ws->copy_encoder);
    if (err != ESP_OK) { rmt_del_encoder(ws->bytes_encoder); free(ws); return err; }

    *ret_encoder = &ws->base;
    return ESP_OK;
}

/* ---- Public API ---- */

esp_err_t ws2812_manager_init(gpio_num_t gpio_num)
{
    if (s_rmt_channel != NULL && s_encoder != NULL) {
        if (s_gpio_num == gpio_num) {
            return ESP_OK;
        }
        return ESP_ERR_INVALID_STATE;
    }

    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = gpio_num,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };

    esp_err_t err = rmt_new_tx_channel(&tx_cfg, &s_rmt_channel);
    if (err != ESP_OK) return err;

    err = ws2812_encoder_create(&s_encoder);
    if (err != ESP_OK) return err;

    err = rmt_enable(s_rmt_channel);
    if (err != ESP_OK) return err;

    s_gpio_num = gpio_num;
    ESP_LOGI(TAG, "WS2812 initialised on GPIO %d", gpio_num);
    return ESP_OK;
}

esp_err_t ws2812_manager_set_color(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!s_rmt_channel || !s_encoder) return ESP_ERR_INVALID_STATE;

    /* WS2812 expects GRB byte order */
    uint8_t grb[3] = { green, red, blue };

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    /* Reset encoder state before each transmit so repeated updates from the
     * status LED task don't leave the RMT encoder stuck between frames. */
    esp_err_t err = rmt_encoder_reset(s_encoder);
    if (err != ESP_OK) return err;

    err = rmt_transmit(s_rmt_channel, s_encoder, grb, sizeof(grb), &tx_config);
    if (err != ESP_OK) return err;

    return rmt_tx_wait_all_done(s_rmt_channel, -1);
}
