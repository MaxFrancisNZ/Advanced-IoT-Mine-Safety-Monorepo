#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "driver/uart.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"

#define ESPNOW_CHANNEL 1
#define MAX_JSON_LEN   ESP_NOW_MAX_DATA_LEN_V2

#define UART_BRIDGE_PORT      UART_NUM_1
#define UART_BRIDGE_TX_PIN    17
#define UART_BRIDGE_RX_PIN    16
#define UART_BRIDGE_BAUD_RATE 115200
#define UART_RX_BUF_SIZE      ESP_NOW_MAX_DATA_LEN_V2

static const uint8_t ESPNOW_BROADCAST_ADDR[ESP_NOW_ETH_ALEN] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

static const char *TAG = "ESPNOW_BASE";

typedef struct {
    uint8_t mac[6];
    int len;
    char data[MAX_JSON_LEN + 1];
} espnow_msg_t;

static QueueHandle_t rx_queue = NULL;

static void uart_bridge_init(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate = UART_BRIDGE_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_BRIDGE_PORT, UART_RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_BRIDGE_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_BRIDGE_PORT,
                                 UART_BRIDGE_TX_PIN,
                                 UART_BRIDGE_RX_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG,
             "UART bridge ready on UART%d TX=%d RX=%d @ %d baud",
             UART_BRIDGE_PORT,
             UART_BRIDGE_TX_PIN,
             UART_BRIDGE_RX_PIN,
             UART_BRIDGE_BAUD_RATE);
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "Wi-Fi STA started on channel %d", ESPNOW_CHANNEL);
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info == NULL || data == NULL || len <= 0) {
        return;
    }

    espnow_msg_t msg = {0};

    memcpy(msg.mac, recv_info->src_addr, 6);

    if (len > MAX_JSON_LEN) {
        len = MAX_JSON_LEN;
    }

    memcpy(msg.data, data, len);
    msg.data[len] = '\0';
    msg.len = len;

    xQueueSend(rx_queue, &msg, 0);
}

static void espnow_init(void)
{
    esp_now_peer_info_t broadcast_peer = {0};

    ESP_ERROR_CHECK(esp_now_init());

    memcpy(broadcast_peer.peer_addr, ESPNOW_BROADCAST_ADDR, ESP_NOW_ETH_ALEN);
    broadcast_peer.channel = ESPNOW_CHANNEL;
    broadcast_peer.ifidx = WIFI_IF_STA;
    broadcast_peer.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_peer));

    ESP_LOGI(TAG, "ESP-NOW initialized (recv callback not yet registered)");
}

static void espnow_start_receiving(void)
{
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_LOGI(TAG, "ESP-NOW recv callback registered");
}

static void espnow_to_uart_task(void *pvParameters)
{
    espnow_msg_t msg;

    while (1) {
        if (xQueueReceive(rx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            uart_write_bytes(UART_BRIDGE_PORT, msg.data, msg.len);
            uart_write_bytes(UART_BRIDGE_PORT, "\n", 1);

            ESP_LOGI(TAG,
                     "Forwarded %d bytes from %02X:%02X:%02X:%02X:%02X:%02X to UART\n%s",
                     msg.len,
                     msg.mac[0], msg.mac[1], msg.mac[2],
                     msg.mac[3], msg.mac[4], msg.mac[5],
                     msg.data);
        }
    }
}

/* Pi -> wearable command framing: [0xAA 0x55 len_lo len_hi payload crc8].
 * payload is forwarded verbatim as an ESP-NOW broadcast so the targeted
 * wearable receives it. crc8 is computed over len_lo..end-of-payload using
 * polynomial 0x07 (CRC-8/CCITT). */
#define FRAME_SYNC1   0xAA
#define FRAME_SYNC2   0x55
#define FRAME_MAX_PAYLOAD ESP_NOW_MAX_DATA_LEN_V2

static uint8_t crc8_07(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

typedef enum {
    FRAME_STATE_SYNC1 = 0,
    FRAME_STATE_SYNC2,
    FRAME_STATE_LEN_LO,
    FRAME_STATE_LEN_HI,
    FRAME_STATE_PAYLOAD,
    FRAME_STATE_CRC,
} frame_state_t;

static void uart_to_espnow_task(void *pvParameters)
{
    (void)pvParameters;

    uint8_t rx_buf[64];
    static uint8_t payload[FRAME_MAX_PAYLOAD];
    static uint8_t header[2];  /* len_lo, len_hi for CRC scope */

    frame_state_t state = FRAME_STATE_SYNC1;
    size_t expected_len = 0;
    size_t got_len = 0;

    while (1) {
        int n = uart_read_bytes(UART_BRIDGE_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(100));
        if (n <= 0) {
            continue;
        }

        for (int i = 0; i < n; i++) {
            uint8_t b = rx_buf[i];

            switch (state) {
            case FRAME_STATE_SYNC1:
                if (b == FRAME_SYNC1) {
                    state = FRAME_STATE_SYNC2;
                }
                break;
            case FRAME_STATE_SYNC2:
                if (b == FRAME_SYNC2) {
                    state = FRAME_STATE_LEN_LO;
                } else if (b == FRAME_SYNC1) {
                    /* stay armed on the new SYNC1 */
                    state = FRAME_STATE_SYNC2;
                } else {
                    state = FRAME_STATE_SYNC1;
                }
                break;
            case FRAME_STATE_LEN_LO:
                header[0] = b;
                state = FRAME_STATE_LEN_HI;
                break;
            case FRAME_STATE_LEN_HI:
                header[1] = b;
                expected_len = (size_t)header[0] | ((size_t)header[1] << 8);
                if (expected_len == 0 || expected_len > FRAME_MAX_PAYLOAD) {
                    ESP_LOGW(TAG, "Frame length out of range: %u", (unsigned)expected_len);
                    state = FRAME_STATE_SYNC1;
                    break;
                }
                got_len = 0;
                state = FRAME_STATE_PAYLOAD;
                break;
            case FRAME_STATE_PAYLOAD:
                payload[got_len++] = b;
                if (got_len == expected_len) {
                    state = FRAME_STATE_CRC;
                }
                break;
            case FRAME_STATE_CRC: {
                /* CRC covers len_lo, len_hi, then payload. */
                uint8_t crc = crc8_07(header, sizeof(header));
                /* Continue the CRC over the payload */
                for (size_t k = 0; k < expected_len; k++) {
                    uint8_t bb = payload[k];
                    crc ^= bb;
                    for (int j = 0; j < 8; j++) {
                        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
                    }
                }
                if (crc != b) {
                    ESP_LOGW(TAG, "Frame CRC mismatch (got 0x%02X, want 0x%02X, len=%u)",
                             b, crc, (unsigned)expected_len);
                } else {
                    esp_err_t err = esp_now_send(ESPNOW_BROADCAST_ADDR, payload, expected_len);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "ESP-NOW broadcast failed: %s", esp_err_to_name(err));
                    } else {
                        ESP_LOGI(TAG, "Forwarded %u-byte command to wearables (msg_type=0x%02X)",
                                 (unsigned)expected_len, payload[0]);
                    }
                }
                state = FRAME_STATE_SYNC1;
                break;
            }
            }
        }
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    rx_queue = xQueueCreate(10, sizeof(espnow_msg_t));
    if (rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }

    // Bring up the UART bridge first so the consumer side of the queue
    // is ready before we accept any ESP-NOW packets.
    uart_bridge_init();

    wifi_init_sta();
    espnow_init();

    // Start the forwarding task before registering the ESP-NOW recv
    // callback so no incoming packets land in the queue undrained.
    xTaskCreate(espnow_to_uart_task, "espnow_to_uart", 4096, NULL, 4, NULL);
    xTaskCreate(uart_to_espnow_task, "uart_to_espnow", 4096, NULL, 4, NULL);

    espnow_start_receiving();

    ESP_LOGI(TAG, "ESP-NOW <-> UART bridge ready");
}
