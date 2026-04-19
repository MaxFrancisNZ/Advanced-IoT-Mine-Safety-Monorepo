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
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    memcpy(broadcast_peer.peer_addr, ESPNOW_BROADCAST_ADDR, ESP_NOW_ETH_ALEN);
    broadcast_peer.channel = ESPNOW_CHANNEL;
    broadcast_peer.ifidx = WIFI_IF_STA;
    broadcast_peer.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_peer));

    ESP_LOGI(TAG, "ESP-NOW initialized");
}

static void espnow_to_uart_task(void *pvParameters)
{
    espnow_msg_t msg;

    while (1) {
        if (xQueueReceive(rx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            uart_write_bytes(UART_BRIDGE_PORT, msg.data, msg.len);
            uart_write_bytes(UART_BRIDGE_PORT, "\n", 1);

            ESP_LOGI(TAG,
                     "Forwarded %d bytes from %02X:%02X:%02X:%02X:%02X:%02X to UART",
                     msg.len,
                     msg.mac[0], msg.mac[1], msg.mac[2],
                     msg.mac[3], msg.mac[4], msg.mac[5]);
        }
    }
}

static void uart_to_espnow_task(void *pvParameters)
{
    uint8_t rx_byte = 0;
    char line_buf[MAX_JSON_LEN + 1];
    size_t line_len = 0;

    while (1) {
        int read_len = uart_read_bytes(UART_BRIDGE_PORT, &rx_byte, 1, pdMS_TO_TICKS(100));
        if (read_len <= 0) {
            continue;
        }

        if (rx_byte == '\r') {
            continue;
        }

        if (rx_byte == '\n') {
            if (line_len == 0) {
                continue;
            }

            // TODO: Re-enable UART->ESP-NOW forwarding when ready
            // esp_err_t err = esp_now_send(ESPNOW_BROADCAST_ADDR, (const uint8_t *)line_buf, line_len);
            // if (err != ESP_OK) {
            //     ESP_LOGE(TAG, "ESP-NOW send failed: %s", esp_err_to_name(err));
            // } else {
            //     line_buf[line_len] = '\0';
            //     ESP_LOGI(TAG, "Sent %u UART bytes over ESP-NOW: %s", (unsigned)line_len, line_buf);
            // }
            line_buf[line_len] = '\0';
            ESP_LOGW(TAG, "UART->ESP-NOW disabled, dropped %u bytes: %s", (unsigned)line_len, line_buf);

            line_len = 0;
            continue;
        }

        if (line_len < MAX_JSON_LEN) {
            line_buf[line_len++] = (char)rx_byte;
        } else {
            ESP_LOGW(TAG, "Dropping oversized UART line");
            line_len = 0;
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

    uart_bridge_init();
    wifi_init_sta();
    espnow_init();

    xTaskCreate(espnow_to_uart_task, "espnow_to_uart", 4096, NULL, 4, NULL);
    xTaskCreate(uart_to_espnow_task, "uart_to_espnow", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "ESP-NOW <-> UART bridge ready");
}
