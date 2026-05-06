#include "espnow_manager.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"

static const char *TAG = "ESPNOW";
static uint8_t s_peer_mac[6] = {0};
static SemaphoreHandle_t s_send_done_sem = NULL;
static esp_now_send_status_t s_last_send_status = ESP_NOW_SEND_FAIL;
static espnow_manager_recv_cb_t s_recv_cb = NULL;
static void *s_recv_cb_context = NULL;

static void wifi_init_sta(uint8_t channel)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    wifi_mode_t mode = WIFI_MODE_NULL;
    err = esp_wifi_get_mode(&mode);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK(err);
    }

    if (mode == WIFI_MODE_NULL) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    } else if (mode == WIFI_MODE_AP) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    }

    err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "Wi-Fi ready for ESPNOW on channel %u (mode=%d)", channel, mode);
}

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    if (tx_info == NULL) {
        ESP_LOGW(TAG, "Send callback with NULL tx_info");
        return;
    }

    const uint8_t *mac_addr = tx_info->des_addr;
    ESP_LOGI(TAG,
             "Send to %02X:%02X:%02X:%02X:%02X:%02X -> %s",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5],
             status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");

    s_last_send_status = status;
    if (s_send_done_sem != NULL) {
        xSemaphoreGive(s_send_done_sem);
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info == NULL || recv_info->src_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGW(TAG, "Receive callback received invalid arguments");
        return;
    }

    if (s_recv_cb != NULL) {
        s_recv_cb(recv_info->src_addr, data, len, s_recv_cb_context);
    }
}

esp_err_t espnow_manager_init(const espnow_manager_config_t *config)
{
    if (config == NULL || config->log_tag == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    TAG = config->log_tag;
    memcpy(s_peer_mac, config->peer_mac, sizeof(s_peer_mac));

    if (s_send_done_sem == NULL) {
        s_send_done_sem = xSemaphoreCreateBinary();
        if (s_send_done_sem == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    wifi_init_sta(config->channel);

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, config->peer_mac, sizeof(peer.peer_addr));
    peer.channel = config->channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = config->encrypt;

    return esp_now_add_peer(&peer);
}

esp_err_t espnow_manager_send(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_now_send(s_peer_mac, data, len);
}

esp_err_t espnow_manager_send_and_wait(const uint8_t *data, size_t len, uint32_t timeout_ms, bool *out_delivered)
{
    if (data == NULL || len == 0U || out_delivered == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_send_done_sem == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_send_done_sem, 0);

    esp_err_t err = espnow_manager_send(data, len);
    if (err != ESP_OK) {
        *out_delivered = false;
        return err;
    }

    if (xSemaphoreTake(s_send_done_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        *out_delivered = false;
        return ESP_ERR_TIMEOUT;
    }

    *out_delivered = (s_last_send_status == ESP_NOW_SEND_SUCCESS);
    return ESP_OK;
}

esp_err_t espnow_manager_register_recv_cb(espnow_manager_recv_cb_t callback, void *context)
{
    s_recv_cb = callback;
    s_recv_cb_context = context;
    return ESP_OK;
}
