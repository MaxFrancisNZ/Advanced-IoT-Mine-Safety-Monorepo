#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint8_t peer_mac[6];
    uint8_t channel;
    bool encrypt;
    const char *log_tag;
} espnow_manager_config_t;

esp_err_t espnow_manager_init(const espnow_manager_config_t *config);
esp_err_t espnow_manager_send(const uint8_t *data, size_t len);
esp_err_t espnow_manager_send_and_wait(const uint8_t *data, size_t len, uint32_t timeout_ms, bool *out_delivered);
