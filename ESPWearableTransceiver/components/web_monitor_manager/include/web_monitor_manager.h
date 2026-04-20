#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    const char *ap_ssid;
    const char *ap_password;
    uint8_t channel;
    uint8_t max_connections;
} web_monitor_manager_config_t;

esp_err_t web_monitor_manager_early_init(void);
esp_err_t web_monitor_manager_init(const web_monitor_manager_config_t *config);
void web_monitor_manager_finalize_startup_history(void);
bool web_monitor_manager_has_active_viewer(void);
