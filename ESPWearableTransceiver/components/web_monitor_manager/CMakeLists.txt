idf_component_register(SRCS "web_monitor_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_http_server esp_wifi esp_event esp_netif freertos log)
