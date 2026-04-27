idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_wifi nvs_flash esp_event esp_netif
                    PRIV_REQUIRES esp_driver_uart)
