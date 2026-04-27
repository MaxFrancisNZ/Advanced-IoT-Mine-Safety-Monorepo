idf_component_register(SRCS "ws2812_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_driver_gpio
                    PRIV_REQUIRES esp_driver_rmt)
