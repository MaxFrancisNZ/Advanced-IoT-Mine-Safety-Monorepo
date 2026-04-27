idf_component_register(SRCS "dht_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_driver_gpio esp_timer)
