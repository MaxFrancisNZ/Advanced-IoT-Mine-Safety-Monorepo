idf_component_register(SRCS "mic_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_driver_gpio log
                    PRIV_REQUIRES esp_driver_i2s)
