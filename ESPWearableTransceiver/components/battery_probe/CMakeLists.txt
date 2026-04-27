idf_component_register(SRCS "battery_probe.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_adc driver esp_driver_gpio)
