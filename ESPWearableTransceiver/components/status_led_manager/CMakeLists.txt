idf_component_register(SRCS "status_led_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver ws2812_manager freertos esp_timer)
