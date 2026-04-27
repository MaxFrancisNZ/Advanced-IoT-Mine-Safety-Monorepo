idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash spiffs driver i2c_manager espnow_manager led_manager dht_manager battery_probe mic_manager adpcm ws2812_manager status_led_manager web_monitor_manager)
