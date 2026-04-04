idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash driver i2c_manager espnow_manager led_manager dht_manager)
