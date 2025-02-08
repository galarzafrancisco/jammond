idf_component_register(SRCS "jammond-controller.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_timer
                    REQUIRES esp_adc esp_driver_uart
                    )
