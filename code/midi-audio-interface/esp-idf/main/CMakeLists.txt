idf_component_register(
    SRCS "jammond-midi-audio.c"
    INCLUDE_DIRS "."
    REQUIRES esp_adc esp_driver_uart driver esp_timer
)
