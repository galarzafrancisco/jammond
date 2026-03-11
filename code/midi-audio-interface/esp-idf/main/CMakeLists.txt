idf_component_register(
    SRCS "jammond-midi-audio.c"
    INCLUDE_DIRS "."
    REQUIRES esp_adc esp_driver_uart driver esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    CFG_TUD_AUDIO=1
    CFG_TUD_AUDIO_ENABLE_EP_OUT=1
    CFG_TUD_AUDIO_ENABLE_EP_IN=0
    CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP=0
    CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX=192
    CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ=4096
    CFG_TUD_AUDIO_CTRL_BUF_SZ=256
)
