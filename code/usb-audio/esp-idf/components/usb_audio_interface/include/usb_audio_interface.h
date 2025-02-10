
#ifndef USB_AUDIO_INTERFACE_H
#define USB_AUDIO_INTERFACE_H
#include <stdio.h>
#include <stdint.h>
#include "esp_err.h"

typedef esp_err_t (*usb_audio_interface_output_cb_t)(int16_t *samples_in_16b, size_t q_samples);

typedef struct
{
  usb_audio_interface_output_cb_t freshly_baked_samples_cb; /*!< callback function that is expected to do something with incoming samples */
} usb_audio_interface_config_t;

esp_err_t usb_audio_interface_init(usb_audio_interface_config_t *config);

#endif // USB_AUDIO_INTERFACE_H
