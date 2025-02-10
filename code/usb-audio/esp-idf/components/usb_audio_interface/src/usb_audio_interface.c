static const char *TAG = "usb_audio_interface";

/* -----------------------------------------------------------------------------------------------------------------------------

Main

----------------------------------------------------------------------------------------------------------------------------- */

#include "usb_audio_interface.h"
#include "usb_device_uac.h"    // USB audio
#include "freertos/FreeRTOS.h" // General OS stuff
#include "esp_log.h"           // logger
#include "math.h"              // to calculate log volume curve

/* -----------------------------------------------------------------------------------------------------------------------------

Globals

----------------------------------------------------------------------------------------------------------------------------- */

static usb_audio_interface_output_cb_t freshly_baked_samples_cb;
static volatile float usb_volume = 0;          // [0:1] multiplier
static volatile float usb_pre_mute_volume = 0; // what was the volume before muting. Will return to this after unmuting.

/* -----------------------------------------------------------------------------------------------------------------------------

Callbacks to react to USB events

----------------------------------------------------------------------------------------------------------------------------- */

// Adjust volume of USB stream
static void uac_device_set_volume_cb(uint32_t volume, void *arg)
{
  // Volume is 0-100. Map it to 0.0 - 1.0 logarithmically.
  if (volume == 0)
  {
    usb_volume = 0.0f;
  }
  else
  {
    usb_volume = powf(10, (float)volume / 100.0f * 2.5f - 2.5f); // Maps 0-100 to ~0.003 - 1.0
  }
  ESP_LOGI(TAG, "Volume Set: %u -> Scaled: %f", volume, usb_volume);
}

// Mute USB stream
static void uac_device_set_mute_cb(uint32_t mute, void *arg)
{
  if (mute)
  {
    usb_pre_mute_volume = usb_volume;
    usb_volume = 0;
  }
  else
  {
    usb_volume = usb_pre_mute_volume;
  }
  ESP_LOGI(TAG, "uac_device_set_mute_cb: %" PRIu32 "", mute);
}

/*
 Handle incoming samples (Mac > USB > ESP)
 This callback will be passed to TinyUSB.
 It's basically a wrapper around the callback provided by the user,
 but with some nice stuff like:
 - transforms bytes to 16 bit samples
 - applies volume adjustment
 */
static esp_err_t uac_device_output_cb(uint8_t *buffer_8b, size_t q_bytes, void *arg)
{
  // buf contains interleaved stereo samples in 16-bit PCM format
  int16_t *samples_in_16b = (int16_t *)buffer_8b; // Treat buffer as an array of 16-bit integers
  size_t q_samples = q_bytes / 2;                 // 2 bytes for each 16 bit sample

  if (freshly_baked_samples_cb)
  {
    for (size_t i = 0; i < q_samples; i++)
    {
      float scaled_sample = samples_in_16b[i] * usb_volume; // Apply volume
      samples_in_16b[i] = (int16_t)scaled_sample;           // Convert back to int16
    }
    return freshly_baked_samples_cb(samples_in_16b, q_samples);
  }
  return ESP_OK;
}

/* -----------------------------------------------------------------------------------------------------------------------------

Init USB audio interface

----------------------------------------------------------------------------------------------------------------------------- */

esp_err_t usb_audio_interface_init(usb_audio_interface_config_t *config)
{
  freshly_baked_samples_cb = config->freshly_baked_samples_cb;

  uac_device_config_t uac_device_config = {
      .output_cb = uac_device_output_cb, // receive audio from the host
      .input_cb = NULL,                  // transfer audio to the host
      .set_mute_cb = uac_device_set_mute_cb,
      .set_volume_cb = uac_device_set_volume_cb,
      .cb_ctx = NULL,
  };

  return uac_device_init(&uac_device_config);
}