static const char *TAG = "USB audio device";

/* -----------------------------------------------------------------------------------------------------------------------------

Libs

----------------------------------------------------------------------------------------------------------------------------- */

#include "freertos/FreeRTOS.h" // General OS stuff
#include "freertos/task.h"     // General OS stuff
#include <stdio.h>
#include "esp_err.h" // error checker
#include "esp_log.h" // logger
#include "math.h"    // To calculate sinewave

// My modules to make my life easier
#include "dac.h"                 // my module
#include "usb_audio_interface.h" // my module

/* -----------------------------------------------------------------------------------------------------------------------------

PIN LAYOUT

----------------------------------------------------------------------------------------------------------------------------- */

volatile uint32_t stereo_samples_sent = 0;
static esp_err_t pipe_usb_audio_to_dac(int16_t *samples_in_16b, size_t q_samples)
{

  size_t q_samples_written;
  ESP_ERROR_CHECK_WITHOUT_ABORT(dac_write(samples_in_16b, q_samples, &q_samples_written));

  if (q_samples_written != q_samples)
  {
    ESP_LOGW(TAG, "Received %d samples from USB, wrote %d to I2S. Drift!", q_samples, q_samples_written);
  }

  stereo_samples_sent += q_samples_written / 2; // 2 channels (stereo)
  // ESP_LOGI(TAG, "Received %d bytes from USB, wrote %d bytes to I2S. Samples: %d", len, bytes_written, num_samples);

  return ESP_OK;
}

/* -----------------------------------------------------------------------------------------------------------------------------

Main

----------------------------------------------------------------------------------------------------------------------------- */

void app_main(void)
{

  // Start DAC
  dac_init();

  // Start the USB driver to receive audio
  usb_audio_interface_config_t usb_audio_interface_config = {
      .freshly_baked_samples_cb = pipe_usb_audio_to_dac,
  };
  usb_audio_interface_init(&usb_audio_interface_config);
}
