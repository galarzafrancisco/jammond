static const char *TAG = "USB audio device";

/* -----------------------------------------------------------------------------------------------------------------------------

Libs

----------------------------------------------------------------------------------------------------------------------------- */

#include "freertos/FreeRTOS.h" // General OS stuff
#include "freertos/task.h"     // General OS stuff
#include <stdio.h>
#include "esp_err.h"        // error checker
#include "esp_log.h"        // logger
#include "driver/i2s_std.h" // I2S driver for the DAC
#include "math.h"           // To calculate sinewave
#include "usb_device_uac.h" // USB audio

/* -----------------------------------------------------------------------------------------------------------------------------

PIN LAYOUT

----------------------------------------------------------------------------------------------------------------------------- */

#define I2S_PORT I2S_NUM_0 // Use I2S0
#define SAMPLE_RATE 44100  // Audio sample rate
#define BITS_PER_SAMPLE 16 // PCM bit depth - THIS IS NOT A VARIABLE! DON'T CHANGE IT!
#define PI 3.14159265
#define SINE_WAVE_FREQ 220 // A4 note (440 Hz)

/* -----------------------------------------------------------------------------------------------------------------------------

I2S DAC

----------------------------------------------------------------------------------------------------------------------------- */
i2s_chan_handle_t tx_handle;

/* -----------------------------------------------------------------------------------------------------------------------------

USB audio

----------------------------------------------------------------------------------------------------------------------------- */
static void uac_device_set_mute_cb(uint32_t mute, void *arg)
{
  ESP_LOGI(TAG, "uac_device_set_mute_cb: %" PRIu32 "", mute);
}

static void uac_device_set_volume_cb(uint32_t volume, void *arg)
{
  ESP_LOGI(TAG, "uac_device_set_volume_cb: %" PRIu32 "", volume);
}

static esp_err_t uac_device_output_cb(uint8_t *buf, size_t len, void *arg)
{
  size_t bytes_written;

  // buf contains interleaved stereo samples in 16-bit PCM format
  int16_t *samples = (int16_t *)buf;          // Treat buffer as an array of 16-bit integers
  size_t num_samples = len / sizeof(int16_t); // Convert bytes to number of samples

  // Send samples to I2S
  i2s_channel_write(tx_handle, samples, len, &bytes_written, portMAX_DELAY);

  // ESP_LOGI(TAG, "Received %d bytes from USB, wrote %d bytes to I2S. Samples: %d", len, bytes_written, num_samples);

  return ESP_OK;
}

/* -----------------------------------------------------------------------------------------------------------------------------

Helpers

----------------------------------------------------------------------------------------------------------------------------- */

void generate_sine_wave()
{
  size_t bytes_written;
  int16_t sample[2];
  int amplitude = 32767 / 6; // Max value for 16-bit audio (limit to 25% so I don't break my ears)

  for (int i = 0; i < SAMPLE_RATE; i++)
  {
    float theta = (2.0 * PI * SINE_WAVE_FREQ * i) / SAMPLE_RATE;
    int16_t value = (int16_t)(amplitude * sin(theta));

    // Stereo sample (L, R)
    sample[0] = value;
    sample[1] = value;

    i2s_channel_write(tx_handle, sample, sizeof(sample), &bytes_written, portMAX_DELAY);
  }
}

/* -----------------------------------------------------------------------------------------------------------------------------

Setup

----------------------------------------------------------------------------------------------------------------------------- */

void init_i2s_dac_driver()
{

  // Step 1: Define the I2S channel configuration
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

  chan_cfg.dma_desc_num = 10;
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));

  // Step 2: Define the standard mode configuration
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {
          .mclk = I2S_GPIO_UNUSED, // Use I2S_GPIO_UNUSED if not used
          .bclk = GPIO_NUM_14,
          .ws = GPIO_NUM_12,
          .dout = GPIO_NUM_13,
          .din = I2S_GPIO_UNUSED, // Not used in TX mode
      },
  };

  // Step 3: Initialize the I2S channel with the standard mode configuration
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));

  // Step 4: Enable the I2S channel
  ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
}

void init_usb_uac_audio_device()
{
  uac_device_config_t config = {
      .output_cb = uac_device_output_cb, // receive audio from the host
      .input_cb = NULL,                  // transfer audio to the host
      .set_mute_cb = uac_device_set_mute_cb,
      .set_volume_cb = uac_device_set_volume_cb,
      .cb_ctx = NULL,
  };

  uac_device_init(&config);
}

/* -----------------------------------------------------------------------------------------------------------------------------

Main

----------------------------------------------------------------------------------------------------------------------------- */

void app_main(void)
{
  init_i2s_dac_driver();
  init_usb_uac_audio_device();
  // while (1)
  // {
  //   generate_sine_wave();
  // }
}
