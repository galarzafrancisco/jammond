static const char *TAG = "dac";

/* -----------------------------------------------------------------------------------------------------------------------------

Main

----------------------------------------------------------------------------------------------------------------------------- */

#include "dac.h"
#include <stdio.h>
#include "esp_err.h"           // error checker
#include "esp_log.h"           // logger
#include "driver/i2s_std.h"    // I2S driver for the DAC
#include "freertos/FreeRTOS.h" // General OS stuff

/* -----------------------------------------------------------------------------------------------------------------------------

Config

----------------------------------------------------------------------------------------------------------------------------- */

#define I2S_PORT I2S_NUM_0 // Use I2S0
#define BCLK GPIO_NUM_14
#define DATA_OUT GPIO_NUM_13
#define WORD_SELECT GPIO_NUM_12

/* -----------------------------------------------------------------------------------------------------------------------------

Globals

----------------------------------------------------------------------------------------------------------------------------- */

i2s_chan_handle_t i2s_dac_tx_handle;

/* -----------------------------------------------------------------------------------------------------------------------------

Init DAC

----------------------------------------------------------------------------------------------------------------------------- */

void dac_init()
{

  // Step 1: Define the I2S channel configuration
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);

  // buffer_size = dma_frame_num * slot_num * slot_bit_width / 8
  chan_cfg.dma_desc_num = 6;    // default 6
  chan_cfg.dma_frame_num = 512; // default 240
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_dac_tx_handle, NULL));

  // Step 2: Define the standard mode configuration
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(CONFIG_UAC_SAMPLE_RATE),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {
          .mclk = I2S_GPIO_UNUSED, // PCM5102a doesn't need a clock
          .bclk = BCLK,
          .ws = WORD_SELECT,
          .dout = DATA_OUT,
          .din = I2S_GPIO_UNUSED, // Not used in TX mode
      },
  };

  // Step 3: Initialize the I2S channel with the standard mode configuration
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_dac_tx_handle, &std_cfg));

  // Step 4: Enable the I2S channel
  ESP_ERROR_CHECK(i2s_channel_enable(i2s_dac_tx_handle));
}

/* -----------------------------------------------------------------------------------------------------------------------------

DAC methods

----------------------------------------------------------------------------------------------------------------------------- */

/*
Writes a buffer of 16 bit samples.
This is a blocking call, meaning that if I2S buffers are busy
it will wait until they become available (or time out).
It returns an error if I2S is not ready.
It updates the number of samples written via q_samples_written
*/
esp_err_t dac_write(int16_t *samples_in_16b, size_t q_samples, size_t *q_samples_written)
{
  size_t q_bytes = q_samples * 2; // 16 bits is 2 bytes
  size_t bytes_written;
  esp_err_t err = i2s_channel_write(i2s_dac_tx_handle, samples_in_16b, q_bytes, &bytes_written, portMAX_DELAY);
  if (q_samples_written)
  {
    *q_samples_written = bytes_written / 2;
  }
  return err;
}