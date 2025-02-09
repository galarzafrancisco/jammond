
/* -----------------------------------------------------------------------------------------------------------------------------

Libs

----------------------------------------------------------------------------------------------------------------------------- */

#include <stdio.h>
#include "driver/i2s_std.h"
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* -----------------------------------------------------------------------------------------------------------------------------

PIN LAYOUT

----------------------------------------------------------------------------------------------------------------------------- */

#define I2S_PORT I2S_NUM_0 // Use I2S0
#define SAMPLE_RATE 44100  // Audio sample rate
#define BITS_PER_SAMPLE 16 // PCM bit depth
#define PI 3.14159265
#define SINE_WAVE_FREQ 220 // A4 note (440 Hz)

/* -----------------------------------------------------------------------------------------------------------------------------

Helpers

----------------------------------------------------------------------------------------------------------------------------- */
i2s_chan_handle_t tx_handle;

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

void i2s_init()
{

  // Step 1: Define the I2S channel configuration
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

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

/* -----------------------------------------------------------------------------------------------------------------------------

Main

----------------------------------------------------------------------------------------------------------------------------- */

void app_main(void)
{
  i2s_init();
  while (1)
  {
    generate_sine_wave();
  }
}
