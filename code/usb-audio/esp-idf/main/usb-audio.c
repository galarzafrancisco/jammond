
/* -----------------------------------------------------------------------------------------------------------------------------

Libs

----------------------------------------------------------------------------------------------------------------------------- */

#include <stdio.h>
#include "driver/i2s.h"
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

    i2s_write(I2S_PORT, sample, sizeof(sample), &bytes_written, portMAX_DELAY);

  }
}


/* -----------------------------------------------------------------------------------------------------------------------------

Setup

----------------------------------------------------------------------------------------------------------------------------- */

void i2s_init()
{
  i2s_config_t i2s_config = {
      .mode = I2S_MODE_MASTER | I2S_MODE_TX, // no Rx
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = BITS_PER_SAMPLE,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // Stereo
      .communication_format = I2S_COMM_FORMAT_I2S,
      .dma_buf_count = 6,
      .dma_buf_len = 1024,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1};

  i2s_pin_config_t pin_config = {
      .bck_io_num = 14,   // Bit Clock (BCK) green
      .data_out_num = 13, // Data Out (DOUT) yellow
      .ws_io_num = 12,    // Word Select (LRCK) orange
      .data_in_num = I2S_PIN_NO_CHANGE};

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

/* -----------------------------------------------------------------------------------------------------------------------------

Main

----------------------------------------------------------------------------------------------------------------------------- */

void app_main(void)
{
  i2s_init();
    while (1) {
        generate_sine_wave();
    }
}
