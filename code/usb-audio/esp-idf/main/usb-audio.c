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
#include "ring_buffer.h"         // my module

/* -----------------------------------------------------------------------------------------------------------------------------

Buffers

----------------------------------------------------------------------------------------------------------------------------- */

#define OUTPUT_BUFFER_SIZE 256
static float output_buffer_memory[OUTPUT_BUFFER_SIZE];
ring_buffer_t output_buffer_handler;

volatile uint32_t stereo_samples_sent = 0;
static esp_err_t usb_to_buffer(float *float_sampels, size_t q_samples)
{
  for (size_t i = 0; i < q_samples; i++)
  {
    if (!ring_buffer_write(&output_buffer_handler, float_sampels[i]))
    {
      ESP_LOGW(TAG, "Ring buffer returned false. TODO!");
    }
  }
  return ESP_OK;
}

static esp_err_t pipe_usb_audio_to_dac(float *float_samples, size_t q_samples)
{

  // Convert float to int16
  int16_t samples_in_16b[q_samples];
  for (size_t i = 0; i < q_samples; i++)
  {
    samples_in_16b[i] = (int16_t)float_samples[i];
  }

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

#define SAMPLES_PER_BLOCK 96 // 1ms at 48KHz

void dac_output_task(void *arg)
{
  // Create a local buffer to hold one block of DAC samples.
  float float_samples[SAMPLES_PER_BLOCK];
  int16_t dac_buffer[SAMPLES_PER_BLOCK];

  while (1)
  {
    // Fill the DAC buffer with samples read from the ring buffer.
    size_t samples_read = ring_buffer_read_samples(&output_buffer_handler, float_samples, SAMPLES_PER_BLOCK);

    // Turn float into int16 for the DAC
    for (size_t i = 0; i < samples_read; i++)
    {
      // Convert the float sample (assumed range -1.0 to 1.0) to a 16-bit PCM sample.
      dac_buffer[i] = (int16_t)float_samples; // (the usb sample is a float but is in the range of a 16 int )
    }

    // Write the block of samples to the DAC.
    // This call blocks until the I2S hardware is ready to accept the data.
    size_t samples_written = 0;
    esp_err_t err = dac_write(dac_buffer, samples_read, &samples_written);
    if (err != ESP_OK)
    {
      ESP_LOGE(TAG, "dac_write error: %d", err);
    }
    else if (samples_written != SAMPLES_PER_BLOCK)
    {
      ESP_LOGW(TAG, "Only wrote %d samples out of %d", samples_written, SAMPLES_PER_BLOCK);
    }

    // Optional: Delay to allow other tasks to run or to adjust timing.
    // If dac_write is blocking properly according to the DAC rate,
    // this delay may be unnecessary.
    // vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/* -----------------------------------------------------------------------------------------------------------------------------

Main

----------------------------------------------------------------------------------------------------------------------------- */

void app_main(void)
{

  // Init buffers
  ring_buffer_init(&output_buffer_handler, output_buffer_memory, OUTPUT_BUFFER_SIZE);

  // Start DAC
  dac_init();

  // Start the USB driver to receive audio
  usb_audio_interface_config_t usb_audio_interface_config = {
      // .freshly_baked_samples_cb = pipe_usb_audio_to_dac,
      .freshly_baked_samples_cb = usb_to_buffer,
  };
  usb_audio_interface_init(&usb_audio_interface_config);

  // Create the DAC output task
  xTaskCreate(dac_output_task, "DAC Output", 4096, NULL, tskIDLE_PRIORITY + 2, NULL); // 2048 overflows
}
