static const char *TAG = "USB audio device";

/* -----------------------------------------------------------------------------------------------------------------------------

Libs

----------------------------------------------------------------------------------------------------------------------------- */

#include "freertos/FreeRTOS.h" // General OS stuff
#include "freertos/task.h"     // General OS stuff
#include <stdio.h>
#include "esp_err.h"   // error checker
#include "esp_log.h"   // logger
#include "math.h"      // To calculate sinewave
#include "esp_timer.h" // To time the dac

// My modules to make my life easier
#include "dac.h"                 // my module
#include "usb_audio_interface.h" // my module
#include "synth.h"               // my module

/* -----------------------------------------------------------------------------------------------------------------------------

PIN LAYOUT

----------------------------------------------------------------------------------------------------------------------------- */

/* -----------------------------------------------------------------------------------------------------------------------------

Globals

----------------------------------------------------------------------------------------------------------------------------- */
#define SAMPLE_RATE CONFIG_UAC_SAMPLE_RATE
#define FRAMES_SAMPLES_PER_STEP (SAMPLE_RATE / 1000 * 3) // 3ms @ 48KHz
#define AUDIO_TASK_STACK (SAMPLE_RATE / 10)              // ~4092 for 48KHz
#define SAMPLES_PER_STEP (FRAMES_SAMPLES_PER_STEP * 2)
#define USB_RING_BUFFER_SIZE (SAMPLE_RATE / 20) // 2400 @ 48KHz
static int16_t usb_ring_buffer[USB_RING_BUFFER_SIZE];
volatile size_t usb_ring_buffer_read_idx = 1; // start reading ahead of the writer
volatile size_t usb_ring_buffer_write_idx = 0;

// This callback is called by the USB driver when new USB audio data is available.
// 'data' points to the new samples (for example, in 16-bit or float format).
// For this example, assume the incoming data is in float format.
static esp_err_t usb_audio_callback(int16_t *samples_in_16b, size_t q_samples)
{

  for (size_t i = 0; i < q_samples; i++)
  {
    while (usb_ring_buffer_write_idx == usb_ring_buffer_read_idx)
    {
      vTaskDelay(10);
    }
    usb_ring_buffer[usb_ring_buffer_write_idx] = samples_in_16b[i];
    usb_ring_buffer_write_idx++;
    if (usb_ring_buffer_write_idx >= USB_RING_BUFFER_SIZE)
    {
      usb_ring_buffer_write_idx -= USB_RING_BUFFER_SIZE;
    }
  }
  return ESP_OK;
}

void grab_from_usb(int16_t *samples_in_16b, size_t q_samples)
{
  for (size_t i = 0; i < q_samples; i++)
  {
    samples_in_16b[i] = usb_ring_buffer[usb_ring_buffer_read_idx];
    usb_ring_buffer[usb_ring_buffer_read_idx] = 0; // clear buffer after reading
    usb_ring_buffer_read_idx++;
    if (usb_ring_buffer_read_idx >= USB_RING_BUFFER_SIZE)
    {
      usb_ring_buffer_read_idx -= USB_RING_BUFFER_SIZE;
    }
  }
}
// volatile uint32_t stereo_samples_sent = 0;
// static esp_err_t pipe_usb_audio_to_dac(int16_t *samples_in_16b, size_t q_samples)
// {

//   size_t q_samples_written;
//   ESP_ERROR_CHECK_WITHOUT_ABORT(dac_write(samples_in_16b, q_samples, &q_samples_written));

//   if (q_samples_written != q_samples)
//   {
//     ESP_LOGW(TAG, "Received %d samples from USB, wrote %d to I2S. Drift!", q_samples, q_samples_written);
//   }

//   stereo_samples_sent += q_samples_written / 2; // 2 channels (stereo)
//   // ESP_LOGI(TAG, "Received %d bytes from USB, wrote %d bytes to I2S. Samples: %d", len, bytes_written, num_samples);

//   return ESP_OK;
// }

/* -----------------------------------------------------------------------------------------------------------------------------

Audio Task

----------------------------------------------------------------------------------------------------------------------------- */

#define MEASURE_INTERVAL 1000

void audio_task()
{
  uint64_t total_wait_time_us = 0;
  size_t loop_counter = 0;

  int16_t output_buffer[SAMPLES_PER_STEP];
  int16_t usb_buffer[SAMPLES_PER_STEP];
  int16_t synth_buffer[SAMPLES_PER_STEP];

  // Init buffers
  for (size_t i = 0; i < SAMPLES_PER_STEP; i++)
  {
    output_buffer[i] = 0;
    usb_buffer[i] = 0;
    synth_buffer[i] = 0;
  }

  float synth_volume = 0;
  float usb_volume = 1;
  float master_volume = 1;
  while (1)
  {
    size_t samples_written = 0;

    // // Generate synth samples
    synth_make_samples(synth_buffer, FRAMES_SAMPLES_PER_STEP); // pass frames instead of samples because synth is mono

    // Read from USB
    grab_from_usb(usb_buffer, SAMPLES_PER_STEP);

    // Mix
    for (size_t i = 0; i < SAMPLES_PER_STEP; i++)
    {
      output_buffer[i] = (usb_buffer[i] * usb_volume +
                          synth_buffer[i] * synth_volume) *
                         master_volume;
    }
    // Measure the time spent in dac_write (which is blocking)
    uint64_t t_start = esp_timer_get_time();
    dac_write(output_buffer, SAMPLES_PER_STEP, &samples_written);
    uint64_t t_end = esp_timer_get_time();
    total_wait_time_us += (t_end - t_start);
    loop_counter++;

    // Every MEASURE_INTERVAL loops, log the total and average wait time
    if (loop_counter >= MEASURE_INTERVAL)
    {
      float avg_us_free = (float)total_wait_time_us / loop_counter;
      float us_per_cycle = 1000000.0F / CONFIG_UAC_SAMPLE_RATE * FRAMES_SAMPLES_PER_STEP;
      float cpu_usage = 1.0F - (avg_us_free / us_per_cycle);

      ESP_LOGI(TAG, "CPU usage %.1f%%", cpu_usage * 100);

      // Reset the counters for the next interval.
      loop_counter = 0;
      total_wait_time_us = 0;
    }
  }
}

/* -----------------------------------------------------------------------------------------------------------------------------

Main

----------------------------------------------------------------------------------------------------------------------------- */

void app_main(void)
{
  // Start DAC
  dac_init();

  // Init synth
  synth_init();

  // Start the USB driver to receive audio
  usb_audio_interface_config_t usb_audio_interface_config = {
      .freshly_baked_samples_cb = usb_audio_callback,
  };
  usb_audio_interface_init(&usb_audio_interface_config);

  xTaskCreatePinnedToCore(audio_task, "audio_task", AUDIO_TASK_STACK, NULL, 5, NULL, 1);
}
