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
#include "tusb.h"

/* -----------------------------------------------------------------------------------------------------------------------------

PIN LAYOUT

----------------------------------------------------------------------------------------------------------------------------- */

#define I2S_PORT I2S_NUM_0 // Use I2S0
#define BITS_PER_SAMPLE 16 // PCM bit depth - THIS IS NOT A VARIABLE! DON'T CHANGE IT!
#define PI 3.14159265
#define SINE_WAVE_FREQ 220 // A4 note (440 Hz)

/* -----------------------------------------------------------------------------------------------------------------------------

I2S DAC

----------------------------------------------------------------------------------------------------------------------------- */
i2s_chan_handle_t i2s_dac_tx_handle;

/* -----------------------------------------------------------------------------------------------------------------------------

USB audio

----------------------------------------------------------------------------------------------------------------------------- */

#define OUTPUT_BUFFER_STEREO_SAMPLES 128
#define OUTPUT_BUFFER_LENGTH (OUTPUT_BUFFER_STEREO_SAMPLES * 2)
static float output_buffer[OUTPUT_BUFFER_LENGTH];
volatile uint16_t output_buffer_write_index = 0;
volatile uint16_t output_buffer_dac_read_index = 0;

static void init_output_buffer(void) {
  for (output_buffer_write_index = 0; output_buffer_write_index < OUTPUT_BUFFER_LENGTH; output_buffer_write_index++) {
    output_buffer[output_buffer_write_index] = 0;
  }
  output_buffer_write_index = 0;
  return;
}

volatile bool receiving_usb_audio = false;
volatile float usb_volume = 0;          // [0:1] multiplier
volatile float usb_pre_mute_volume = 0; // what was the volume before muting. Will return to this after unmuting.

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

static void uac_device_set_volume_cb(uint32_t volume, void *arg)
{
    // Volume is 0-100, we map it to 0.0 - 1.0 in a logarithmic way
    if (volume == 0) {
      usb_volume = 0.0f;
    } else {
      usb_volume = powf(10, (float)volume / 100.0f * 2.0f - 2.0f); // Maps 0-100 to ~0.01 - 1.0
    }
    ESP_LOGI(TAG, "Volume Set: %u -> Scaled: %f", volume, usb_volume);
}

// Invoked when device is mounted
static void uac_usb_ok_cb(void *arg)
{
  receiving_usb_audio = true;
  ESP_LOGI(TAG, "USB OK 🔥");
  // Mute audio for a few seconds to avoid weird sounds while the i2s driver restarts
  uac_device_set_mute_cb(1, NULL);
  ESP_ERROR_CHECK_WITHOUT_ABORT(i2s_channel_enable(i2s_dac_tx_handle));
  vTaskDelay(pdMS_TO_TICKS(1000));
  uac_device_set_mute_cb(1, NULL);
}

// Invoked when device is unmounted
static void uac_usb_disconnected_cb(void *arg)
{
  receiving_usb_audio = false;
  ESP_LOGI(TAG, "USB disconnected 💩");
  ESP_ERROR_CHECK_WITHOUT_ABORT(i2s_channel_disable(i2s_dac_tx_handle));
}

volatile uint32_t samples_sent = 0;
static esp_err_t uac_device_output_cb(uint8_t *buf, size_t len, void *arg)
{
  size_t bytes_written;

  // buf contains interleaved stereo samples in 16-bit PCM format
  int16_t *samples = (int16_t *)buf; // Treat buffer as an array of 16-bit integers
  size_t num_samples = len / 2; // 2 bytes for each 16 bit sample

  for (size_t i = 0; i < num_samples; i++)
  {
    float scaled_sample = samples[i] * usb_volume; // Apply volume
    samples[i] = (int16_t)scaled_sample;           // Convert back to int16
  }

  // Send samples to I2S
  i2s_channel_write(i2s_dac_tx_handle, samples, len, &bytes_written, portMAX_DELAY);

  if (bytes_written != len)
  {
    ESP_LOGW(TAG, "Received %d bytes from USB, wrote %d bytes to I2S. Drift!", len, bytes_written);
  }

  samples_sent += bytes_written / 2 / 2; // 2 bytes per channel (16 bits), 2 channels (stereo)
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

  for (int i = 0; i < CONFIG_UAC_SAMPLE_RATE; i++)
  {
    float theta = (2.0 * PI * SINE_WAVE_FREQ * i) / CONFIG_UAC_SAMPLE_RATE;
    int16_t value = (int16_t)(amplitude * sin(theta));

    // Stereo sample (L, R)
    sample[0] = value;
    sample[1] = value;

    ESP_ERROR_CHECK(i2s_channel_write(i2s_dac_tx_handle, sample, sizeof(sample), &bytes_written, portMAX_DELAY));
  }
}

/* -----------------------------------------------------------------------------------------------------------------------------

Setup

----------------------------------------------------------------------------------------------------------------------------- */

void init_i2s_dac_driver()
{

  // Step 1: Define the I2S channel configuration
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

  // buffer_size = dma_frame_num * slot_num * slot_bit_width / 8
  chan_cfg.dma_desc_num = 6;    // default 6
  chan_cfg.dma_frame_num = 256; // default 240
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_dac_tx_handle, NULL));

  // Step 2: Define the standard mode configuration
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(CONFIG_UAC_SAMPLE_RATE),
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
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_dac_tx_handle, &std_cfg));

  // Step 4: Enable the I2S channel
  ESP_ERROR_CHECK(i2s_channel_enable(i2s_dac_tx_handle));
}

void init_usb_uac_audio_device()
{
  uac_device_config_t config = {
      .output_cb = uac_device_output_cb, // receive audio from the host
      .input_cb = NULL,                  // transfer audio to the host
      .set_mute_cb = uac_device_set_mute_cb,
      .set_volume_cb = uac_device_set_volume_cb,
      .cb_ctx = NULL,
      .uac_usb_ok_cb = uac_usb_ok_cb,
      .uac_usb_disconnected_cb = uac_usb_disconnected_cb,
  };

  uac_device_init(&config);
}

/* -----------------------------------------------------------------------------------------------------------------------------

Main

----------------------------------------------------------------------------------------------------------------------------- */
void monitor_audio_sync(void *arg)
{
  while (1)
  {
    if (!receiving_usb_audio)
    {
      vTaskDelay(10);
      continue;
    }
    static uint32_t last_samples_sent = 0;
    uint32_t expected_samples = 48000; // We should have played this much in 1 sec
    uint32_t actual_samples_sent = samples_sent - last_samples_sent;

    ESP_LOGI(TAG, "Expected: %d, Actual Sent: %d, Diff: %d",
             expected_samples, actual_samples_sent, actual_samples_sent - expected_samples);

    last_samples_sent = samples_sent;
    vTaskDelay(pdMS_TO_TICKS(1000)); // Run every 1 second
  }
}

void app_main(void)
{
  // Fill the audio output buffer with zeros
  init_output_buffer();

  // Start the I2S driver to send audio to the DAC
  init_i2s_dac_driver();

  // Start the USB driver to receive audio
  init_usb_uac_audio_device();

  // monitoring
  xTaskCreate(monitor_audio_sync, "monitor_audio_sync", 4 * 1024, NULL, 1, NULL);
}
