static const char *TAG = "synth";

/* -----------------------------------------------------------------------------------------------------------------------------

Libs

----------------------------------------------------------------------------------------------------------------------------- */

#include <stdio.h>
#include "esp_err.h" // error checker
#include "esp_log.h" // logger
#include <math.h>    // for sin() and M_PI

/* -----------------------------------------------------------------------------------------------------------------------------

Config

----------------------------------------------------------------------------------------------------------------------------- */
#define WAVE_FREQUENCY 440
#define SAMPLE_RATE 48000
#define WAVE_TABLE_LENGTH 1024

/* -----------------------------------------------------------------------------------------------------------------------------

Globals

----------------------------------------------------------------------------------------------------------------------------- */
// 'step' determines how far to advance the read index on each sample.
// It is calculated so that one full period of the wave corresponds to SAMPLE_RATE / WAVE_FREQUENCY samples.
float step = 0;
float read_index = 0;
int16_t wave_table[WAVE_TABLE_LENGTH];

/* -----------------------------------------------------------------------------------------------------------------------------

Init Synth

----------------------------------------------------------------------------------------------------------------------------- */
void synth_init()
{
    // Calculate the step size.
    // For a full period, we need to traverse WAVE_TABLE_LENGTH entries.
    // The period in samples is SAMPLE_RATE / WAVE_FREQUENCY.
    // Therefore: step = WAVE_TABLE_LENGTH / (SAMPLE_RATE / WAVE_FREQUENCY)
    //               = WAVE_TABLE_LENGTH * WAVE_FREQUENCY / SAMPLE_RATE
    step = (float)WAVE_TABLE_LENGTH * ((float)WAVE_FREQUENCY / (float)SAMPLE_RATE);

    // Calculate the wave table (sine wave)
    for (size_t i = 0; i < WAVE_TABLE_LENGTH; i++)
    {
        double angle = 2.0 * M_PI * i / WAVE_TABLE_LENGTH;
        // Scale the sine wave to the range of int16_t [-32767, 32767]
        wave_table[i] = (int16_t)(32767 * sin(angle)) * 0.02; // lower volume
    }

    // Initialize the read index to the beginning of the table.
    read_index = 0;

    ESP_LOGI(TAG, "Synth initialized. Step size: %f", step);
}

/* -----------------------------------------------------------------------------------------------------------------------------

Synth methods

----------------------------------------------------------------------------------------------------------------------------- */

/**
 * @brief Fills the provided buffer with synthesized samples.
 *
 * This function generates q_samples samples by reading values from the wavetable.
 * The read_index is advanced by the calculated step each time, and it wraps around when
 * it exceeds the length of the wavetable.
 *
 * @param samples_in_16b Buffer to fill with 16-bit samples.
 * @param q_samples Number of samples to generate.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t synth_make_samples(int16_t *samples_in_16b, size_t q_frames)
{
    for (size_t frame_idx = 0; frame_idx < q_frames; frame_idx++)
    {
        // Get the current sample from the wavetable (using the integer part of read_index)
        size_t index = (size_t)read_index; // read_index is a float to keep good track of phase
        samples_in_16b[frame_idx * 2] = wave_table[index];
        samples_in_16b[frame_idx * 2 + 1] = wave_table[index];

        // Advance read_index by the step size
        read_index += step;

        // Wrap the read_index if it goes past the table length.
        if (read_index >= WAVE_TABLE_LENGTH)
        {
            read_index -= WAVE_TABLE_LENGTH;
        }
    }
    return ESP_OK;
}
