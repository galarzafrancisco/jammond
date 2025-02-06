#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2s.h"

/*
Audio config
*/
#define SAMPLE_RATE 44100

/*
Tonewheel generator
*/
#define OCTAVES 8
#define NOTES 12
#define Q_WHEELS 96
#define SINE_LENGTH 1024  // how many samples we use to store a sinewave
#define M_PI 3.14159265358979323846

float wheels[Q_WHEELS];            // Value of each tonewheel
float sine_wave[SINE_LENGTH + 3];  // Array to store sine wave tables for all notes
float idx[Q_WHEELS];               // Indexes for each wheel
float idx_step[Q_WHEELS];          // Index increment for each sample for each wheel

// A4 is 440. MIDI note 69
#define A4 440.00
#define A4_MIDI 69

/*
MIDI
*/
float drawbars[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // range 0-1. Will be used to multiply tonewheels
bool keys_active[61];
int keys_drawbar_wheel[61][9];

// Makes a map that we can index by key number (C1 = 0) (or MIDI note - 24) and drawbar
// and returns the number of wheel active
void populate_keys_drawbar_wheel() {
  for (int key = 0; key < 61; key++) {

    // drawbar 0: 16' is -24
    keys_drawbar_wheel[key][0] = key - 24;

    // drawbar 1: 5 1/3' is -5
    keys_drawbar_wheel[key][1] = key - 5;

    // drawbar 2: 8' is -12
    keys_drawbar_wheel[key][2] = key - 12;

    // drawbar 3: 4' is 0
    keys_drawbar_wheel[key][3] = key;

    // drawbar 4: 2 2/3' is +7
    keys_drawbar_wheel[key][4] = key + 7;

    // drawbar 5: 2' is +12
    keys_drawbar_wheel[key][5] = key + 12;

    // drawbar 6: 1 3/5's is +16
    keys_drawbar_wheel[key][6] = key + 16;

    // drawbar 7: 1 1/3' is +19
    keys_drawbar_wheel[key][7] = key + 19;

    // drawbar 8: 1' is +24
    keys_drawbar_wheel[key][8] = key + 24;

    // Handle overflow
    for (int drawbar = 0; drawbar < 9; drawbar++) {
      // If the wheel is < 0, increase it an octave
      while (keys_drawbar_wheel[key][drawbar] < 0) {
        keys_drawbar_wheel[key][drawbar] += 12;
      }
      // If the wheel is >= 96, decrease it an octave
      while (keys_drawbar_wheel[key][drawbar] > 95) {
        keys_drawbar_wheel[key][drawbar] -= 12;
      }
    }
  }
}

void populate_sine_array() {
  for (int i = 0; i < (SINE_LENGTH + 3); i++) {
    sine_wave[i] = sin(2 * M_PI * i / SINE_LENGTH);
  }
}

float calculate_note_frequency(int note) {
  return (float)A4 * pow(2, (note - A4_MIDI) / 12.0);
}

void populate_keys_active() {
  for (int i = 0; i < 61; i++) {
    keys_active[i] = true;
  }
}

void populate_idx() {
  // Wheels start at C1 (32.703Hz) which is MIDI note 24
  for (int i = 0; i < Q_WHEELS; i++) {
    float wheel_frequency = calculate_note_frequency(i + 24);
    Serial.print("Wheel ");
    Serial.print(i);
    Serial.print(" - note ");
    Serial.print(i + 24);
    Serial.print(": ");
    Serial.print(wheel_frequency);
    Serial.println("Hz");
    idx[i] = 0;
    idx_step[i] = (float)SINE_LENGTH * wheel_frequency / SAMPLE_RATE;
  }
}

void init_wheels() {
  for (int i = 0; i < Q_WHEELS; i++) {
    wheels[i] = 0;
  }
}

// Generates the 96 tones for 1 sample
void generate_tonewheel() {
  for (int w = 0; w < Q_WHEELS; w++) {
    // Increment index
    idx[w] = idx[w] + idx_step[w];
    if (idx[w] >= SINE_LENGTH) {
      idx[w] = idx[w] - SINE_LENGTH;
    }
    int idx_int = (int)idx[w];
    float idx_frac = idx[w] - idx_int;

    // Read sample
    wheels[w] = sine_wave[idx_int] * (1 - idx_frac) + sine_wave[idx_int + 1] * idx_frac;
  }
}

// Take in the keys that are on, the current drawbar settings, and make the sound
float generate_sound() {
  float output = 0;
  for (int d = 0; d < 9; d++) {
    for (int k = 0; k < 61; k++) {
      if (keys_active[k]) {
        output += wheels[keys_drawbar_wheel[k][d]] * drawbars[d];
      }
    }
  }
  return output;
}


TaskHandle_t TonewheelTask;

void setup() {
  Serial.begin(115200);
  populate_sine_array();
  populate_idx();
  populate_keys_drawbar_wheel();
  init_wheels();
  populate_keys_active();

  /*
  I2S from https://github.com/PilnyTomas/issues/blob/4b5315b5547165c9f080fd3e993dce24bf0fe7bf/issue_7252B/issue_7252B.ino
  found via https://forum.arduino.cc/t/arduino-with-esp32-i2s-write-and-dacwrite/1083733/10
  */
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = (i2s_bits_per_sample_t)16,
    .channel_format = (i2s_channel_fmt_t)I2S_COMM_FORMAT_STAND_I2S,
    .communication_format = I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 6,
    .dma_buf_len = 256,
    .use_apll = 1
  };
  if (ESP_OK != i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL)) {  //install and start i2s driver
    log_e("err install");
    while (1) { vTaskDelay(100); }
  }
  //i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN); // 25 + 26
  //i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN); // 25
  i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN);  // 26

  xTaskCreatePinnedToCore(
    TonewheelCode,  /* Function to implement the task */
    "Tonewheel",    /* Name of the task */
    10000,          /* Stack size in words */
    NULL,           /* Task input parameter */
    1,              /* Priority of the task */
    &TonewheelTask, /* Task handle. */
    0);             /* Core where the task should run (setup() and void() run on core 1, so core 0 is free) */
}


void TonewheelCode(void* parameter) {
  for (;;) {
    unsigned long start = millis();
    for (int s = 0; s < SAMPLE_RATE; s++) {
      generate_tonewheel();
      generate_sound();
    }
    unsigned long duration = millis() - start;  // duration to calculate 441000 sample in milliseconds

    float capable_sr = 1000.0 / duration * SAMPLE_RATE;  // 10^3 milliseconds per second

    Serial.print("[");
    Serial.print(xPortGetCoreID());
    Serial.print("] Calculated 44100 sample in ");
    Serial.print(duration);
    Serial.print("us. That means we can target a sample rate of ");
    Serial.print(capable_sr / 1000);
    Serial.print("KHz. We are using ");
    Serial.print(240000000.0 / capable_sr);
    Serial.println(" clock cycles per sample");
    delay(5000);
  }
}

void loop() {

  // Time how long it takes to generate 1 sample
  unsigned long start = millis();
  for (int s = 0; s < SAMPLE_RATE; s++) {
    generate_tonewheel();
    float sound = generate_sound();
    size_t bytes_written;
    i2s_write( I2S_NUM_0, data, BYTES_TO_WRITE, &bytes_written, portMAX_DELAY)
  }
  unsigned long duration = millis() - start;  // duration to calculate 441000 sample in milliseconds

  float capable_sr = 1000.0 / duration * SAMPLE_RATE;  // 10^3 milliseconds per second

  Serial.print("[");
  Serial.print(xPortGetCoreID());
  Serial.print("] Calculated 44100 sample in ");
  Serial.print(duration);
  Serial.print("us. That means we can target a sample rate of ");
  Serial.print(capable_sr / 1000);
  Serial.print("KHz. We are using ");
  Serial.print(240000000.0 / capable_sr);
  Serial.println(" clock cycles per sample");
  delay(5000);
}