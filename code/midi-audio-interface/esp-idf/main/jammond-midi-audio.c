static const char *TAG = "jammond-midi-audio";

#include <stdint.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tusb.h"

// -----------------------------
// MIDI + controls configuration
// -----------------------------

#define MIDI_UART            UART_NUM_2
#define MIDI_UART_RX         8
#define MIDI_BAUD            31250
#define MIDI_CABLE_NUM       0
#define MIDI_CHANNEL         0

#define MULTIPLEXER_IDX_0    9
#define MULTIPLEXER_IDX_1    10
#define MULTIPLEXER_IDX_2    11
#define MUX_ADC_UNIT         ADC_UNIT_1
#define MUX_ADC_ATTEN        ADC_ATTEN_DB_12
#define MUX_ADC_WIDTH        ADC_BITWIDTH_12
#define MUX1_ADC_CHANNEL     ADC_CHANNEL_3
#define MUX2_ADC_CHANNEL     ADC_CHANNEL_4

static adc_oneshot_unit_handle_t s_adc_handle;
static uint8_t s_analog_idx;
static uint8_t s_cached_cc[9];

static const uint8_t s_drawbar_cc_map[9] = {12, 13, 14, 15, 16, 17, 18, 19, 20};

// -----------------------------
// Audio output configuration
// -----------------------------

#define I2S_PORT             I2S_NUM_0
#define I2S_BCLK             GPIO_NUM_14
#define I2S_DOUT             GPIO_NUM_13
#define I2S_WS               GPIO_NUM_12

#define AUDIO_SAMPLE_RATE    48000
#define AUDIO_RING_SAMPLES   (AUDIO_SAMPLE_RATE / 8)

static int16_t s_audio_ring[AUDIO_RING_SAMPLES];
static volatile size_t s_audio_wr_idx;
static volatile size_t s_audio_rd_idx;
static i2s_chan_handle_t s_i2s_tx;

// ----------------------------------
// TinyUSB descriptors (MIDI for now)
// ----------------------------------

enum {
    ITF_NUM_MIDI = 0,
    ITF_NUM_MIDI_STREAMING,
    ITF_COUNT,
};

enum {
    EP_EMPTY = 0,
    EPNUM_MIDI,
};

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MIDI_DESC_LEN)

static const char s_lang_desc[] = {0x09, 0x04};
static const char *s_string_desc[] = {
    s_lang_desc,
    "Jammond",
    "Jammond MIDI+Audio",
    "jammond-midi-audio-01",
    "Jammond MIDI",
};

static const uint8_t s_cfg_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, TUSB_DESC_TOTAL_LEN, 0, 100),
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 4, EPNUM_MIDI, (0x80 | EPNUM_MIDI), 64),
};

// -----------------------------
// MIDI helpers
// -----------------------------

static inline void midi_send_3b(uint8_t status, uint8_t data1, uint8_t data2) {
    if (!tud_midi_mounted()) {
        return;
    }

    uint8_t packet[3] = {status, data1, data2};
    tud_midi_stream_write(MIDI_CABLE_NUM, packet, sizeof(packet));
}

static inline void midi_send_cc(uint8_t channel, uint8_t cc, uint8_t value) {
    midi_send_3b((uint8_t)(0xB0 | (channel & 0x0F)), cc, value);
}

static void midi_uart_forward_task(void *arg) {
    (void)arg;

    uint8_t byte;
    while (1) {
        int read = uart_read_bytes(MIDI_UART, &byte, 1, 0);
        if (read == 1) {
            if (tud_midi_mounted()) {
                uint8_t stream[1] = {byte};
                tud_midi_stream_write(MIDI_CABLE_NUM, stream, sizeof(stream));
            }
        } else {
            taskYIELD();
        }
    }
}

static void controls_read_task(void *arg) {
    (void)arg;

    while (1) {
        if (++s_analog_idx > 8) {
            s_analog_idx = 0;
        }

        gpio_set_level(MULTIPLEXER_IDX_0, (s_analog_idx & 0x01));
        gpio_set_level(MULTIPLEXER_IDX_1, ((s_analog_idx >> 1) & 0x01));
        gpio_set_level(MULTIPLEXER_IDX_2, ((s_analog_idx >> 2) & 0x01));

        vTaskDelay(pdMS_TO_TICKS(2));

        int value = 0;
        adc_channel_t channel = (s_analog_idx > 7) ? MUX2_ADC_CHANNEL : MUX1_ADC_CHANNEL;
        if (adc_oneshot_read(s_adc_handle, channel, &value) == ESP_OK) {
            uint8_t midi_value = (uint8_t)(value >> 5);
            if (midi_value != s_cached_cc[s_analog_idx]) {
                s_cached_cc[s_analog_idx] = midi_value;
                midi_send_cc(MIDI_CHANNEL, s_drawbar_cc_map[s_analog_idx], midi_value);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// -----------------------------
// Audio helpers
// -----------------------------

static inline size_t ring_next(size_t index) {
    size_t next = index + 1;
    if (next >= AUDIO_RING_SAMPLES) {
        next = 0;
    }
    return next;
}

static void audio_ring_push_samples(const int16_t *samples, size_t count) {
    for (size_t i = 0; i < count; i++) {
        size_t next = ring_next(s_audio_wr_idx);
        if (next == s_audio_rd_idx) {
            break;
        }
        s_audio_ring[s_audio_wr_idx] = samples[i];
        s_audio_wr_idx = next;
    }
}

static size_t audio_ring_pop_samples(int16_t *samples, size_t count) {
    size_t copied = 0;
    while (copied < count && s_audio_rd_idx != s_audio_wr_idx) {
        samples[copied++] = s_audio_ring[s_audio_rd_idx];
        s_audio_rd_idx = ring_next(s_audio_rd_idx);
    }

    while (copied < count) {
        samples[copied++] = 0;
    }

    return copied;
}

static void audio_out_task(void *arg) {
    (void)arg;

    int16_t block[256];
    while (1) {
        size_t samples = audio_ring_pop_samples(block, 256);
        size_t bytes_written = 0;
        i2s_channel_write(s_i2s_tx, block, samples * sizeof(int16_t), &bytes_written, portMAX_DELAY);
    }
}

static void dac_init(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 4;
    chan_cfg.dma_frame_num = 240;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_i2s_tx, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK,
            .ws = I2S_WS,
            .dout = I2S_DOUT,
            .din = I2S_GPIO_UNUSED,
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_i2s_tx));
}

// ---------------------------------------
// TODO: TinyUSB Audio callbacks (Phase 2)
// ---------------------------------------

// This is intentionally a local helper until Audio class callbacks are added.
// It lets the existing runtime path be validated with synthetic or forwarded data.
static void usb_audio_feed_pcm16(const int16_t *stereo_samples, size_t sample_count) {
    audio_ring_push_samples(stereo_samples, sample_count);
}

// -----------------------------
// Setup
// -----------------------------

static void setup_adc_mux(void) {
    adc_oneshot_unit_init_cfg_t adc_init = {
        .unit_id = MUX_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = MUX_ADC_ATTEN,
        .bitwidth = MUX_ADC_WIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, MUX1_ADC_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, MUX2_ADC_CHANNEL, &chan_cfg));

    gpio_reset_pin(MULTIPLEXER_IDX_0);
    gpio_reset_pin(MULTIPLEXER_IDX_1);
    gpio_reset_pin(MULTIPLEXER_IDX_2);
    gpio_set_direction(MULTIPLEXER_IDX_0, GPIO_MODE_OUTPUT);
    gpio_set_direction(MULTIPLEXER_IDX_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MULTIPLEXER_IDX_2, GPIO_MODE_OUTPUT);
}

static void setup_uart_midi(void) {
    uart_config_t uart_cfg = {
        .baud_rate = MIDI_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(MIDI_UART, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(MIDI_UART, UART_PIN_NO_CHANGE, MIDI_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_line_inverse(MIDI_UART, UART_SIGNAL_IRDA_RX_INV));
    ESP_ERROR_CHECK(uart_driver_install(MIDI_UART, 2048, 2048, 0, NULL, 0));
}

static void setup_usb(void) {
    tinyusb_config_t usb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = s_string_desc,
        .string_descriptor_count = sizeof(s_string_desc) / sizeof(s_string_desc[0]),
        .external_phy = false,
        .configuration_descriptor = s_cfg_desc,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&usb_cfg));
    ESP_LOGI(TAG, "TinyUSB started with MIDI descriptor. Audio descriptor wiring is next.");
}

void app_main(void) {
    memset((void *)s_cached_cc, 0, sizeof(s_cached_cc));
    s_analog_idx = 0;
    s_audio_wr_idx = 0;
    s_audio_rd_idx = 0;

    setup_adc_mux();
    setup_uart_midi();
    setup_usb();
    dac_init();

    xTaskCreate(midi_uart_forward_task, "midi_uart_forward_task", 4096, NULL, 4, NULL);
    xTaskCreate(controls_read_task, "controls_read_task", 4096, NULL, 3, NULL);
    xTaskCreatePinnedToCore(audio_out_task, "audio_out_task", 4096, NULL, 5, NULL, 1);

    // Keep at least one warm path for DAC verification until TinyUSB Audio callbacks are hooked.
    int16_t silence[128] = {0};
    while (1) {
        usb_audio_feed_pcm16(silence, 128);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
