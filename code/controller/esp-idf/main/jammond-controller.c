const static char *TAG = "Jammond controller";

/* -----------------------------------------------------------------------------------------------------------------------------

Libs

----------------------------------------------------------------------------------------------------------------------------- */

#include <stdlib.h>
#include <stdio.h>
#include "esp_adc/adc_oneshot.h" // adc
#include "esp_log.h"             // log
#include "freertos/FreeRTOS.h"   // delays and stuff
#include "freertos/task.h"       // delays and stuff
#include "driver/gpio.h"         // gpio
#include "tinyusb.h"             // usb
#include "esp_timer.h"

/* -----------------------------------------------------------------------------------------------------------------------------

PIN LAYOUT

----------------------------------------------------------------------------------------------------------------------------- */

/*
Define digital inputs for buttons and pedals
*/

/*
Define analog inputs for drawbars, pots and pedals
*/
#define MULTIPLEXER_IN_ADC_ATTENUATION ADC_ATTEN_DB_12 // I tried no attenuation but ADC got saturaded with way less than 3.3V.
#define MULTIPLEXER_IN_ADC_RESOLUTION ADC_BITWIDTH_12  // 12 ESP32-S3 is 12 bits. If I try to set anything else it complains. TODO: Configure this with a macro.
#define MULTIPLEXER_IN_ADC_UNIT ADC_UNIT_1             // ADC unit to use
#define MULTIPLEXER_1_IN_ADC_CHANNEL ADC_CHANNEL_3     // GPIO4 / ADC1_CH3
#define MULTIPLEXER_2_IN_ADC_CHANNEL ADC_CHANNEL_4     // GPIO5 / ADC1_CH4

/*
Define digital outputs for indexing the multiplexers
*/
#define MULTIPLEXERS_IDX_0 9
#define MULTIPLEXERS_IDX_1 10
#define MULTIPLEXERS_IDX_2 11

/*
Define MIDI Rx & Tx
The WEMOS ESP32 board I have has 3 UARTs:
- UART0: Rx/Tx exposed via pins 1/3.
         Also exposed via the serial-usb bridge.
         This is what's used to program the chip, so we can't use it for MIDI at the same time.
         From code we access it via "Serial".
- UART1: Rx/Tx pins are shared with FLASH so we can't use that one either.
- UART2: Rx/Tx pins are 16/17.
         This one is up for grabs so I'll use it for MIDI.
         It's exposed via "Serial2" but I this depends on the board, so I will actually define
         my own MIDI Serial interface as `HardwareSerial SerialMIDI(2);` where (2) is the UART id.

The FREENOVE ESP32-s3 has 2 UARTs (I think! based on the pinout. Double check this).
- U0: used for the built-in usb bridge.
- U1: free. Rx/Tx are 17/18, also called U1TXD and U1RXD
*/

/* -----------------------------------------------------------------------------------------------------------------------------

MIDI CC maps

----------------------------------------------------------------------------------------------------------------------------- */

#define DRAWBAR_UPPER_IDX 0
#define DRAWBAR_LOWER_IDX 1
#define DRAWBAR_PEDALBOARD_IDX 2

uint8_t drawbar_CC_map[3][9] = {
    // MIDI CC # range is 0-127 so 7 bits
    {12, 13, 14, 15, 16, 17, 18, 19, 20}, // Upper
    {21, 22, 23, 24, 25, 26, 27, 28, 29}, // Lower
    {33, 35, 0, 0, 0, 0, 0, 0, 0}         // Pedalboard
};

#define MIDI_NOTE_ON 0x90
#define MIDI_NOTE_OFF 0x80
#define MIDI_MASK_STATUS_BYTE 0b10000000
#define MIDI_MASK_REAL_TIME_BYTE 0b11111000
#define MIDI_MASK_VOICE_COMMAND_BYTE 0b11110000
#define MIDI_MASK_VOICE_CHANNEL_BYTE 0b00001111

/* -----------------------------------------------------------------------------------------------------------------------------

Global variables

----------------------------------------------------------------------------------------------------------------------------- */

/*
ACD reads
*/
uint8_t analog_input_idx = 0;                                                     // used to index the analog components to be accessed via the 4051 multiplexers
adc_oneshot_unit_handle_t multiplexers_adc_handle;                                // handler to read from ADCs
uint16_t analog_values[17] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // ADC is 12 bits
/*
Leslie status
*/
#define LESLIE_STOP 0
#define LESLIE_SLOW 1
#define LESLIE_FAST 2
uint8_t leslie_speed = LESLIE_SLOW;
/*
Drawbar status
*/
uint8_t drawbar_selected = DRAWBAR_UPPER_IDX;
uint8_t drawbar_midi_values[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
/*
MIDI config
*/
uint8_t midi_channel = 0;   // 0-15 maps to 1-16
uint8_t midi_cable_num = 0; // TinyUSB concept: MIDI jack associated with USB endpoint (?)

/* -----------------------------------------------------------------------------------------------------------------------------

USB

----------------------------------------------------------------------------------------------------------------------------- */

// Interface counter
enum interface_count
{
#if CFG_TUD_MIDI
    ITF_NUM_MIDI = 0,       // MIDI uses a control interface to transfer configuration
    ITF_NUM_MIDI_STREAMING, // and a second one to actually send data
#endif
    ITF_COUNT
};

// USB Endpoint numbers
enum usb_endpoints
{
    // Available USB Endpoints: 5 IN/OUT EPs and 1 IN EP
    EP_EMPTY = 0, // Endpoint 0 is always reserved for control transfers in USB.
#if CFG_TUD_MIDI
    EPNUM_MIDI, // Defined only if CFG_TUD_MIDI is enabled (MIDI support is compiled in).
#endif
};

/** TinyUSB descriptors **/

#define TUSB_DESCRIPTOR_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_MIDI * TUD_MIDI_DESC_LEN) // add a second interface

/* -----------------------------------------------------------------------------------------------------------------------------

Helpers

----------------------------------------------------------------------------------------------------------------------------- */

/*
midi_task_read_and_discard reads incoming MIDI messages from the USB interface and discards them
*/
static void midi_task_read_and_discard(void *arg)
{
    // The MIDI interface always creates input and output port/jack descriptors
    // regardless of these being used or not. Therefore incoming traffic should be read
    // (possibly just discarded) to avoid the sender blocking in IO
    uint8_t packet[4];
    bool read = false;
    for (;;)
    {
        vTaskDelay(1);
        while (tud_midi_available())
        {
            read = tud_midi_packet_read(packet);
            if (read)
            {
                ESP_LOGI(TAG, "Read - Time (ms since boot): %lld, Data: %02hhX %02hhX %02hhX %02hhX",
                         esp_timer_get_time(), packet[0], packet[1], packet[2], packet[3]);
            }
        }
    }
}

/*
sendMidiCC sends a CC message
*/
void sendMidiCC(uint8_t channel, uint8_t number, uint8_t value)
{
    // Don't send if CC number is zero
    if (number == 0)
        return;

    // Send serial MIDI
    uint8_t statusByte = 0b10110000 | channel; // channel should be 0-15. I won't enforce it to make it more efficient.
    // SerialMIDI.write(statusByte);
    // SerialMIDI.write(number);
    // SerialMIDI.write(value);

    // Send BLE MIDI
    // if (BLEMidiServer.isConnected())
    // {
    //     BLEMidiServer.controlChange(midi_channel, number, value);
    // }
    ESP_LOGI(TAG, "Sending MIDI CC %d value %d", number, value);
    // Send USB MIDI
    if (tud_midi_mounted())
    {
        uint8_t data[3] = {statusByte, number, value};
        tud_midi_stream_write(midi_cable_num, data, 3);
    }
}

/*
Read analog inputs
*/
void readAnalogComponents(void)
{
    // TODO: [note] Only reading the first 9 (drawbars) for now. This is a test and I don't have the other pots connected.
    if (++analog_input_idx > 8)
        analog_input_idx = 0; // increment multiplexer index to read analog components

    // Select inputs from multiplexers
    gpio_set_level(MULTIPLEXERS_IDX_0, (analog_input_idx & 7) & 1);
    gpio_set_level(MULTIPLEXERS_IDX_1, (analog_input_idx & 7) >> 1 & 1);
    gpio_set_level(MULTIPLEXERS_IDX_2, (analog_input_idx & 7) >> 2 & 1);

    // Give the multiplexer a second to switch
    vTaskDelay(pdMS_TO_TICKS(10));

    // Select which multiplexer to use and then read from it
    int value;
    esp_err_t err = adc_oneshot_read(multiplexers_adc_handle, analog_input_idx > 7 ? MULTIPLEXER_1_IN_ADC_CHANNEL : MULTIPLEXER_2_IN_ADC_CHANNEL, &value);
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to read analog %d: %s", analog_input_idx, err);
        return;
    }
    analog_values[analog_input_idx] = value;

    if (analog_input_idx < 9)
    {
        // It's a drawbar
        uint8_t midi_value = value >> 5; // shift 12 bits from the ADC to 7 bits wanted by MIDI
        if (midi_value != drawbar_midi_values[analog_input_idx])
        {
            drawbar_midi_values[analog_input_idx] = midi_value;
            sendMidiCC(midi_channel, drawbar_CC_map[drawbar_selected][analog_input_idx], drawbar_midi_values[analog_input_idx]);
        }
    }
}

static void periodic_midi_write_example_cb(void *arg)
{
    // Example melody stored as an array of note values
    uint8_t const note_sequence[] = {
        74, 78, 81, 86, 90, 93, 98, 102, 57, 61, 66, 69, 73, 78, 81, 85, 88, 92, 97, 100, 97, 92, 88, 85, 81, 78,
        74, 69, 66, 62, 57, 62, 66, 69, 74, 78, 81, 86, 90, 93, 97, 102, 97, 93, 90, 85, 81, 78, 73, 68, 64, 61,
        56, 61, 64, 68, 74, 78, 81, 86, 90, 93, 98, 102};

    static uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
    static uint8_t const channel = 0;   // 0 for channel 1
    static uint32_t note_pos = 0;

    // Previous positions in the note sequence.
    int previous = note_pos - 1;

    // If we currently are at position 0, set the
    // previous position to the last note in the sequence.
    if (previous < 0)
    {
        previous = sizeof(note_sequence) - 1;
    }

    // Send Note On for current position at full velocity (127) on channel 1.
    ESP_LOGI(TAG, "Writing MIDI data %d", note_sequence[note_pos]);

    if (tud_midi_mounted())
    {
        uint8_t note_on[3] = {MIDI_NOTE_ON | channel, note_sequence[note_pos], 127};
        tud_midi_stream_write(cable_num, note_on, 3);

        // Send Note Off for previous note.
        uint8_t note_off[3] = {MIDI_NOTE_OFF | channel, note_sequence[previous], 0};
        tud_midi_stream_write(cable_num, note_off, 3);
    }

    // Increment position
    note_pos++;

    // If we are at the end of the sequence, start over.
    if (note_pos >= sizeof(note_sequence))
    {
        note_pos = 0;
    }
}

void sendMidiSequence(void)
{
    // Periodically send MIDI packets
    int const tempo = 286;
    const esp_timer_create_args_t periodic_midi_args = {
        .callback = &periodic_midi_write_example_cb,
        /* name is optional, but may help identify the timer when debugging */
        .name = "periodic_midi"};

    ESP_LOGI(TAG, "MIDI write task init");
    esp_timer_handle_t periodic_midi_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_midi_args, &periodic_midi_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_midi_timer, tempo * 1000));
}

/* -----------------------------------------------------------------------------------------------------------------------------

Setup

----------------------------------------------------------------------------------------------------------------------------- */

void setupPins()
{
    // Init ADCs
    adc_oneshot_unit_init_cfg_t multiplexers_adc_init_config = {
        .unit_id = MULTIPLEXER_IN_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&multiplexers_adc_init_config, &multiplexers_adc_handle));
    adc_oneshot_chan_cfg_t multiplexers_adc_config = {
        .atten = MULTIPLEXER_IN_ADC_ATTENUATION,
        .bitwidth = MULTIPLEXER_IN_ADC_RESOLUTION,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(multiplexers_adc_handle, MULTIPLEXER_1_IN_ADC_CHANNEL, &multiplexers_adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(multiplexers_adc_handle, MULTIPLEXER_2_IN_ADC_CHANNEL, &multiplexers_adc_config));

    // Init multiplexer indexing pins as outputs
    gpio_reset_pin(MULTIPLEXERS_IDX_0);
    gpio_reset_pin(MULTIPLEXERS_IDX_1);
    gpio_reset_pin(MULTIPLEXERS_IDX_2);
    gpio_set_direction(MULTIPLEXERS_IDX_0, GPIO_MODE_OUTPUT);
    gpio_set_direction(MULTIPLEXERS_IDX_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MULTIPLEXERS_IDX_2, GPIO_MODE_OUTPUT);
}

void setupUSB()
{

    static const char language_descriptor[] = {0x09, 0x04}; // English (0x0409)
    /**
     * @brief String descriptor
     */
    static const char *usb_string_descriptor[6] = {
        // array of pointer to string descriptors
        language_descriptor, // 0: is supported language is English (0x0409)
        "Cave Man",          // 1: Manufacturer
        "Jammond",           // 2: Product
        "4.8.15.16.32.42",   // 3: Serials, should use chip ID
        "Jammond MIDI",      // 4: MIDI
        "Jammond MIDI2",     // 5: MIDI
    };

    /**
     * @brief Configuration descriptor
     *
     * This is a simple configuration descriptor that defines 1 configuration and a MIDI interface
     */
    static const uint8_t midi_config_descriptors[] = {
        // Configuration number, interface count, string index, total length, attribute, power in mA
        TUD_CONFIG_DESCRIPTOR(
            1,                         // This is the first configuration
            ITF_COUNT,                 // how many interfaces are defined (1 for MIDI, 1 for MIDI stream (?)) total 2
            0,                         // tells what language is supported (index of usb_string_descriptor)
            TUSB_DESCRIPTOR_TOTAL_LEN, // automatically calculated with the length of this descriptor and the midi descriptor below
            0,                         // no attribute?
            100                        // draw 100mA
            ),

        // Interface number, string index, EP Out & EP In address, EP size
        TUD_MIDI_DESCRIPTOR(
            ITF_NUM_MIDI,        // Number of the control MIDI interface (0). The streaming interface is automatically created (+1)
            4,                   // Name of the MIDI device (index of usb_string_descriptor)
            EPNUM_MIDI,          // endpoint number of the MIDI interface (1)
            (0x80 | EPNUM_MIDI), //
            64                   // size of the endpoint (?)
            ),
    };

    tinyusb_config_t const usb_config = {
        .device_descriptor = NULL, // If device_descriptor is NULL, tinyusb_driver_install() will use Kconfig
        .string_descriptor = usb_string_descriptor,
        .string_descriptor_count = sizeof(usb_string_descriptor) / sizeof(usb_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = midi_config_descriptors,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&usb_config));

    ESP_LOGI(TAG, "USB initialization DONE");

    // Create task to read incoming MIDI data so that we don't saturate the usb buffer

    ESP_LOGI(TAG, "MIDI read task init");
    xTaskCreate(midi_task_read_and_discard, "midi_task_read_and_discard", 4 * 1024, NULL, 5, NULL);
}


void setupSerialMIDI() {

}


/* -----------------------------------------------------------------------------------------------------------------------------

Let's get cracking

----------------------------------------------------------------------------------------------------------------------------- */

void app_main(void)
{
    // Configure pins for in/out/adc
    setupPins();

    // Start USB interface
    setupUSB();

    // Start serial MIDI
    setupSerialMIDI();

    // Main loop
    while (1)
    {
        // TODO: explore making this a task. Maybe I can have 1 core do coms & non critical stuff
        // while the other core does real time audio processing.
        for (int i = 0; i < 9; i++)
        {
            readAnalogComponents();
        }
        ESP_LOGI(TAG, "drawbar reads: [%d] [%d] [%d] [%d] [%d] [%d] [%d] [%d] [%d]", analog_values[0], analog_values[1], analog_values[2], analog_values[3], analog_values[4], analog_values[5], analog_values[6], analog_values[7], analog_values[8]);

        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
