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
#include "driver/uart.h"         // serial
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
#define MIDI_UART UART_NUM_2
#define MIDI_UART_RX 8

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
/*
Serial MIDI
*/
#define SERIAL_MIDI_BAUD_RATE 31250
// #define SERIAL_MIDI_BAUD_RATE 1200
const int serial_midi_buffer_size = (1024 * 2);
QueueHandle_t serial_midi_queue;
#define MIDI_RX_BUF_SIZE 1
uint8_t serial_midi_buffer[MIDI_RX_BUF_SIZE]; // Allocated on stack

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
sendMidiNoteOn
*/
void sendMidiNoteOn(uint8_t channel, uint8_t note, uint8_t velocity)
{
    uint8_t statusByte = MIDI_NOTE_ON | channel;
    // USB
    if (tud_midi_mounted())
    {
        uint8_t data[3] = {statusByte, note, velocity};
        tud_midi_stream_write(midi_cable_num, data, 3);
    }
}

/*
sendMidiNoteOff
*/
void sendMidiNoteOff(uint8_t channel, uint8_t note, uint8_t velocity)
{
    uint8_t statusByte = MIDI_NOTE_OFF | channel;
    // USB
    if (tud_midi_mounted())
    {
        uint8_t data[3] = {statusByte, note, velocity};
        tud_midi_stream_write(midi_cable_num, data, 3);
    }
}

/*
midiThrough read 1 MIDI message from Serial Rx.
If it's a note on/off it writes it to USB,
otherwise it ignores it.
*/

/*
 * Stub functions for handling note on/off events.
 * Replace these with your own implementations.
 */
void handle_note_on(uint8_t note, uint8_t velocity)
{
    // Example: trigger an oscillator or envelope for note ON.
    ESP_LOGI(TAG, "Note ON: note=%d, velocity=%d\n", note, velocity);
    return;
}

void handle_note_off(uint8_t note, uint8_t velocity)
{
    // Example: stop the note.
    ESP_LOGI(TAG, "Note OFF: note=%d, velocity=%d\n", note, velocity);
    return;
}

/*
 * MIDI parser state variables.
 */
static uint8_t running_status = 0;  // last status byte (only valid for channel messages)
static uint8_t data_bytes[2] = {0}; // buffer for data bytes (for messages needing 2 bytes)
static int data_count = 0;          // how many data bytes received for current message
static int expected_data_bytes = 0; // how many data bytes expected for current message
static bool in_sysex = false;       // true if we are inside a sysex message

/*
 * Returns the number of data bytes expected for a given status.
 * Only valid for channel messages (0x80 to 0xEF).
 */
static inline int get_expected_data_bytes(uint8_t status)
{
    switch (status & 0xF0)
    {
    case 0x80: // Note Off: status 0x80 - 0x8F
    case 0x90: // Note On: status 0x90 - 0x9F
    case 0xA0: // Polyphonic Key Pressure
    case 0xB0: // Control Change
    case 0xE0: // Pitch Bend
        return 2;
    case 0xC0: // Program Change
    case 0xD0: // Channel Pressure
        return 1;
    default:
        return 0; // Should not happen for channel messages
    }
}

/*
 * Process a complete MIDI message (i.e. status + data bytes).
 * Here we only trigger actions if the message is Note On/Off on channel 1.
 */
static void process_midi_message(uint8_t status, uint8_t *data, int length)
{
    // Only process channel messages (status < 0xF0)
    // Also, only channel 1: MIDI encodes channel 1 as low nibble == 0.
    if (status >= 0xF0 || ((status & 0x0F) != 0))
    {
        // Not a channel message or not on channel 1; ignore it.
        return;
    }

    uint8_t command = status & 0xF0;
    if (command == 0x90)
    {
        // Note On message: data[0] = note, data[1] = velocity.
        // Velocity 0 is treated as Note Off.
        uint8_t note = data[0];
        uint8_t velocity = data[1];
        if (velocity == 0)
        {
            handle_note_off(note, velocity);
        }
        else
        {
            handle_note_on(note, velocity);
        }
    }
    else if (command == 0x80)
    {
        // Note Off message.
        uint8_t note = data[0];
        uint8_t velocity = data[1];
        handle_note_off(note, velocity);
    }
    // Other channel messages can be handled (or simply ignored) as needed.
    return;
}

/*
 * midi_parse_byte() processes one incoming MIDI byte.
 * It implements running status, ignores real-time messages,
 * and handles sysex messages.
 */
void midi_parse_byte(uint8_t byte)
{
    if (byte != 0xF8 && byte != 0xFE && byte != 0xFC)
    {
        ESP_LOGI(TAG, "uart byte: %02X", byte);
    }
    // --- Handle real–time messages ---
    // MIDI real–time messages are in the range 0xF8-0xFF.
    // (Except F7 which is used to end sysex.)
    if (byte >= 0xF8)
    {
        // ignore all real–time messages.
        return;
    }

    // --- Handle sysex messages ---
    if (in_sysex)
    {
        if (byte >= 0b11111000)
        {
            // Real time message can be injected during a sysex stream. Ignore.
            return;
        }
        if (byte == 0b11110111)
        {
            // End of sysex
            in_sysex = false;
            return;
        }
        if (byte < 0b10000000)
        {
            // Data byte inside of sysex. Ignore.
            return;
        }
        else
        {
            // Status byte inside of sysex. This shouldn't happen.
            // We probably missed the end of sysex. Break out of it and process it.
            in_sysex = false;
        }
    }

    // --- Process status vs. data bytes ---
    if (byte & 0b10000000)
    {
        // This is a status byte.
        if (byte == 0b11110000)
        {
            // Start of a sysex message.
            in_sysex = true;
            // Clear any running status.
            running_status = 0;
            data_count = 0;
            expected_data_bytes = 0;
            return;
        }

        // For system common messages (0b11110001 - 11110110) we simply ignore.
        // (You might want to handle them in a more complete implementation.)
        if (byte >= 0b11110001)
        {
            // Clear running status for these messages.
            running_status = 0;
            data_count = 0;
            expected_data_bytes = 0;
            return;
        }

        // Otherwise, it’s a channel message.
        // Update running status.
        running_status = byte;
        data_count = 0;
        expected_data_bytes = get_expected_data_bytes(byte);
        ESP_LOGI(TAG, "status byte: %02X - expected data bytes: %d", running_status, expected_data_bytes);
    }
    else
    {
        // This is a data byte.
        if (running_status == 0)
        {
            ESP_LOGI(TAG, "Received data byte but running status is 0!!!!: %02X", byte);
            // No active running status, so ignore stray data.
            return;
        }
        // Append the data byte.
        if (data_count < (int)sizeof(data_bytes))
        {
            data_bytes[data_count] = byte;
            data_count++;
        }
        else
        {
            // Safety: if too many data bytes, reset.
            ESP_LOGI(TAG, "Received more data bytes than the size of the buffer. This shouldn't happen!!");
            data_count = 0;
            return;
        }
        // When we have collected enough data bytes for the message, process it.
        if (data_count >= expected_data_bytes)
        {
            data_count = 0;
            ESP_LOGI(TAG, "Got a full MIDI message");
            process_midi_message(running_status, data_bytes, data_count);
            // Reset data_count but keep the running status so that
            // subsequent messages (running status) can be processed.
        }
        return;
    }
    return;
}

void midi_in_task()
{
    while (1)
    {
        // Read serial
        // Read from UART
        // Read serial_midi_buffer from UART.

        // ESP_ERROR_CHECK(uart_get_buffered_serial_midi_buffer_len(MIDI_UART, (size_t*)&length));
        int rxBytes = uart_read_bytes(MIDI_UART, serial_midi_buffer, 1, 0);
        if (rxBytes < 1)
        {
            // vTaskDelay(pdMS_TO_TICKS(5)); // yield
            taskYIELD();
            continue;
        }

        for (int buf_idx = 0; buf_idx < rxBytes; buf_idx++)
        {
            uint8_t value = serial_midi_buffer[buf_idx];
            if (value != 0xFC && value != 0xFE)
            {

                ESP_LOGI(TAG, "%2d/%2d serial hex: %02X - dec: %d - bin %d%d%d%d%d%d%d%d", buf_idx + 1, rxBytes, value, value,
                         (value & 0b10000000) ? 1 : 0,
                         (value & 0b01000000) ? 1 : 0,
                         (value & 0b00100000) ? 1 : 0,
                         (value & 0b00010000) ? 1 : 0,
                         (value & 0b00001000) ? 1 : 0,
                         (value & 0b00000100) ? 1 : 0,
                         (value & 0b00000010) ? 1 : 0,
                         (value & 0b00000001) ? 1 : 0);
            }
            midi_parse_byte(value);
        }

        taskYIELD();
        // vTaskDelay(pdMS_TO_TICKS(5)); // yield
    }
}

/*
Read analog inputs
*/
void read_analog_task()
{
    while (1)
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

        vTaskDelay(pdMS_TO_TICKS(1)); // yield
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

void setupSerialMIDI()
{
    // Allocate memory for the Rx buffer
    // uint8_t *serial_midi_buffer = (uint8_t *)malloc(MIDI_RX_BUF_SIZE + 1);

    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = SERIAL_MIDI_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        // .data_bits = UART_DATA_BITS_MAX,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    // ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 4, 5, 18, 19));
    // Tx, Rx, Rts, Cts (only Rx)
    ESP_ERROR_CHECK(uart_set_pin(MIDI_UART, UART_PIN_NO_CHANGE, MIDI_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_line_inverse(MIDI_UART, UART_SIGNAL_IRDA_RX_INV));
    // ESP_ERROR_CHECK(uart_set_line_inverse(MIDI_UART, UART_SIGNAL));

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(MIDI_UART, serial_midi_buffer_size,
                                        serial_midi_buffer_size, 10, &serial_midi_queue, 0));
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

    /*
    Priorities so far:
    - audio gen 6
    - audio out: 5
    - midi in: 4
    - midi out: 3
    - analog read: 2
    */
    xTaskCreate(midi_in_task, "midi_in_task", 4096, NULL, 4, NULL);
    xTaskCreate(read_analog_task, "read_analog_task", 4096, NULL, 3, NULL); // 2048 overflows
}
