const static char *TAG = "Jammond controller";

/* -----------------------------------------------------------------------------------------------------------------------------

Libs

----------------------------------------------------------------------------------------------------------------------------- */
#include <stdio.h>
#include "esp_adc/adc_oneshot.h" // adc
#include "esp_log.h"             // log
#include "freertos/FreeRTOS.h"   // delays and stuff
#include "freertos/task.h"       // delays and stuff

/* -----------------------------------------------------------------------------------------------------------------------------

PIN LAYOUT

----------------------------------------------------------------------------------------------------------------------------- */

/*
Define digital inputs for buttons and pedals
*/

/*
Define analog inputs for drawbars, pots and pedals
*/
#define MULTIPLEXER_IN_ADC_ATTENUATION ADC_ATTEN_DB_0 // don't attenuate, read the full input voltage
#define MULTIPLEXER_IN_ADC_RESOLUTION ADC_BITWIDTH_12 // 12 ESP32-S3 is 12 bits. If I try to set anything else it complains. TODO: Configure this with a macro.
#define MULTIPLEXER_IN_ADC_UNIT ADC_UNIT_1            // ADC unit to use
#define MULTIPLEXER_1_IN_ADC_CHANNEL ADC_CHANNEL_3    // GPIO4 / ADC1_CH3
#define MULTIPLEXER_2_IN_ADC_CHANNEL ADC_CHANNEL_4    // GPIO5 / ADC1_CH4

/*
Define digital outputs for indexing the multiplexers
*/
#define MULTIPLEXERS_IDX_0 GPIO9
#define MULTIPLEXERS_IDX_1 GPIO10
#define MULTIPLEXERS_IDX_2 GPIO11

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

/* -----------------------------------------------------------------------------------------------------------------------------

Global variables

----------------------------------------------------------------------------------------------------------------------------- */

// Handler to read from ADCs
adc_oneshot_unit_handle_t multiplexers_adc_handle;

/* -----------------------------------------------------------------------------------------------------------------------------

Setup

----------------------------------------------------------------------------------------------------------------------------- */

void setupPins()
{
    // Init ADCs
    adc_oneshot_unit_init_cfg_t multiplexers_adc_init_config = {
        .unit_id = MULTIPLEXER_IN_ADC_UNIT};
    adc_oneshot_chan_cfg_t multiplexers_adc_config = {
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = MULTIPLEXER_IN_ADC_RESOLUTION,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&multiplexers_adc_init_config, &multiplexers_adc_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(multiplexers_adc_handle, MULTIPLEXER_1_IN_ADC_CHANNEL, &multiplexers_adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(multiplexers_adc_handle, MULTIPLEXER_2_IN_ADC_CHANNEL, &multiplexers_adc_config));
}

/* -----------------------------------------------------------------------------------------------------------------------------

Helpers

----------------------------------------------------------------------------------------------------------------------------- */

/* -----------------------------------------------------------------------------------------------------------------------------

Let's get cracking

----------------------------------------------------------------------------------------------------------------------------- */

void app_main(void)
{
    setupPins();

    while (1)
    {
        // Read from adc
        static int adc_raw[2][10];
        ESP_ERROR_CHECK(adc_oneshot_read(multiplexers_adc_handle, MULTIPLEXER_1_IN_ADC_CHANNEL, &adc_raw[0][0]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", MULTIPLEXER_IN_ADC_UNIT + 1, MULTIPLEXER_1_IN_ADC_CHANNEL, adc_raw[0][0]);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
