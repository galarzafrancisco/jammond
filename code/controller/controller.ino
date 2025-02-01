/* -----------------------------------------------------------------------------------------------------------------------------

Libs

----------------------------------------------------------------------------------------------------------------------------- */
// #include <MIDI.h> // < used to read/write MIDI messages via the serial peripheral interface
// #include <Arduino.h>
#include <BLEMidi.h>

/* -----------------------------------------------------------------------------------------------------------------------------

PIN LAYOUT

----------------------------------------------------------------------------------------------------------------------------- */

/*
Define digital inputs for buttons and pedals
*/

/*
Define analog inputs for drawbars, pots and pedals
*/
#define MULTIPLEXER_1_IN A14  // 13
#define MULTIPLEXER_2_IN A15  // 12

/*
Define digital outputs for indexing the multiplexers
*/
#define MULTIPLEXERS_IDX_0 21
#define MULTIPLEXERS_IDX_1 22
#define MULTIPLEXERS_IDX_2 23

/*
Define MIDI Rx & Tx
*/

/*
Define I2S lines
*/



/* -----------------------------------------------------------------------------------------------------------------------------

MIDI CC maps

----------------------------------------------------------------------------------------------------------------------------- */
#define DRAWBAR_UPPER_IDX 0
#define DRAWBAR_LOWER_IDX 1
#define DRAWBAR_PEDALBOARD_IDX 2

int drawbar_CC_map[3][9] = {
  { 12, 13, 14, 15, 16, 17, 18, 19, 20 },  // Upper
  { 21, 22, 23, 24, 25, 26, 27, 28, 29 },  // Lower
  { 33, 35, 0, 0, 0, 0, 0, 0, 0 }          // Pedalboard
};



/* -----------------------------------------------------------------------------------------------------------------------------

Global variables

----------------------------------------------------------------------------------------------------------------------------- */
#define DEADBAND 2            // ignore CC changes that move less than this value
#define ANALOG_LPF_ALPHA 0.5  // used to filter high frequency noise from analog reads
uint8_t analog_input_idx = 0;     // used to index the analog components to be accessed via the 4051 multiplexers
uint8_t drawbar_selected = DRAWBAR_UPPER_IDX;
#define LESLIE_STOP 0
#define LESLIE_SLOW 1
#define LESLIE_FAST 2
uint8_t leslie_speed = LESLIE_SLOW;
float analog_inputs_values_read[17] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t analog_inputs_midi_values_sent[17] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // 7 bits
uint8_t midi_channel = 1;



/* -----------------------------------------------------------------------------------------------------------------------------

Setup

----------------------------------------------------------------------------------------------------------------------------- */

/*
Setup MCU pin modes
*/
void setupPins() {
  // Multiplexer index
  pinMode(MULTIPLEXERS_IDX_0, OUTPUT);
  pinMode(MULTIPLEXERS_IDX_1, OUTPUT);
  pinMode(MULTIPLEXERS_IDX_2, OUTPUT);

  // Multiplexer analog input: no need to do anything, just calling analogRead on an analog pin works :)
}

/*
Load config from flash (like modified CC messages, MIDI channels, and anything I might think of)
*/

/*
Start serial MIDI
*/

/*
Start BLE MIDI
*/
void startBLEMIDI() {
  Serial.println("Initializing bluetooth");
  BLEMidiServer.begin("Jammon-d");
}

void setup() {
  // Logger
  Serial.begin(115200);

  // Call setup functions
  setupPins();

  // Start modules
  startBLEMIDI();
}


/* -----------------------------------------------------------------------------------------------------------------------------

Helpers

----------------------------------------------------------------------------------------------------------------------------- */

/*
MIDIThrough read 1 MIDI message from Serial Rx.
If it's a note on/off it writes it to both Serial Tx and BLE
otherwise it ignores it.
*/

/*
sendMidiCC sends a CC message
*/
void sendMidiCC(int channel, int number, int value) {
  // Don't send if CC number is zero
  if (number == 0) return;

  // Serial.print("Sending MIDI CC #");
  // Serial.print(number);
  // Serial.print(" value: ");
  // Serial.println(value);
  // // Send serial MIDI
  // MIDI.sendControlChange(number, value, channel); // Midi lib wants channels 1~16

  // Send BLE MIDI
  if (BLEMidiServer.isConnected()) {
    // Serial.println("BLE OK");
    BLEMidiServer.controlChange(midi_channel - 1, number, value);  // wants MIDI channels 0-15 and I do 1-16 in my conifg
  }

  // TODO: implement Send USB
}

/*
Read analog inputs
*/
void readAnalogComponents() {
  // TODO: [note] Only reading the first 9 (drawbars) for now. This is a test and I don't have the other pots connected.
  if (++analog_input_idx > 8) analog_input_idx = 0;  // increment multiplexer index to read analog components

  // Select inputs from multiplexers
  digitalWrite(MULTIPLEXERS_IDX_0, (analog_input_idx & 7) & 1);
  digitalWrite(MULTIPLEXERS_IDX_1, (analog_input_idx & 7) >> 1 & 1);
  digitalWrite(MULTIPLEXERS_IDX_2, (analog_input_idx & 7) >> 2 & 1);

  // Select which multiplexer to use and then read from it
  float current_value = analogRead(analog_input_idx > 7 ? MULTIPLEXER_2_IN : MULTIPLEXER_1_IN); // 12 bits

  // Run the read through an IIR LPF to remove noise
  analog_inputs_values_read[analog_input_idx] += ANALOG_LPF_ALPHA * (current_value - analog_inputs_values_read[analog_input_idx]);

  uint8_t midi_value = (uint16_t)analog_inputs_values_read[analog_input_idx] >> 5; // Shift the ADC's 12 bits to MIDI's 7 bits

  // Debug for drawbar 9 (pin is noisy)
  // if (analog_input_idx == 8) {
  //   Serial.print("ADC: ");
  //   Serial.print(current_value);
  //   Serial.print(" | Current after LPF: ");
  //   Serial.print(analog_inputs_values_read[analog_input_idx]);
  //   Serial.print(" | MIDI value: ");
  //   Serial.print(midi_value);
  //   Serial.print(" | Previously sent: ");
  //   Serial.println(analog_inputs_midi_values_sent[analog_input_idx]);
  // }
  // If the value changed hasn't changed, move on
  if (abs(midi_value - analog_inputs_midi_values_sent[analog_input_idx]) < DEADBAND) return;

  // Update the value
  analog_inputs_midi_values_sent[analog_input_idx] = midi_value;

    // Send MIDI CC
    if (analog_input_idx < 9) {  // drawbar
    sendMidiCC(midi_channel, drawbar_CC_map[drawbar_selected][analog_input_idx], midi_value);
  }
}

/* -----------------------------------------------------------------------------------------------------------------------------

Let's get cracking

----------------------------------------------------------------------------------------------------------------------------- */

void loop() {
  readAnalogComponents();
}