/* -----------------------------------------------------------------------------------------------------------------------------

Libs

----------------------------------------------------------------------------------------------------------------------------- */


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
#define DEADBAND 8         // this is how much an analog value must change for us to consider it: protects us from noise
int analog_input_idx = 0;  // used to index the analog components to be accessed via the 4051 multiplexers
int drawbar_selected = DRAWBAR_UPPER_IDX;
#define LESLIE_STOP 0
#define LESLIE_SLOW 1
#define LESLIE_FAST 2
int leslie_speed = LESLIE_SLOW;
int analog_values[17] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
int midi_channel = 1;



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

void setup() {
  // Logger
  Serial.begin(115200);

  // Call setup functions
  setupPins();

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
  int current_value = analogRead(analog_input_idx > 7 ? MULTIPLEXER_2_IN : MULTIPLEXER_1_IN);

  // See how much this value changed. Ignore small changes (noise).
  int diff = abs(current_value - analog_values[analog_input_idx]);
  if (diff <= DEADBAND) return; // TODO: implement low-pass IIR filter instead

  analog_values[analog_input_idx] = current_value;

  // Now we have a 12 bit (4096) ADC and MIDI is 7 bit (128)
  int midi_value = current_value >> 5;

}

/* -----------------------------------------------------------------------------------------------------------------------------

Let's get cracking

----------------------------------------------------------------------------------------------------------------------------- */

void loop() {
  readAnalogComponents();
}