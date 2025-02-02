/* -----------------------------------------------------------------------------------------------------------------------------

Libs

----------------------------------------------------------------------------------------------------------------------------- */
// #include <MIDI.h>  // < used to read/write MIDI messages via the serial peripheral interface
// #include <Arduino.h> // < ???? needed for Serial MIDI???
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
#define MULTIPLEXER_2_IN A15  // 12 < if this is HIGH during startup, it will crash 🤷🏻‍♂️ and throw errors while flashing 🤷🏻‍♂️🤷🏻‍♂️

/*
Define digital outputs for indexing the multiplexers
*/
#define MULTIPLEXERS_IDX_0 21
#define MULTIPLEXERS_IDX_1 22
#define MULTIPLEXERS_IDX_2 23

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
*/
#define MIDI_UART 2
#define MIDI_RX_PIN 16                 // not really needed, here more as a reference for me
#define MIDI_BAUD_RATE 31250           // not really needed, here more as a reference for me
HardwareSerial SerialMIDI(MIDI_UART);  // Serial 1 shares pins with flash, so that's a no go

/*
Define I2S lines
*/

/* -----------------------------------------------------------------------------------------------------------------------------

MIDI CC maps

----------------------------------------------------------------------------------------------------------------------------- */
#define DRAWBAR_UPPER_IDX 0
#define DRAWBAR_LOWER_IDX 1
#define DRAWBAR_PEDALBOARD_IDX 2

uint8_t drawbar_CC_map[3][9] = {
  // MIDI CC # range is 0-127 so 7 bits
  { 12, 13, 14, 15, 16, 17, 18, 19, 20 },  // Upper
  { 21, 22, 23, 24, 25, 26, 27, 28, 29 },  // Lower
  { 33, 35, 0, 0, 0, 0, 0, 0, 0 }          // Pedalboard
};

/* -----------------------------------------------------------------------------------------------------------------------------

Global variables

----------------------------------------------------------------------------------------------------------------------------- */
#define DEADBAND 32            // this is how much an analog value must change for us to consider it: protects us from noise (not really, I need a low pass filter). ACD is 10bits so 32/4096*256=2 MIDI
uint8_t analog_input_idx = 0;  // used to index the analog components to be accessed via the 4051 multiplexers
uint8_t drawbar_selected = DRAWBAR_UPPER_IDX;
#define LESLIE_STOP 0
#define LESLIE_SLOW 1
#define LESLIE_FAST 2
uint8_t leslie_speed = LESLIE_SLOW;
uint16_t analog_values[17] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // ADC is 12 bits
uint8_t midi_channel = 0;                                                            // 0-15 maps to 1-16

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
  Serial.println("Initializing bluetooth MIDI");
  BLEMidiServer.begin("Jammon-d");
}

/*
Start serial MIDI
*/
void startSerialMIDI() {
  // Initialize SerialMIDI for MIDI input.
  // We don't need to define the pin, the board configuration
  // already knows that the pins for UART 2 Rx/Tx are 16/17.
  SerialMIDI.begin(MIDI_BAUD_RATE);

  Serial.print("Serial MIDI started on UART ");
  Serial.println(MIDI_UART);
}

void setup() {
  // Logger
  Serial.begin(115200);

  // Call setup functions
  setupPins();

  // Start modules
  startBLEMIDI();
  startSerialMIDI();
}

/* -----------------------------------------------------------------------------------------------------------------------------

Helpers

----------------------------------------------------------------------------------------------------------------------------- */

/*
MIDIThrough read 1 MIDI message from Serial Rx.
If it's a note on/off it writes it to both Serial Tx and BLE
otherwise it ignores it.
*/
// State machine
#define SERIAL_MIDI_STATE_WAITING_FOR_STATUS_BYTE 0
#define SERIAL_MIDI_STATE_RECEIVED_STATUS_BYTE 1
uint8_t serialMidiState = SERIAL_MIDI_STATE_WAITING_FOR_STATUS_BYTE;

// Spec of MIDI messages: https://midi.org/summary-of-midi-1-0-messages
uint8_t serialMidiStatus = 0;  // holds the MIDI Status byte received and being processed
#define SERIAL_MIDI_DATA_BYTES_LENGTH 2
uint8_t serialMidiRawData[SERIAL_MIDI_DATA_BYTES_LENGTH];  // holds the data bytes received
uint8_t serialMidiPendingDataBytes = 0;                    // number of data bytes pending

#define MIDI_MASK_STATUS_BYTE 0b10000000
#define MIDI_MASK_REAL_TIME_BYTE 0b11111000
#define MIDI_MASK_VOICE_COMMAND_BYTE 0b11110000
#define MIDI_MASK_VOICE_CHANNEL_BYTE 0b00001111

void midiThrough() {
  if (!SerialMIDI.available()) {
    // No messages, move on.
    return;
  }

  uint8_t byteReceived = SerialMIDI.read();

  // Check if the byte is status (begining of a message, MSB=1) or data (continuation of a message, MSB=00)
  if (byteReceived & MIDI_MASK_STATUS_BYTE) {
    // Status message

    // It could be a real time message. If so, ignore it as it could be placed between other messages.
    // Real time messages have the 5 MSB set to 1
    if ((byteReceived & MIDI_MASK_REAL_TIME_BYTE) == MIDI_MASK_REAL_TIME_BYTE) {
      return;
    }

    // Extract command to calculate how many data bytes we need
    uint8_t command = byteReceived & MIDI_MASK_VOICE_COMMAND_BYTE;

    // Note on / off have 2 data bytes
    if (command == 0x90 || command == 0x80) {
      serialMidiPendingDataBytes = 2;
      serialMidiStatus = byteReceived;  // store the received status byte
      serialMidiState = SERIAL_MIDI_STATE_RECEIVED_STATUS_BYTE;
    } else {
      // We don't care about any other message, so let's ignore any data byte we receive
      serialMidiState = SERIAL_MIDI_STATE_WAITING_FOR_STATUS_BYTE;
    }

    return;

  } else {
    // Data message

    // Ignore any data byte if we are waiting for a status byte
    if (serialMidiState == SERIAL_MIDI_STATE_WAITING_FOR_STATUS_BYTE) {
      return;
    }

    // Store the data byte
    serialMidiRawData[SERIAL_MIDI_DATA_BYTES_LENGTH - serialMidiPendingDataBytes] = byteReceived;
    serialMidiPendingDataBytes--;

    if (serialMidiPendingDataBytes < 1) {
      // We're done receiving data
      serialMidiState = SERIAL_MIDI_STATE_WAITING_FOR_STATUS_BYTE;
      // Process the message
      uint8_t command = serialMidiStatus & MIDI_MASK_VOICE_COMMAND_BYTE;
      switch (command) {
        case 0x90:  // Note on
          sendMidiNoteOn(serialMidiStatus & MIDI_MASK_VOICE_CHANNEL_BYTE, serialMidiRawData[0], serialMidiRawData[1]);
          break;
        case 0x80:  // Note off
          sendMidiNoteOff(serialMidiStatus & MIDI_MASK_VOICE_CHANNEL_BYTE, serialMidiRawData[0], serialMidiRawData[1]);
          break;
      }
    }
  }

  return;
}

/*
sendMidiCC sends a CC message
*/
void sendMidiCC(uint8_t channel, uint8_t number, uint8_t value) {
  // Don't send if CC number is zero
  if (number == 0)
    return;

  // TODO: Send serial MIDI

  // Send BLE MIDI
  if (BLEMidiServer.isConnected()) {
    BLEMidiServer.controlChange(midi_channel, number, value);
  }

  // TODO: implement Send USB
}

// Sends a Note On message via BLE MIDI.
// channel: MIDI channel (0–15)
// note: MIDI note number (0–127)
// velocity: MIDI velocity (0–127)
void sendMidiNoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {

  // Send BLE MIDI
  if (BLEMidiServer.isConnected()) {
    BLEMidiServer.noteOn(channel, note, velocity);
  }
}

// Sends a Note Off message via BLE MIDI.
// channel: MIDI channel (1–16)
// note: MIDI note number (0–127)
// velocity: MIDI velocity (0–127)
void sendMidiNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {
  if (BLEMidiServer.isConnected()) {
    // BLE MIDI expects channels 0-15.
    BLEMidiServer.noteOff(channel, note, velocity);
  }
}

/*
Read analog inputs
*/
void readAnalogComponents() {
  // TODO: [note] Only reading the first 9 (drawbars) for now. This is a test and I don't have the other pots connected.
  if (++analog_input_idx > 8)
    analog_input_idx = 0;  // increment multiplexer index to read analog components

  // Select inputs from multiplexers
  digitalWrite(MULTIPLEXERS_IDX_0, (analog_input_idx & 7) & 1);
  digitalWrite(MULTIPLEXERS_IDX_1, (analog_input_idx & 7) >> 1 & 1);
  digitalWrite(MULTIPLEXERS_IDX_2, (analog_input_idx & 7) >> 2 & 1);

  // Select which multiplexer to use and then read from it
  uint16_t current_value = analogRead(analog_input_idx > 7 ? MULTIPLEXER_2_IN : MULTIPLEXER_1_IN);

  // See how much this value changed. Ignore small changes (noise).
  uint16_t diff = abs(current_value - analog_values[analog_input_idx]);
  if (diff <= DEADBAND)
    return;  // TODO: implement low-pass IIR filter instead

  analog_values[analog_input_idx] = current_value;

  // Now we have a 12 bit (4096) ADC and MIDI is 7 bit (128)
  uint8_t midi_value = current_value >> 5;

  // Send MIDI CC
  if (analog_input_idx < 9) {  // drawbar
    Serial.print("Sending CC #");
    Serial.print(drawbar_CC_map[drawbar_selected][analog_input_idx]);
    Serial.print(" [drawbar #");
    Serial.print(analog_input_idx + 1);
    Serial.print("]: ");
    Serial.println(midi_value);

    sendMidiCC(midi_channel, drawbar_CC_map[drawbar_selected][analog_input_idx], midi_value);
  }
}

/* -----------------------------------------------------------------------------------------------------------------------------

Let's get cracking

----------------------------------------------------------------------------------------------------------------------------- */

void loop() {
  midiThrough();
  readAnalogComponents();
}