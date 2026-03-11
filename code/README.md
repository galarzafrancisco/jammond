# Jammond

## Controller
Contains code to read drawbars, buttons, pots, take MIDI input from the keyboard module, and output a combined MIDI signal.

## Tonewheel
Contains code related to generating the Hammond sound. This is just for fun, a POC at best. Sound will actually come from a good quality software like VB3.

## Leslie
Same as above.

## MIDI + Audio interface
Contains a new ESP-IDF codebase for a single ESP32-S3 firmware exposing both USB MIDI and USB Audio output paths.

### Follow-up work
- Run hardware validation on macOS and Windows and capture latency/underrun behavior.
- Implement runtime sample-rate switching in the I2S path when host sends USB Audio rate changes.
- Define and implement handling for USB MIDI OUT messages coming from the host.
- Expand bring-up docs with wiring details and reproducible validation steps.
