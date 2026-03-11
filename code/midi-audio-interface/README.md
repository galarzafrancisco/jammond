# MIDI + Audio Interface (ESP32-S3)

This folder contains a new codebase intended to replace the split controller/audio experiments with one composite USB device.

Target behavior:
- USB MIDI interface (host sees MIDI in/out)
- USB Audio Output interface (host sends stereo PCM to the board)
- UART MIDI input merged into USB MIDI output
- Analog pots read through ADC and emitted as MIDI CC
- USB audio forwarded to an external DAC over I2S

## Current status

The composite USB implementation is in place in `esp-idf/main/jammond-midi-audio.c` and includes:
- USB Audio Output + USB MIDI descriptors using raw TinyUSB
- UART MIDI forwarding to USB MIDI
- Analog pot scanning and MIDI CC generation
- USB audio ingestion and forwarding to an external DAC over I2S

Hardware validation is tracked in `HARDWARE_VALIDATION.md`.

## Why a new project

`usb_device_uac` is convenient for Audio but hides descriptor-level control, which prevents exposing an additional MIDI function in the same composite device. This project keeps direct TinyUSB descriptor ownership.

## Next execution order

1. Run full hardware validation on macOS and Windows (enumeration, MIDI forwarding, audio continuity).
2. Add runtime audio sample-rate switching in the I2S path.
3. Define and implement USB MIDI OUT handling policy.
4. Write bring-up and validation guide for repeatable lab testing.
