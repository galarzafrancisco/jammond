# MIDI + Audio Interface (ESP32-S3)

This folder contains a new codebase intended to replace the split controller/audio experiments with one composite USB device.

Target behavior:
- USB MIDI interface (host sees MIDI in/out)
- USB Audio Output interface (host sends stereo PCM to the board)
- UART MIDI input merged into USB MIDI output
- Analog pots read through ADC and emitted as MIDI CC
- USB audio forwarded to an external DAC over I2S

## Current status

This initial implementation sets up the new project structure and the end-to-end runtime pipeline for:
- UART MIDI parsing and forwarding to USB MIDI
- Analog pot scanning and MIDI CC generation
- DAC output task and ring buffer plumbing

The remaining piece is wiring raw TinyUSB Audio class descriptors/callbacks in `main/jammond-midi-audio.c` so USB speaker packets are consumed directly from TinyUSB and pushed to the DAC ring buffer.

## Why a new project

`usb_device_uac` is convenient for Audio but hides descriptor-level control, which prevents exposing an additional MIDI function in the same composite device. This project keeps direct TinyUSB descriptor ownership.

## Next execution order

1. Add TinyUSB composite descriptor with Audio + MIDI interfaces.
2. Implement TinyUSB Audio OUT receive callbacks and feed the ring buffer.
3. Validate sample rate and endpoint sizing at 48k stereo, then expand to 96k if needed.
4. Validate full device enumeration on macOS/Windows and MIDI merge behavior under load.
