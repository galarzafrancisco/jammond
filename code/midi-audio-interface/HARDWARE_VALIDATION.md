# MIDI + Audio Hardware Validation Report

This report tracks end-to-end validation of `code/midi-audio-interface/esp-idf` on real ESP32-S3 hardware.

## Firmware under test

- Branch: `feature/validate-midi-audio-hardware`
- App path: `code/midi-audio-interface/esp-idf`
- Target: ESP32-S3
- USB profile: Composite USB Audio Output + USB MIDI
- Audio format: 48 kHz, stereo, 16-bit PCM

## Lab setup

Fill this section before each run.

- Board and revision:
- External DAC model and wiring:
- MIDI source device:
- UART wiring details:
- Host OS and version:
- Test apps used (DAW, MIDI monitor, audio player):
- Firmware commit SHA:

## Validation matrix

Use `PASS`, `FAIL`, or `PENDING`.

| Test | macOS | Windows | Notes |
| --- | --- | --- | --- |
| Enumerates as composite Audio Output + MIDI | PENDING | PENDING | |
| UART MIDI -> USB MIDI forwarding under sustained traffic | PENDING | PENDING | |
| Pot/ADC CC generation stability (acceptable jitter only) | PENDING | PENDING | |
| USB audio playback continuity to external DAC at 48 kHz stereo | PENDING | PENDING | |

## Measurements

- End-to-end MIDI latency:
- End-to-end audio latency:
- Buffer underruns observed:
- Buffer overruns observed:
- Host-specific quirks:

## Defects and follow-ups

List regressions or defects discovered during validation.

1. None recorded yet.

## Execution note for this branch

This branch was prepared in a CI-like coding environment without direct access to physical ESP32-S3 hardware, external DAC, or macOS/Windows hosts. The checklist and report structure are complete, but hardware test rows remain `PENDING` until run in the lab.
