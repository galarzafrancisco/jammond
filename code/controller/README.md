# Controller

I need to create 1 MIDI interface to emit CC messages for buttons, drawbars, pots, etc and note messages for the keyboard.

- Keyboard: I'll just include the whole Studiologic board which has a MIDI output (2 in fact!). I'll tap into this MIDI output and read it via a serial interface. Easy done.

- Pots, sliders, etc: I have the D9X drawbar controller with everything I need. That guy has an Arduino that reads the pyisical controllers and creates MIDI messages, both USB and 5 pin DIN. But I don't like the layout of the buttons and pots. I might use the controller board but mount the buttons and pots somewhere else. OOOOR I might just not use any of that and read the components directly from my controller module.


## Options
- I could modify the code in the D9X's Arduino to pretty much read the MIDI messages from the keyboard, filter out unwanted CC messages and merge it into the MIDI output it is already generating. If I do this I need to keep the drawbar controller board which is kind of big. Plus that Arduino is a bit shitty and a pain to program because the USB port is configured as OTG so I fuck it, I won't do this.

- I will actually use an ESP32 to read the keyboard MIDI and the pots, sliders, etc. The ESP32 will then create a MIDI output via one of the serial interfaces. It will also serve bluetooth MIDI because why not? BLE MIDI is very low latency and super usable, I could connect it to my laptop with no cables. That's sweet. I might even have the ESP serve a little website for advance settings like changig what CC messages are sent, MIDI channels, etc.


# Hardware
| name | type | q |
|------|------|---|
| Drawbars | analog | 9 |
| Volume | analog | 1 |
| Drive | analog | 1 |
| Reverb | analog | 1 |
| Key click | analog | 1 |
| Bass | analog | 1 |
| Treble | analog | 1 |
| Vibrato / chorus selector | analog | 1 |
| Swell pedal | analog | 1 |
| Leslie speed | digital | 1 |
| Leslie stop | digital | 1 |
| Vibrato on/off | digital | 1 |
| Perc on/off | digital | 1 |
| Perc soft/normal | digital | 1 |
| Perc slow/fast | digital | 1 |
| Perc 2nd/3rd | digital | 1 |
| Sustain pedal | digital | 1 |
| Switch pedal | digital | 1 |

I need 17 analog inputs and 9 digital inputs.
The D9X board uses 2 CD4051 8x1 multiplexers, which gives us 16 inputs. But I need 17. Where's the difference? In the vibrato/chorus selector. I'm planning to use a pot to select it but they use a button to cycle between the types. We'll see what I end up doing. Point is 2 CD4051 is not enough. I don't want to add a third one just for 1 extra analog input, so I'll use one from the ESP32.

So redoing the table again but for the MCU, I need:
| name | type | q |
|------|------|---|
| CD4051 1 | analog | 1 |
| CD4051 2 | analog | 1 |
| Extra pot | analog | 1 |

I need 3 analog inputs available in the MCU.

Also need 9 digital inputs.

Also need 3 digital outputs (index 0-7 for the multiplexers).

Also need serial input for the keyboard controller.

And serial output for MIDI.

I'd love to have another serial Tx for communicating with other ICs in the future. And an I2S interface for making some sounds.

### MCU pins needed
| Type | q |
|------|---|
| Analog in | 3 |
| Digital in | 9 |
| Digital out | 3 |
| Serial Rx | 1 |
| Serial Tx | 1 |
| I2S | 4 |

Total pins needed : 21

The ESP32 has a shit ton of pins (36 GPIO) that can be assigned to pretty much anything. We are gucci.

