# Blus firmware

Rust firmware for the popular Black Pill, in combination with a DAC (e.g. PCM5102a).

Enables a UAC 1.0 sound card with
- Fixed 48 kHz sample rate (stereo)
- Fixed 32 bit audio sample width
- Software volume control

## [Black Pill 1.2](v1.2/)
Data is output via the following I2S pins:
- Serial clock (SCK) at 3.072 MHz: PB10
- Serial data (SD): PB15
- Word select (WS) at 48 kHz: PB12

## [Black Pill 3.1](v3.1/)
Data is output via the following I2S pins:
- Serial clock (SCK) at 3.072 MHz: PB3
- Serial data (SD): PB5
- Word select (WS) at 48 kHz: PA15
