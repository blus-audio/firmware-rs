# Blus firmware

Rust firmware for the popular Black Pill V1.2 with STM32F401CC, in combination with a DAC (e.g. PCM5102a).

Enables a UAC 1.0 sound card with
- Fixed 48 kHz sample rate (stereo)
- Fixed 32 bit audio sample width
- Software volume control

Data is output via the following I2S pins:
- Serial clock (SCK) at 3.072 MHz: PB10
- Serial data (SD): PB15
- Word select (WS) at 48 kHz: PB12
