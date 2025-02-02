# Blus firmware

Rust firmware for different audio hardware.

Implements a USB UAC 1.0 compliant sound card with software volume control. Audio data is then output via I2S (or TDM),
depending on the specific target platform.

- For the [Black pill v1.2](./blackpill_pcm5102a/) with an attached I2S DAC
- For [Blus Mini Mk1](./blus_mini_mk1/)
- For [Blus Mini Mk2](./blus_mini_mk2/)

For Blus hardware, see https://github.com/blus-audio/hardware.
