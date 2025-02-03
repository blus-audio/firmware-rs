#![no_std]

pub mod audio_filter;

use micromath::F32Ext;

#[derive(Clone, Copy, PartialEq, Debug, defmt::Format)]
pub enum AudioSource {
    None,
    Usb,
    Spdif,
    Ext,
    Rpi,
}

pub type BiquadType = biquad::DirectForm2Transposed<f32>;
pub type AudioFilter<'d> = audio_filter::Filter<'d, BiquadType>;

pub fn db_to_linear(db: f32) -> f32 {
    10.0_f32.powf(db / 20.0)
}
