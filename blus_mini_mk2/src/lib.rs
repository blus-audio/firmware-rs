#![no_std]

pub mod audio_routing;
pub mod usb_audio;

use micromath::F32Ext;

use audio::AudioSource;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::signal::Signal;
use embassy_usb::class::uac1;
use heapless::Vec;

// Stereo input -> two two-way speakers
pub const INPUT_CHANNEL_COUNT: usize = 2;
pub const OUTPUT_CHANNEL_COUNT: usize = 4;

pub const SAMPLE_RATE_HZ: u32 = 48_000;
pub const FEEDBACK_COUNTER_TICK_RATE: u32 = 24_576_000;

pub const SAMPLE_WIDTH: uac1::SampleWidth = uac1::SampleWidth::Width4Byte;
pub const SAMPLE_WIDTH_BIT: usize = SAMPLE_WIDTH.in_bit();
pub const SAMPLE_SIZE: usize = SAMPLE_WIDTH as usize;
pub const SAMPLE_SIZE_PER_S: usize = (SAMPLE_RATE_HZ as usize) * INPUT_CHANNEL_COUNT * SAMPLE_SIZE;
pub const SAMPLE_SIZE_PER_MS: usize = SAMPLE_SIZE_PER_S.div_ceil(1000);

pub const AUDIO_CHANNELS: [uac1::Channel; INPUT_CHANNEL_COUNT] = [uac1::Channel::LeftFront, uac1::Channel::RightFront];

// Size of audio samples per 1 ms - suitable for full-speed USB
#[cfg(not(feature = "usb_high_speed"))]
pub const USB_FRAME_SIZE: usize = SAMPLE_SIZE_PER_MS;

// Size of audio samples per 125 us - suitable for high-speed USB
#[cfg(feature = "usb_high_speed")]
pub const USB_FRAME_SIZE: usize = SAMPLE_SIZE_PER_MS.div_ceil(8);

// 8 ms period
#[cfg(not(feature = "usb_high_speed"))]
pub const FEEDBACK_REFRESH_PERIOD: uac1::FeedbackRefresh = uac1::FeedbackRefresh::Period8Frames;

// 8 ms period
#[cfg(feature = "usb_high_speed")]
pub const FEEDBACK_REFRESH_PERIOD: uac1::FeedbackRefresh = uac1::FeedbackRefresh::Period64Frames;

// Factor of two as a margin for feedback (excessive)
pub const USB_MAX_PACKET_SIZE: usize = 2 * USB_FRAME_SIZE;
pub const USB_MAX_SAMPLE_COUNT: usize = USB_MAX_PACKET_SIZE / SAMPLE_SIZE;

// Buffer of around 1 ms size
pub const SPDIF_SAMPLE_COUNT: usize = SAMPLE_SIZE_PER_MS / SAMPLE_SIZE;
pub const RPI_SAMPLE_COUNT: usize = SAMPLE_SIZE_PER_MS / SAMPLE_SIZE;

pub const MAX_SAMPLE_COUNT: usize = if USB_MAX_SAMPLE_COUNT > SPDIF_SAMPLE_COUNT {
    USB_MAX_SAMPLE_COUNT
} else {
    SPDIF_SAMPLE_COUNT
};

// Task communication
pub static FEEDBACK_SIGNAL: Signal<CriticalSectionRawMutex, u32> = Signal::new();
pub static SAI_ACTIVE_SIGNAL: Signal<ThreadModeRawMutex, AudioSource> = Signal::new();
pub static AMP_SETUP_SIGNAL: Signal<ThreadModeRawMutex, bool> = Signal::new();

pub static USB_GAIN_SIGNAL: Signal<ThreadModeRawMutex, (f32, f32)> = Signal::new();
pub static POT_GAIN_SIGNAL: Signal<ThreadModeRawMutex, f32> = Signal::new();

pub static AUDIO_SOURCE_SIGNAL: Signal<ThreadModeRawMutex, AudioSource> = Signal::new();

// Type definitions
pub const SAMPLE_BLOCK_COUNT: usize = 5;
pub type UsbSampleBlock = Vec<u32, USB_MAX_SAMPLE_COUNT>;
pub type SpdifSampleBlock = [u32; SPDIF_SAMPLE_COUNT];
pub type RpiSampleBlock = [u32; RPI_SAMPLE_COUNT];

pub type BiquadType = biquad::DirectForm2Transposed<f32>;

#[derive(Debug)]
pub enum SampleBlock {
    Usb(UsbSampleBlock),
    Spdif(SpdifSampleBlock),
    Rpi(RpiSampleBlock),
}

pub fn db_to_linear(db: f32) -> f32 {
    10.0_f32.powf(db / 20.0)
}
