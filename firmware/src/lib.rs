#![no_std]

pub mod audio_filter;
pub mod audio_routing;
pub mod tas2780;
pub mod usb_audio;

use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::signal::Signal;
use embassy_usb::class::uac1;
use heapless::Vec;

pub static SAI_ACTIVE_SIGNAL: Signal<ThreadModeRawMutex, bool> = Signal::new();

pub const INPUT_CHANNEL_COUNT: usize = 2;
pub const OUTPUT_CHANNEL_COUNT: usize = 4;
pub const SAMPLE_RATE_HZ: u32 = 48000;

pub const SAMPLE_WIDTH: uac1::SampleWidth = uac1::SampleWidth::Width4Byte;
pub const SAMPLE_SIZE: usize = SAMPLE_WIDTH as usize;
pub const SAMPLE_BIT_COUNT: usize = SAMPLE_WIDTH.in_bit();

// Size of audio samples per 1 ms - suitable for full-speed USB
pub const AUDIO_SIZE_PER_MS: usize = (SAMPLE_RATE_HZ as usize / 1000) * INPUT_CHANNEL_COUNT * SAMPLE_SIZE;

// Factor of two as a margin for feedback (excessive)
pub const USB_PACKET_SIZE: usize = 2 * AUDIO_SIZE_PER_MS;
pub const SAMPLE_COUNT: usize = USB_PACKET_SIZE / SAMPLE_SIZE;

// 1 ms for USB high-speed, 8 ms for USB full-speed.
pub const FEEDBACK_REFRESH_PERIOD: uac1::FeedbackRefreshPeriod = uac1::FeedbackRefreshPeriod::Period8Frames;
pub static FEEDBACK_SIGNAL: Signal<CriticalSectionRawMutex, u32> = Signal::new();

pub type SampleBlock = Vec<f32, SAMPLE_COUNT>;

pub type BiquadType = biquad::DirectForm2Transposed<f32>;
pub type AudioFilter = audio_filter::Filter<BiquadType>;
