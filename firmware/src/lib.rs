#![no_std]

pub mod audio_filter;
pub mod audio_routing;
pub mod tas2780;
pub mod usb_audio;

use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_sync::signal::Signal;
use embassy_usb::class::uac1;
use heapless::Vec;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AudioSource {
    None,
    Usb,
    Spdif,
    Ext,
    RaspberryPi,
}

// Stereo input -> two two-way speakers
pub const INPUT_CHANNEL_COUNT: usize = 2;
pub const OUTPUT_CHANNEL_COUNT: usize = 4;

pub const SAMPLE_RATE_HZ: u32 = 48_000;
pub const FEEDBACK_COUNTER_TICK_RATE: u32 = 24_576_000;

pub const SAMPLE_WIDTH: uac1::SampleWidth = uac1::SampleWidth::Width4Byte;
pub const SAMPLE_WIDTH_BIT: usize = SAMPLE_WIDTH.in_bit();
pub const SAMPLE_SIZE: usize = SAMPLE_WIDTH as usize;
pub const SAMPLE_SIZE_PER_S: usize = (SAMPLE_RATE_HZ as usize) * INPUT_CHANNEL_COUNT * SAMPLE_SIZE;

// Size of audio samples per 1 ms - suitable for full-speed USB
#[cfg(not(feature = "usb_high_speed"))]
pub const USB_FRAME_SIZE: usize = SAMPLE_SIZE_PER_S.div_ceil(1000);

// Size of audio samples per 125 us - suitable for high-speed USB
#[cfg(feature = "usb_high_speed")]
pub const USB_FRAME_SIZE: usize = SAMPLE_SIZE_PER_S.div_ceil(8000);

// 8 ms period
#[cfg(not(feature = "usb_high_speed"))]
pub const FEEDBACK_REFRESH_PERIOD: uac1::FeedbackRefresh = uac1::FeedbackRefresh::Period8Frames;

// 8 ms period
#[cfg(feature = "usb_high_speed")]
pub const FEEDBACK_REFRESH_PERIOD: uac1::FeedbackRefresh = uac1::FeedbackRefresh::Period64Frames;

// Factor of two as a margin for feedback (excessive)
pub const USB_MAX_PACKET_SIZE: usize = 2 * USB_FRAME_SIZE;
pub const USB_MAX_SAMPLE_COUNT: usize = USB_MAX_PACKET_SIZE / SAMPLE_SIZE;

// Signals and channels for task communication
pub static USB_STREAMING_SIGNAL: Signal<ThreadModeRawMutex, bool> = Signal::new();
pub static RPI_STREAMING_SIGNAL: Signal<ThreadModeRawMutex, bool> = Signal::new();
pub static FEEDBACK_SIGNAL: Signal<CriticalSectionRawMutex, u32> = Signal::new();
pub static SAI_ACTIVE_SIGNAL: Signal<ThreadModeRawMutex, bool> = Signal::new();
pub static USB_VOLUME_SIGNAL: Signal<ThreadModeRawMutex, Vec<uac1::speaker::Volume, INPUT_CHANNEL_COUNT>> =
    Signal::new();
pub static GAIN_SIGNAL: Signal<ThreadModeRawMutex, (f32, f32)> = Signal::new();

pub static SOURCE_CHANNEL: PubSubChannel<ThreadModeRawMutex, AudioSource, 1, 2, 1> = PubSubChannel::new();

pub type SourceSubscriber = Subscriber<'static, ThreadModeRawMutex, AudioSource, 1, 2, 1>;
pub type SourcePublisher = Publisher<'static, ThreadModeRawMutex, AudioSource, 1, 2, 1>;

// Type definitions
pub type SampleBlock = Vec<u32, USB_MAX_SAMPLE_COUNT>;
pub type BiquadType = biquad::DirectForm2Transposed<f32>;
pub type AudioFilter<'d> = audio_filter::Filter<'d, BiquadType>;
