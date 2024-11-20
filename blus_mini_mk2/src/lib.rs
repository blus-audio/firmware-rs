//! Commonly used definitions, settings, and thread synchronization instances.
#![no_std]
#![warn(missing_docs)]

pub mod audio_routing;
pub mod usb_audio;

use micromath::F32Ext;

use audio::AudioSource;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::signal::Signal;
use embassy_usb::class::uac1;
use heapless::Vec;

/// Stereo input.
pub const INPUT_CHANNEL_COUNT: usize = 2;

/// Two two-way speakers.
pub const OUTPUT_CHANNEL_COUNT: usize = 4;

/// Fixed sample rate.
pub const SAMPLE_RATE_HZ: u32 = 48_000;

/// The frequency of the timer that is used for USB feedback measurements.
pub const FEEDBACK_COUNTER_TICK_RATE: u32 = 24_576_000;

/// The globally used sample width.
pub const SAMPLE_WIDTH: uac1::SampleWidth = uac1::SampleWidth::Width4Byte;

/// The globally used sample width in bit.
pub const SAMPLE_WIDTH_BIT: usize = SAMPLE_WIDTH.in_bit();

/// The sample size in byte.
pub const SAMPLE_SIZE: usize = SAMPLE_WIDTH as usize;

/// The input sample size in byte per second of playback.
pub const SAMPLE_SIZE_PER_S: usize = (SAMPLE_RATE_HZ as usize) * INPUT_CHANNEL_COUNT * SAMPLE_SIZE;

/// The input sample size in byte per millisecond of playback (rounded up).
pub const SAMPLE_SIZE_PER_MS: usize = SAMPLE_SIZE_PER_S.div_ceil(1000);

/// Selection of audio channels for USB (left and right).
pub const USB_AUDIO_CHANNELS: [uac1::Channel; INPUT_CHANNEL_COUNT] =
    [uac1::Channel::LeftFront, uac1::Channel::RightFront];

/// Size of audio samples per 1 ms - suitable for full-speed USB.
#[cfg(not(feature = "usb_high_speed"))]
pub const USB_FRAME_SIZE: usize = SAMPLE_SIZE_PER_MS;

/// Size of audio samples per 125 us - suitable for high-speed USB.
#[cfg(feature = "usb_high_speed")]
pub const USB_FRAME_SIZE: usize = SAMPLE_SIZE_PER_MS.div_ceil(8);

/// Feedback refresh period for full-speed USB, selected as 8 ms.
#[cfg(not(feature = "usb_high_speed"))]
pub const FEEDBACK_REFRESH_PERIOD: uac1::FeedbackRefresh = uac1::FeedbackRefresh::Period8Frames;

/// Feedback refresh period for high-speed USB, selected as 8 ms.
#[cfg(feature = "usb_high_speed")]
pub const FEEDBACK_REFRESH_PERIOD: uac1::FeedbackRefresh = uac1::FeedbackRefresh::Period64Frames;

/// Use a maximum USB packet size with a factor of two of the regular frame size.
/// This leaves (excessive) room for the feedback mechanism.
pub const USB_MAX_PACKET_SIZE: usize = 2 * USB_FRAME_SIZE;

/// The maximum sample count that fits in a USB packet.
pub const USB_MAX_SAMPLE_COUNT: usize = USB_MAX_PACKET_SIZE / SAMPLE_SIZE;

/// The default sample count for sources other than USB.
/// Conveniently, a buffer of 1 ms size, just like for full-speed USB.
pub const DEFAULT_SAMPLE_COUNT: usize = SAMPLE_SIZE_PER_MS / SAMPLE_SIZE;

/// The maximum number of input samples per block of samples from any source (USB or otherwise).
pub const MAX_SAMPLE_COUNT: usize = if USB_MAX_SAMPLE_COUNT > DEFAULT_SAMPLE_COUNT {
    USB_MAX_SAMPLE_COUNT
} else {
    DEFAULT_SAMPLE_COUNT
};

// Thread synchronization
/// Signal for a new feedback value, sent by the feedback interrupt handler.
pub static FEEDBACK_SIGNAL: Signal<CriticalSectionRawMutex, u32> = Signal::new();

/// Signal that is emitted when the playback SAI becomes active. Carries the new source within.
pub static SAI_ACTIVE_SIGNAL: Signal<ThreadModeRawMutex, AudioSource> = Signal::new();

/// Signal that is emitted when amplifier setup is complete.
pub static AMP_SETUP_SIGNAL: Signal<ThreadModeRawMutex, bool> = Signal::new();

/// Signal that is emitted when there is a new gain setting for the USB input.
pub static USB_GAIN_SIGNAL: Signal<ThreadModeRawMutex, (f32, f32)> = Signal::new();

/// Signal that is emitted when there is a new gain setting for the USB input.
pub static POT_GAIN_SIGNAL: Signal<ThreadModeRawMutex, f32> = Signal::new();

// Type definitions
/// A sample block, originating from different sources.
#[derive(Debug)]
pub enum SampleBlock {
    /// Samples from USB.
    Usb(UsbSampleBlock),
    /// Samples from S/PDIF.
    Spdif(SpdifSampleBlock),
    /// Samples from the Raspberry Pi.
    Rpi(RpiSampleBlock),
}

/// The number of sample blocks that exist.
pub const SAMPLE_BLOCK_COUNT: usize = 5;

/// The type of data that the USB input generates.
pub type UsbSampleBlock = Vec<u32, USB_MAX_SAMPLE_COUNT>;

/// The type of data that the S/PDIF input generates.
pub type SpdifSampleBlock = [u32; DEFAULT_SAMPLE_COUNT];

/// The type of data that the Raspberry Pi input generates.
pub type RpiSampleBlock = [u32; DEFAULT_SAMPLE_COUNT];

/// The type of biquad filter that is used for processing.
pub type BiquadType = biquad::DirectForm2Transposed<f32>;

/// Convert a gain in dB to linear scale.
pub fn db_to_linear(db: f32) -> f32 {
    10.0_f32.powf(db / 20.0)
}
