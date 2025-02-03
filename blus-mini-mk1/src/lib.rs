#![no_std]

pub mod audio_routing;
pub mod usb_audio;

use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::signal::Signal;
use embassy_usb::class::uac1;
use heapless::Vec;

// Stereo input -> two two-way speakers
pub const INPUT_CHANNEL_COUNT: usize = 2;

pub const SAMPLE_RATE_HZ: u32 = 48_000;
pub const FEEDBACK_COUNTER_TICK_RATE: u32 = 24_576_000 / 2;

pub const SAMPLE_WIDTH: uac1::SampleWidth = uac1::SampleWidth::Width4Byte;
pub const SAMPLE_WIDTH_BIT: usize = SAMPLE_WIDTH.in_bit();
pub const SAMPLE_SIZE: usize = SAMPLE_WIDTH as usize;
pub const SAMPLE_SIZE_PER_S: usize = (SAMPLE_RATE_HZ as usize) * INPUT_CHANNEL_COUNT * SAMPLE_SIZE;
pub const SAMPLE_SIZE_PER_MS: usize = SAMPLE_SIZE_PER_S.div_ceil(1000);

pub const AUDIO_CHANNELS: [uac1::Channel; INPUT_CHANNEL_COUNT] = [uac1::Channel::LeftFront, uac1::Channel::RightFront];

// Size of audio samples per 1 ms - suitable for full-speed USB
pub const USB_FRAME_SIZE: usize = SAMPLE_SIZE_PER_MS;

// 8 ms period
pub const FEEDBACK_REFRESH_PERIOD: uac1::FeedbackRefresh = uac1::FeedbackRefresh::Period8Frames;

// Factor of two as a margin for feedback (excessive)
pub const USB_MAX_PACKET_SIZE: usize = 2 * USB_FRAME_SIZE;
pub const USB_MAX_SAMPLE_COUNT: usize = USB_MAX_PACKET_SIZE / SAMPLE_SIZE;

// Task communication
pub static FEEDBACK_SIGNAL: Signal<CriticalSectionRawMutex, u32> = Signal::new();
pub static I2S_ACTIVE_SIGNAL: Signal<ThreadModeRawMutex, bool> = Signal::new();
pub static VOLUME_SIGNAL: Signal<ThreadModeRawMutex, (f32, f32)> = Signal::new();

// Type definitions
pub type UsbSampleBlock = Vec<u32, USB_MAX_SAMPLE_COUNT>;
