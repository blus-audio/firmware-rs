use core::sync::atomic::Ordering::Relaxed;
use defmt::{debug, panic};
use embassy_stm32::{peripherals, usb};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel;
use embassy_usb::class::uac1::speaker;
use embassy_usb::driver::EndpointError;
use static_assertions;

use crate::*;

// Number of ticks of the feedback timer per audio sample period.
const TICKS_PER_SAMPLE: u32 = FEEDBACK_COUNTER_TICK_RATE / SAMPLE_RATE_HZ;
static_assertions::const_assert_eq!(TICKS_PER_SAMPLE * SAMPLE_RATE_HZ, FEEDBACK_COUNTER_TICK_RATE);

// Feedback is provided in 16.16 format for high-speed endpoints.
#[cfg(feature = "usb_high_speed")]
const FEEDBACK_SHIFT: usize = 16;

// Feedback is provided in 10.14 format for full-speed endpoints.
#[cfg(not(feature = "usb_high_speed"))]
const FEEDBACK_SHIFT: usize = 14;

const FEEDBACK_FACTOR: u32 = ((1 << FEEDBACK_SHIFT) / TICKS_PER_SAMPLE) / FEEDBACK_REFRESH_PERIOD.frame_count() as u32;
static_assertions::const_assert_eq!(
    (FEEDBACK_FACTOR << (FEEDBACK_REFRESH_PERIOD as usize)) * TICKS_PER_SAMPLE,
    (1 << FEEDBACK_SHIFT)
);

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn feedback_handler<'d, T: usb::Instance + 'd>(
    feedback: &mut speaker::Feedback<'d, usb::Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut packet: Vec<u8, 4> = Vec::new();

    loop {
        let counter = FEEDBACK_SIGNAL.wait().await;

        packet.clear();

        let value = counter * FEEDBACK_FACTOR;

        #[cfg(feature = "usb_high_speed")]
        {
            packet.push(value as u8).unwrap();
            packet.push((value >> 8) as u8).unwrap();
            packet.push((value >> 16) as u8).unwrap();
            packet.push((value >> 24) as u8).unwrap();
        }

        #[cfg(not(feature = "usb_high_speed"))]
        {
            packet.push(value as u8).unwrap();
            packet.push((value >> 8) as u8).unwrap();
            packet.push((value >> 16) as u8).unwrap();
        }

        feedback.write_packet(&packet).await?;
    }
}

async fn stream_handler<'d, T: usb::Instance + 'd>(
    stream: &mut speaker::Stream<'d, usb::Driver<'d, T>>,
    sender: &mut zerocopy_channel::Sender<'static, NoopRawMutex, UsbSampleBlock>,
) -> Result<(), Disconnected> {
    loop {
        let mut usb_data = [0u8; USB_MAX_PACKET_SIZE];
        let data_size = stream.read_packet(&mut usb_data).await?;
        USB_IS_STREAMING.store(true, Relaxed);

        let word_count = data_size / SAMPLE_SIZE;

        if word_count * SAMPLE_SIZE == data_size {
            // Obtain a buffer from the channel
            let samples = sender.send().await;
            samples.clear();

            for w in 0..word_count {
                let byte_offset = w * SAMPLE_SIZE;
                let sample = u32::from_le_bytes(usb_data[byte_offset..byte_offset + SAMPLE_SIZE].try_into().unwrap());

                // Fill the sample buffer with data.
                samples.push(sample).unwrap();
            }

            sender.send_done();
        } else {
            debug!("Invalid USB buffer size of {}, skipped.", data_size);
        }
    }
}

#[embassy_executor::task]
pub async fn streaming_task(
    mut stream: speaker::Stream<'static, usb::Driver<'static, peripherals::USB_OTG_HS>>,
    mut sender: zerocopy_channel::Sender<'static, NoopRawMutex, UsbSampleBlock>,
) {
    loop {
        stream.wait_connection().await;
        _ = stream_handler(&mut stream, &mut sender).await;
        USB_IS_STREAMING.store(false, Relaxed);
    }
}

#[embassy_executor::task]
pub async fn feedback_task(mut feedback: speaker::Feedback<'static, usb::Driver<'static, peripherals::USB_OTG_HS>>) {
    loop {
        feedback.wait_connection().await;
        _ = feedback_handler(&mut feedback).await;
    }
}

#[embassy_executor::task]
pub async fn usb_task(mut usb_device: embassy_usb::UsbDevice<'static, usb::Driver<'static, peripherals::USB_OTG_HS>>) {
    usb_device.run().await;
}

#[embassy_executor::task]
pub async fn control_task(control_monitor: speaker::ControlMonitor<'static>) {
    loop {
        control_monitor.changed().await;

        let mut usb_gain_left = 0.0_f32;
        let mut usb_gain_right = 0.0_f32;

        for channel in AUDIO_CHANNELS {
            let volume = control_monitor.volume(channel).unwrap();

            let gain = match volume {
                speaker::Volume::Muted => 0.0,
                speaker::Volume::DeciBel(volume_db) => {
                    if volume_db > 0.0 {
                        panic!("Volume must not be positive.")
                    }

                    db_to_linear(volume_db)
                }
            };

            match channel {
                uac1::Channel::LeftFront => {
                    usb_gain_left = gain;
                }
                uac1::Channel::RightFront => {
                    usb_gain_right = gain;
                }
                _ => (),
            }
        }

        USB_GAIN_SIGNAL.signal((usb_gain_left, usb_gain_right));
    }
}
