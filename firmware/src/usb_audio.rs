use defmt::{info, panic};
use embassy_futures::join::join3;
use embassy_stm32::{peripherals, usb};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel;
use embassy_usb::class::uac1::speaker;
use embassy_usb::driver::EndpointError;

use crate::*;

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
    // Measured number of samples per frame (1 ms).
    let feedback_value: u32 = (SAMPLE_RATE_HZ / 1000) << 14;

    let feedback_value = [
        feedback_value as u8,
        (feedback_value >> 8) as u8,
        (feedback_value >> 16) as u8,
    ];

    loop {
        feedback.write_packet(&feedback_value).await?;
    }
}

async fn stream_handler<'d, T: usb::Instance + 'd>(
    stream: &mut speaker::Stream<'d, usb::Driver<'d, T>>,
    sender: &mut zerocopy_channel::Sender<'static, NoopRawMutex, SampleBlock>,
) -> Result<(), Disconnected> {
    loop {
        let mut usb_data = [0u8; USB_PACKET_SIZE];
        let data_size = stream.read_packet(&mut usb_data).await?;
        let word_count = data_size / SAMPLE_SIZE;

        if word_count * SAMPLE_SIZE == data_size {
            // Obtain a buffer from the channel
            let samples = sender.send().await;
            samples.clear();

            for w in 0..word_count {
                let byte_offset = w * SAMPLE_SIZE;
                let sample = audio_filter::sample_to_f32(u32::from_le_bytes(
                    usb_data[byte_offset..byte_offset + SAMPLE_SIZE].try_into().unwrap(),
                ));

                // Fill the sample buffer with data.
                samples.push(sample).unwrap();
            }

            sender.send_done();
        } else {
            info!("Invalid USB buffer size of {}, skipped.", data_size);
        }
    }
}

/// Runs handlers for USB communication.
#[embassy_executor::task]
pub async fn usb_audio_task(
    mut stream: speaker::Stream<'static, usb::Driver<'static, peripherals::USB_OTG_HS>>,
    mut feedback: speaker::Feedback<'static, usb::Driver<'static, peripherals::USB_OTG_HS>>,
    mut sender: zerocopy_channel::Sender<'static, NoopRawMutex, SampleBlock>,
    mut usb_device: embassy_usb::UsbDevice<'static, usb::Driver<'static, peripherals::USB_OTG_HS>>,
) {
    let feedback_fut = async {
        loop {
            feedback.wait_connection().await;
            _ = feedback_handler(&mut feedback).await;
        }
    };

    let stream_fut = async {
        loop {
            stream.wait_connection().await;

            info!("Stream connected");
            _ = stream_handler(&mut stream, &mut sender).await;
            info!("Stream disconnected");
        }
    };
    info!("Launch USB task.");

    // Run everything concurrently.
    join3(usb_device.run(), feedback_fut, stream_fut).await;
}
