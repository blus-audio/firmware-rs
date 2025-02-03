use audio::audio_filter::{sample_to_f32, sample_to_u32};
use defmt::info;
use embassy_stm32::time::Hertz;
use embassy_stm32::{i2s, peripherals};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel;
use embassy_time::{Duration, WithTimeout as _};

use crate::*;

#[allow(unused)]
pub struct I2sResources<'d> {
    pub i2s: peripherals::SPI2,

    pub ck: peripherals::PB10,
    pub sd: peripherals::PB15,
    pub ws: peripherals::PB12,
    pub dma: peripherals::DMA1_CH4,
    pub dma_buf: &'d mut [u16],
}

fn new_i2s<'d>(resources: &'d mut I2sResources) -> i2s::I2S<'d, u16> {
    let mut config = i2s::Config::default();
    config.format = i2s::Format::Data32Channel32;
    config.master_clock = false;

    i2s::I2S::new_txonly_nomck(
        &mut resources.i2s,
        &mut resources.sd,
        &mut resources.ws,
        &mut resources.ck,
        &mut resources.dma,
        resources.dma_buf,
        Hertz(SAMPLE_RATE_HZ),
        config,
    )
}

#[embassy_executor::task]
pub async fn audio_routing_task(
    mut i2s_resources: I2sResources<'static>,
    mut usb_audio_receiver: zerocopy_channel::Receiver<'static, NoopRawMutex, UsbSampleBlock>,
) {
    let mut volume = (0.0, 0.0);
    let mut i2s_dac = new_i2s(&mut i2s_resources);
    let mut running = false;

    loop {
        // Data should arrive at least once every millisecond.
        let result = usb_audio_receiver
            .receive()
            .with_timeout(Duration::from_millis(2))
            .await;

        if let Some(new_volume) = VOLUME_SIGNAL.try_take() {
            volume = new_volume;
        }

        let error = if let Ok(samples) = result {
            let mut processed_samples: Vec<u16, { USB_MAX_SAMPLE_COUNT * 2 }> = Vec::new();

            for (index, sample) in samples.iter().enumerate() {
                let sample_f32 = sample_to_f32(*sample);

                let sample_f32 = if index % 2 == 0 {
                    sample_f32 * volume.0
                } else {
                    sample_f32 * volume.1
                };

                let sample = sample_to_u32(sample_f32);
                let low = sample as u16;
                let high = (sample >> 16) as u16;

                processed_samples.push(high).unwrap();
                processed_samples.push(low).unwrap();
            }

            let error = if !running {
                info!("Start I2S");

                i2s_dac.clear();
                let error = i2s_dac.write_immediate(&processed_samples).await.is_err();
                i2s_dac.start();
                running = true;
                error
            } else {
                i2s_dac.write(&processed_samples).await.is_err()
            };

            // Notify the channel that the buffer is now ready to be reused
            usb_audio_receiver.receive_done();

            error
        } else {
            true
        };

        // Stop I2S in case of errors or stopped streaming.
        if error && running {
            info!("Stop I2S");

            i2s_dac.stop().await;
            drop(i2s_dac);
            i2s_dac = new_i2s(&mut i2s_resources);
            running = false;
        }
    }
}
