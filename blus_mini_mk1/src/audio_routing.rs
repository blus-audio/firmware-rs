use defmt::info;
use embassy_stm32::time::Hertz;
use embassy_stm32::{i2s, peripherals};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel;
use embassy_time::{Duration, WithTimeout as _};

use crate::*;

#[allow(unused)]
pub struct I2sResources {
    pub i2s: peripherals::SPI3,

    pub ck: peripherals::PC10,
    pub sd: peripherals::PC12,
    pub ws: peripherals::PA4,
    pub mck: peripherals::PC7,
    pub dma: peripherals::DMA1_CH7,
}

fn new_i2s<'d>(resources: &'d mut I2sResources) -> i2s::I2S<'d> {
    let mut config = i2s::Config::default();

    config.format = i2s::Format::Data32Channel32;

    i2s::I2S::new_txonly(
        &mut resources.i2s,
        &mut resources.sd,
        &mut resources.ws,
        &mut resources.ck,
        &mut resources.mck,
        &mut resources.dma,
        Hertz((SAMPLE_WIDTH_BIT * INPUT_CHANNEL_COUNT * SAMPLE_RATE_HZ as usize) as u32),
        config,
    )
}

#[embassy_executor::task]
pub async fn audio_routing_task(
    mut i2s_resources: I2sResources,
    mut usb_audio_receiver: zerocopy_channel::Receiver<'static, NoopRawMutex, UsbSampleBlock>,
) {
    let mut i2s_amp = new_i2s(&mut i2s_resources);

    loop {
        // Data should arrive at least once every millisecond.
        let result = usb_audio_receiver
            .receive()
            .with_timeout(Duration::from_millis(2))
            .await;

        let renew = if let Ok(samples) = result {
            let result = i2s_amp.write(samples).await;

            // Notify the channel that the buffer is now ready to be reused
            usb_audio_receiver.receive_done();

            result.is_err()
        } else {
            true
        };

        // Renew the I2S setup in case of errors.
        if renew {
            info!("Renew I2S");
            I2S_ACTIVE_SIGNAL.signal(false);

            drop(i2s_amp);

            i2s_amp = new_i2s(&mut i2s_resources);

            I2S_ACTIVE_SIGNAL.signal(true);
        }
    }
}
