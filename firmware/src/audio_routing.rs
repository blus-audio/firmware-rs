use defmt::{info, panic};
use embassy_stm32::gpio::Output;
use embassy_stm32::{peripherals, sai};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel;
use embassy_time::{Duration, Timer, WithTimeout};
use grounded::uninit::GroundedArrayCell;

use crate::*;

// Factor of two for being able to buffer two full USB packets (excessive)
const SAI_AMP_SAMPLE_COUNT: usize = 2 * USB_MAX_SAMPLE_COUNT;

// Buffer of around 10 ms size
const SAI_RPI_SAMPLE_COUNT: usize = SAMPLE_SIZE_PER_S.div_ceil(1000) / SAMPLE_SIZE;

#[allow(unused)]
pub struct Sai1Resources {
    pub sai: peripherals::SAI1,

    pub mclk_a: peripherals::PE2,
    pub sck_a: peripherals::PE5,
    pub sd_a: peripherals::PE6,
    pub fs_a: peripherals::PE4,
    pub dma_a: peripherals::DMA1_CH3,

    pub sd_b: peripherals::PE11,
    pub dma_b: peripherals::DMA1_CH4,
}

#[allow(unused)]
pub struct Sai4Resources {
    pub sai: peripherals::SAI4,

    pub mclk_a: peripherals::PE0,
    pub sck_a: peripherals::PD13,
    pub sd_a: peripherals::PC1,
    pub fs_a: peripherals::PD12,
    pub dma_a: peripherals::BDMA_CH0,

    pub sck_b: peripherals::PE12,
    pub sd_b: peripherals::PE11,
    pub fs_b: peripherals::PE13,
    pub dma_b: peripherals::BDMA_CH1,
}

// Accessible by BDMA (Zone D3)
#[link_section = ".sram4"]
static mut SAI_AMP_WRITE_BUFFER: GroundedArrayCell<u32, SAI_AMP_SAMPLE_COUNT> = GroundedArrayCell::uninit();

#[link_section = ".sram4"]
static mut SAI_RPI_READ_BUFFER: GroundedArrayCell<u32, SAI_RPI_SAMPLE_COUNT> = GroundedArrayCell::uninit();

fn new_sai4<'d>(
    resources: &'d mut Sai4Resources,
    sai4a_write_buffer: &'d mut [u32],
    sai4b_read_buffer: &'d mut [u32],
    sample_rate_hz: u32,
) -> (
    sai::Sai<'d, peripherals::SAI4, u32>,
    sai::Sai<'d, peripherals::SAI4, u32>,
) {
    let (sai4_a, sai4_b) = sai::split_subblocks(&mut resources.sai);

    let sai4_a_driver = {
        let mut config = sai::Config::default();

        config.slot_count = sai::word::U4(OUTPUT_CHANNEL_COUNT as u8);
        config.slot_enable = 0xFFFF; // All slots
        config.data_size = sai::DataSize::Data32;
        config.frame_sync_definition = sai::FrameSyncDefinition::StartOfFrame;
        config.frame_length = (OUTPUT_CHANNEL_COUNT * SAMPLE_WIDTH_BIT) as u8;
        config.bit_order = sai::BitOrder::MsbFirst;

        match sample_rate_hz {
            SAMPLE_RATE_HZ => config.master_clock_divider = sai::MasterClockDivider::Div2,
            _ => panic!("Unsupported SAI sample rate."),
        }

        sai::Sai::new_asynchronous(
            sai4_a,
            &mut resources.sck_a,
            &mut resources.sd_a,
            &mut resources.fs_a,
            &mut resources.dma_a,
            sai4a_write_buffer,
            config,
        )
    };

    let sai4_b_driver = {
        let mut config = sai::Config::default();
        const CHANNEL_COUNT: usize = 2;

        config.tx_rx = sai::TxRx::Receiver;
        config.slot_count = sai::word::U4(CHANNEL_COUNT as u8);
        config.slot_enable = 0xFFFF; // All slots
        config.data_size = sai::DataSize::Data32;
        // config.frame_sync_definition = sai::FrameSyncDefinition::StartOfFrame;
        config.frame_length = (CHANNEL_COUNT * SAMPLE_WIDTH_BIT) as u8;
        config.frame_sync_active_level_length = sai::word::U7(SAMPLE_WIDTH_BIT as u8);
        config.bit_order = sai::BitOrder::MsbFirst;

        match sample_rate_hz {
            SAMPLE_RATE_HZ => config.master_clock_divider = sai::MasterClockDivider::Div2,
            _ => panic!("Unsupported SAI sample rate."),
        }

        sai::Sai::new_asynchronous(
            sai4_b,
            &mut resources.sck_b,
            &mut resources.sd_b,
            &mut resources.fs_b,
            &mut resources.dma_b,
            sai4b_read_buffer,
            config,
        )
    };

    (sai4_a_driver, sai4_b_driver)
}

fn process(
    samples: &[u32],
    processed_samples: &mut [u32],
    filters: &mut [AudioFilter; OUTPUT_CHANNEL_COUNT],
    gain_left: f32,
    gain_right: f32,
) -> usize {
    for index in 0..samples.len() {
        let sample = audio_filter::sample_to_f32(samples[index]);

        if index % 2 == 0 {
            // Left channel
            processed_samples[2 * index] = audio_filter::sample_to_u32(filters[0].run(sample) * gain_left);
            processed_samples[2 * index + 1] = audio_filter::sample_to_u32(filters[1].run(sample) * gain_left);
        } else {
            // Right channel
            processed_samples[2 * index] = audio_filter::sample_to_u32(filters[2].run(sample) * gain_right);
            processed_samples[2 * index + 1] = audio_filter::sample_to_u32(filters[3].run(sample) * gain_right);
        }
    }

    2 * samples.len()
}

#[embassy_executor::task]
pub async fn source_control_task(
    source_publisher: SourcePublisher,
    mut led_usb: Output<'static>,
    mut led_rpi: Output<'static>,
    mut led_ext: Output<'static>,
) {
    let mut source = AudioSource::RaspberryPi;

    loop {
        for led in [&mut led_usb, &mut led_rpi, &mut led_ext] {
            led.set_low();
        }

        match source {
            AudioSource::Usb => led_usb.set_high(),
            AudioSource::RaspberryPi => led_rpi.set_high(),
            _ => (),
        }
        source_publisher.publish_immediate(source);
        let usb_signal = USB_STREAMING_SIGNAL.wait().await;

        if usb_signal {
            source = AudioSource::Usb;
        } else {
            source = AudioSource::RaspberryPi;
        }
    }
}

#[embassy_executor::task]
pub async fn audio_routing_task(
    mut filters: [AudioFilter<'static>; OUTPUT_CHANNEL_COUNT],
    mut sai4_resources: Sai4Resources,
    mut usb_audio_receiver: zerocopy_channel::Receiver<'static, NoopRawMutex, SampleBlock>,
    mut source_subscriber: SourceSubscriber,
) {
    info!("Amplifier SAI write buffer size: {} samples", SAI_AMP_SAMPLE_COUNT);
    info!("Raspberry Pi SAI read buffer size: {} samples", SAI_RPI_SAMPLE_COUNT);

    let sai_amp_write_buffer: &mut [u32] = unsafe {
        SAI_AMP_WRITE_BUFFER.initialize_all_copied(0);
        let (ptr, len) = SAI_AMP_WRITE_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let sai_rpi_read_buffer: &mut [u32] = unsafe {
        SAI_RPI_READ_BUFFER.initialize_all_copied(0);
        let (ptr, len) = SAI_RPI_READ_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let (mut sai_amp, mut sai_rpi) = new_sai4(
        &mut sai4_resources,
        sai_amp_write_buffer,
        sai_rpi_read_buffer,
        SAMPLE_RATE_HZ,
    );

    let mut gain_left = 0.0;
    let mut gain_right = 0.0;

    let mut rpi_samples = [0u32; SAI_RPI_SAMPLE_COUNT];

    let mut source = AudioSource::RaspberryPi;

    sai_amp.start();
    sai_rpi.start();

    loop {
        if let Some(message) = source_subscriber.try_next_message_pure() {
            source = message;
        }

        if let Some(gain) = GAIN_SIGNAL.try_take() {
            (gain_left, gain_right) = gain;

            if gain_left == 0.0 && gain_right == 0.0 {
                sai_amp.set_mute(true);
            } else {
                sai_amp.set_mute(false);
            }
        }

        let renew = match source {
            AudioSource::RaspberryPi => {
                let result = sai_rpi.read(&mut rpi_samples).await;

                if result.is_err() {
                    true
                } else {
                    let mut processed_samples = [0u32; 2 * SAI_RPI_SAMPLE_COUNT];

                    let output_sample_count = process(
                        &rpi_samples,
                        &mut processed_samples,
                        &mut filters,
                        gain_left,
                        gain_right,
                    );

                    let result = sai_amp.write(&processed_samples[..output_sample_count]).await;
                    result.is_err()
                }
            }
            AudioSource::Usb => {
                let result = usb_audio_receiver
                    .receive()
                    .with_timeout(Duration::from_millis(10))
                    .await;

                match result {
                    Ok(samples) => {
                        let mut processed_samples = [0u32; 2 * USB_MAX_SAMPLE_COUNT];
                        let output_sample_count =
                            process(samples, &mut processed_samples, &mut filters, gain_left, gain_right);
                        let result = sai_amp.write(&processed_samples[..output_sample_count]).await;

                        // Notify the channel that the buffer is now ready to be reused
                        usb_audio_receiver.receive_done();

                        result.is_err()
                    }
                    Err(_) => true,
                }
            }
            _ => {
                Timer::after_millis(100).await;
                true
            }
        };

        if renew {
            SAI_ACTIVE_SIGNAL.signal(false);
            info!("Renew SAI setup.");

            drop(sai_amp);
            drop(sai_rpi);

            (sai_amp, sai_rpi) = new_sai4(
                &mut sai4_resources,
                sai_amp_write_buffer,
                sai_rpi_read_buffer,
                SAMPLE_RATE_HZ,
            );

            sai_amp.start();
            sai_rpi.start();

            SAI_ACTIVE_SIGNAL.signal(true);
        }
    }
}
