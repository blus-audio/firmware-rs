use audio::{audio_filter, AudioFilter};
use core::sync::atomic::Ordering::Relaxed;
use defmt::{info, panic};
use embassy_stm32::gpio::Output;
use embassy_stm32::{peripherals, sai};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel;
use embassy_time::WithTimeout;
use embassy_time::{Duration, Timer};
use grounded::uninit::GroundedArrayCell;
use static_assertions;

use crate::*;

// Factor of two for being able to buffer two full USB packets (excessive)
const SAI_AMP_SAMPLE_COUNT: usize = 2 * USB_MAX_SAMPLE_COUNT;

// Buffer of around 1 ms size
const SAI_RPI_SAMPLE_COUNT: usize = SAMPLE_SIZE_PER_S.div_ceil(1000) / SAMPLE_SIZE;

// Do not exceed the write buffer (after processing) with samples from the Raspberry Pi.
static_assertions::const_assert!(
    SAI_RPI_SAMPLE_COUNT <= (OUTPUT_CHANNEL_COUNT / INPUT_CHANNEL_COUNT) * SAI_AMP_SAMPLE_COUNT
);

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

fn new_sai_amp_rpi<'d>(
    resources: &'d mut Sai4Resources,
    sai_amp_write_buffer: &'d mut [u32],
    sai_rpi_read_buffer: &'d mut [u32],
    sample_rate_hz: u32,
    audio_source: AudioSource,
) -> (
    sai::Sai<'d, peripherals::SAI4, u32>,
    sai::Sai<'d, peripherals::SAI4, u32>,
) {
    let clk_source = match audio_source {
        AudioSource::Spdif => embassy_stm32::pac::rcc::vals::Saiasel::_RESERVED_5,
        _ => embassy_stm32::pac::rcc::vals::Saiasel::PLL1_Q,
    };

    embassy_stm32::pac::RCC.d3ccipr().modify(|w| {
        w.set_sai4asel(clk_source);
    });

    let (sai_amp, sai_rpi) = sai::split_subblocks(&mut resources.sai);

    let sai_amp_driver = {
        let mut config = sai::Config::default();

        config.slot_count = sai::word::U4(OUTPUT_CHANNEL_COUNT as u8);
        config.slot_enable = 0xFFFF; // All slots
        config.frame_sync_definition = sai::FrameSyncDefinition::StartOfFrame;
        config.bit_order = sai::BitOrder::MsbFirst;

        match audio_source {
            AudioSource::Spdif => {
                config.data_size = sai::DataSize::Data16;
                config.frame_length = (OUTPUT_CHANNEL_COUNT * 16) as u8;

                config.master_clock_divider = sai::MasterClockDivider::MasterClockDisabled;
            }
            _ => {
                assert_eq!(SAMPLE_WIDTH_BIT, 32);
                config.data_size = sai::DataSize::Data32;
                config.frame_length = (OUTPUT_CHANNEL_COUNT * 32) as u8;

                match sample_rate_hz {
                    SAMPLE_RATE_HZ => config.master_clock_divider = sai::MasterClockDivider::Div2,
                    _ => panic!("Unsupported SAI sample rate."),
                }
            }
        };

        sai::Sai::new_asynchronous(
            sai_amp,
            &mut resources.sck_a,
            &mut resources.sd_a,
            &mut resources.fs_a,
            &mut resources.dma_a,
            sai_amp_write_buffer,
            config,
        )
    };

    let sai_rpi_driver = {
        let mut config = sai::Config::default();
        const CHANNEL_COUNT: usize = INPUT_CHANNEL_COUNT;

        config.tx_rx = sai::TxRx::Receiver;
        config.slot_count = sai::word::U4(CHANNEL_COUNT as u8);
        config.slot_enable = 0xFFFF; // All slots
        config.data_size = sai::DataSize::Data32;
        config.frame_length = (CHANNEL_COUNT * SAMPLE_WIDTH_BIT) as u8;
        config.frame_sync_active_level_length = sai::word::U7(SAMPLE_WIDTH_BIT as u8);
        config.bit_order = sai::BitOrder::MsbFirst;
        config.mute_value = sai::MuteValue::LastValue;

        match sample_rate_hz {
            SAMPLE_RATE_HZ => config.master_clock_divider = sai::MasterClockDivider::Div2,
            _ => panic!("Unsupported SAI sample rate."),
        }

        sai::Sai::new_asynchronous(
            sai_rpi,
            &mut resources.sck_b,
            &mut resources.sd_b,
            &mut resources.fs_b,
            &mut resources.dma_b,
            sai_rpi_read_buffer,
            config,
        )
    };

    (sai_amp_driver, sai_rpi_driver)
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
pub async fn audio_routing_task(
    mut filters: [AudioFilter<'static>; OUTPUT_CHANNEL_COUNT],
    mut sai4_resources: Sai4Resources,
    mut usb_audio_receiver: zerocopy_channel::Receiver<'static, NoopRawMutex, UsbSampleBlock>,
    mut spdif_audio_receiver: zerocopy_channel::Receiver<'static, NoopRawMutex, SpdifSampleBlock>,
    mut status_led: Output<'static>,
    mut led_usb: Output<'static>,
    mut led_rpi: Output<'static>,
    mut led_spdif: Output<'static>,
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

    let (mut sai_amp, mut sai_rpi) = new_sai_amp_rpi(
        &mut sai4_resources,
        sai_amp_write_buffer,
        sai_rpi_read_buffer,
        SAMPLE_RATE_HZ,
        AudioSource::None,
    );

    let mut usb_gain = (0.0, 0.0);
    let mut pot_gain = (0.0, 0.0);

    let mut rpi_samples = [0u32; SAI_RPI_SAMPLE_COUNT];

    sai_amp.start();
    sai_rpi.start();

    loop {
        // Update the audio source.
        for led in [&mut led_usb, &mut led_rpi, &mut led_spdif] {
            led.set_low();
        }

        // Source prioritization is determined by the order of checks.
        let audio_source = if RPI_IS_STREAMING.load(Relaxed) {
            led_rpi.set_high();
            AudioSource::RaspberryPi
        } else if USB_IS_STREAMING.load(Relaxed) {
            led_usb.set_high();
            AudioSource::Usb
        } else {
            AudioSource::None
        };

        // let audio_source = AudioSource::Spdif;

        let renew = match audio_source {
            AudioSource::RaspberryPi => {
                let result = sai_rpi.read(&mut rpi_samples).await;

                if result.is_err() {
                    true
                } else {
                    let mut processed_samples = [0u32; 2 * SAI_RPI_SAMPLE_COUNT];

                    status_led.set_high();
                    let output_sample_count = process(&rpi_samples, &mut processed_samples, &mut filters, 1.0, 1.0);
                    status_led.set_low();

                    let result = sai_amp.write(&processed_samples[..output_sample_count]).await;
                    result.is_err()
                }
            }
            AudioSource::Usb => {
                if let Some(gain) = USB_GAIN_SIGNAL.try_take() {
                    usb_gain = gain;
                }

                // Data should arrive at least once every millisecond.
                let result = usb_audio_receiver
                    .receive()
                    .with_timeout(Duration::from_millis(5))
                    .await;

                if let Ok(samples) = result {
                    let mut processed_samples = [0u32; 2 * USB_MAX_SAMPLE_COUNT];

                    status_led.set_high();
                    let output_sample_count =
                        process(samples, &mut processed_samples, &mut filters, usb_gain.0, usb_gain.1);
                    status_led.set_low();

                    let result = sai_amp.write(&processed_samples[..output_sample_count]).await;

                    // Notify the channel that the buffer is now ready to be reused
                    usb_audio_receiver.receive_done();

                    result.is_err()
                } else {
                    USB_IS_STREAMING.store(false, Relaxed);
                    true
                }
            }
            AudioSource::None => {
                sai_amp.set_mute(true);
                Timer::after_millis(100).await;
                false
            }
            AudioSource::Spdif => {
                if let Some(gain) = POT_GAIN_SIGNAL.try_take() {
                    pot_gain = (gain, gain);
                }

                // Data should arrive once every millisecond.
                let result = spdif_audio_receiver
                    .receive()
                    .with_timeout(Duration::from_millis(5))
                    .await;

                match result {
                    Ok(samples) => {
                        let mut processed_samples = [0u32; 2 * SPDIF_SAMPLE_COUNT];

                        status_led.set_high();
                        let output_sample_count =
                            process(samples, &mut processed_samples, &mut filters, pot_gain.0, pot_gain.1);

                        // FIXME: Explanation? 16 bit playback in 32 bit DMA mode.
                        for sample in processed_samples.iter_mut() {
                            *sample >>= 16;
                        }
                        status_led.set_low();

                        let result = sai_amp.write(&processed_samples[..output_sample_count]).await;

                        // Notify the channel that the buffer is now ready to be reused
                        spdif_audio_receiver.receive_done();

                        result.is_err()
                    }
                    Err(_) => true,
                }
            }
            _ => false,
        };

        // Renew the SAI setup in case of errors.
        if renew {
            info!("Renew SAI");

            drop(sai_amp);
            drop(sai_rpi);

            SAI_ACTIVE_SIGNAL.signal(audio_source);
            AMP_SETUP_SIGNAL.wait().await;

            (sai_amp, sai_rpi) = new_sai_amp_rpi(
                &mut sai4_resources,
                sai_amp_write_buffer,
                sai_rpi_read_buffer,
                SAMPLE_RATE_HZ,
                audio_source,
            );

            sai_amp.start();
            sai_rpi.start();
        } else {
            // Check, if there is a signal stream arriving from the Raspberry Pi header.
            RPI_IS_STREAMING.store(!sai_rpi.is_muted().unwrap(), Relaxed);
        }
    }
}
