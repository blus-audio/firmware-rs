use crate::*;
use defmt::{info, panic};
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals;
use embassy_stm32::sai;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel;
use grounded::uninit::GroundedArrayCell;

// Factor of two for being able to buffer two full USB packets (excessive)
const SAI4A_BUFFER_SIZE: usize = 2 * USB_PACKET_SIZE;
const SAI4B_BUFFER_SIZE: usize = AUDIO_SIZE_PER_MS;

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
static mut SAI4A_WRITE_BUFFER: GroundedArrayCell<u32, SAI4A_BUFFER_SIZE> = GroundedArrayCell::uninit();
static mut SAI4B_READ_BUFFER: GroundedArrayCell<u32, SAI4B_BUFFER_SIZE> = GroundedArrayCell::uninit();

fn new_sai4<'d>(
    resources: &'d mut Sai4Resources,
    write_buffer: &'d mut [u32],
    read_buffer: &'d mut [u32],
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
        config.frame_length = (OUTPUT_CHANNEL_COUNT * SAMPLE_BIT_COUNT) as u8;
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
            write_buffer,
            config,
        )
    };

    let sai4_b_driver = {
        let mut config = sai::Config::default();

        config.tx_rx = sai::TxRx::Receiver;
        config.slot_count = sai::word::U4(INPUT_CHANNEL_COUNT as u8);
        config.slot_enable = 0xFFFF; // All slots
        config.data_size = sai::DataSize::Data32;
        // config.frame_sync_definition = sai::FrameSyncDefinition::StartOfFrame;
        config.frame_length = (OUTPUT_CHANNEL_COUNT * SAMPLE_BIT_COUNT) as u8;
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
            read_buffer,
            config,
        )
    };

    (sai4_a_driver, sai4_b_driver)
}

#[embassy_executor::task]
pub async fn audio_routing_task(
    filters: (AudioFilter, AudioFilter, AudioFilter, AudioFilter),
    mut sai4_resources: Sai4Resources,
    mut receiver: zerocopy_channel::Receiver<'static, NoopRawMutex, SampleBlock>,
    mut status_led: Output<'static>,
) {
    let sai4_a_write_buffer: &mut [u32] = unsafe {
        SAI4A_WRITE_BUFFER.initialize_all_copied(0);
        let (ptr, len) = SAI4A_WRITE_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let sai4_b_read_buffer: &mut [u32] = unsafe {
        SAI4B_READ_BUFFER.initialize_all_copied(0);
        let (ptr, len) = SAI4B_READ_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let (mut sai4_a_driver, mut _sai4_b_driver) = new_sai4(
        &mut sai4_resources,
        sai4_a_write_buffer,
        sai4_b_read_buffer,
        SAMPLE_RATE_HZ,
    );
    sai4_a_driver.start();
    // _sai4_b_driver.start();

    let (mut filter_a, mut filter_b, mut filter_c, mut filter_d) = filters;

    loop {
        let samples = receiver.receive().await;
        let mut samples_u32: Vec<u32, { 2 * SAMPLE_COUNT }> = Vec::new();

        status_led.set_high();

        for (index, sample) in samples.iter().enumerate() {
            if index % 2 == 0 {
                // Left channel
                samples_u32
                    .push(audio_filter::sample_to_u32(filter_a.run(*sample)))
                    .unwrap();
                samples_u32
                    .push(audio_filter::sample_to_u32(filter_b.run(*sample)))
                    .unwrap();
            } else {
                // Right channel
                samples_u32
                    .push(audio_filter::sample_to_u32(filter_c.run(*sample)))
                    .unwrap();
                samples_u32
                    .push(audio_filter::sample_to_u32(filter_d.run(*sample)))
                    .unwrap();
            }
        }

        status_led.set_low();

        let result = sai4_a_driver.write(&samples_u32).await;

        // Notify the channel that the buffer is now ready to be reused
        receiver.receive_done();

        if let Err(_) = result {
            SAI_ACTIVE_SIGNAL.signal(false);
            info!("Renew SAI setup.");

            drop(sai4_a_driver);
            drop(_sai4_b_driver);
            (sai4_a_driver, _sai4_b_driver) = new_sai4(
                &mut sai4_resources,
                sai4_a_write_buffer,
                sai4_b_read_buffer,
                SAMPLE_RATE_HZ,
            );
            sai4_a_driver.start();

            SAI_ACTIVE_SIGNAL.signal(true);
        }
    }
}
