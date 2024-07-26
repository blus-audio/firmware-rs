#![no_std]
#![no_main]

use blus_fw::uac1;
use core::cell::RefCell;
use defmt::{panic, *};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::{
    adc, bind_interrupts,
    gpio::{Input, Level, Output, Pull, Speed},
    i2c,
    mode::Async,
    peripherals,
    time::Hertz,
    usb,
};
use embassy_sync::{
    blocking_mutex::{
        raw::{NoopRawMutex, ThreadModeRawMutex},
        NoopMutex,
    },
    signal::Signal,
    zerocopy_channel::{Channel, Receiver, Sender},
};
use embassy_time::Timer;
use embassy_usb::driver::EndpointError;
use grounded::uninit::GroundedArrayCell;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    OTG_HS => usb::InterruptHandler<peripherals::USB_OTG_HS>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

const DMA_BUFFER_SIZE: usize = 2048;
const USB_PACKET_SIZE: usize = 96;
const SAMPLE_COUNT: usize = USB_PACKET_SIZE / 4 * 2;
const SAMPLE_BLOCK_COUNT: usize = 2;

static VOLUME_ADC_SIGNAL: Signal<ThreadModeRawMutex, u8> = Signal::new();

static I2C_BUS: StaticCell<NoopMutex<RefCell<i2c::I2c<'static, Async>>>> = StaticCell::new();

type SampleBlock = ([u32; SAMPLE_COUNT], usize);

// Accessible by most system masters (Zone D2)
#[link_section = ".sram1"]
static mut SAI1A_WRITE_BUFFER: GroundedArrayCell<u8, DMA_BUFFER_SIZE> = GroundedArrayCell::uninit();

// Accessible by BDMA (Zone D3)
#[link_section = ".sram4"]
static mut SAI4A_WRITE_BUFFER: GroundedArrayCell<u32, DMA_BUFFER_SIZE> = GroundedArrayCell::uninit();

struct AmplifierResources {
    i2c: i2c::I2c<'static, Async>,
    pin_nsd: Output<'static>,
    pin_irqz: Input<'static>,
}

struct AdcResources {
    adc: adc::Adc<'static, peripherals::ADC1>,
    pin: peripherals::PA6,
    dma: peripherals::DMA1_CH0,
}

struct Sai4Resources {
    sai: peripherals::SAI4,

    sck_a: peripherals::PD13,
    sd_a: peripherals::PC1,
    fs_a: peripherals::PD12,
    dma_a: peripherals::BDMA_CH0,

    sck_b: peripherals::PE12,
    sd_b: peripherals::PE11,
    fs_b: peripherals::PE13,
    dma_b: peripherals::BDMA_CH1,
}

#[embassy_executor::task]
async fn audio_routing_task(
    mut sai4_resources: Sai4Resources,
    mut receiver: Receiver<'static, NoopRawMutex, SampleBlock>,
    mut status_led: Output<'static>,
) {
    use embassy_stm32::sai;

    let sai4a_write_buffer: &mut [u32] = unsafe {
        SAI4A_WRITE_BUFFER.initialize_all_copied(0);
        let (ptr, len) = SAI4A_WRITE_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    fn new_sai4a<'d>(
        sai4_resources: &'d mut Sai4Resources,
        sai4a_write_buffer: &'d mut [u32],
    ) -> sai::Sai<'d, peripherals::SAI4, u32> {
        let mut sai4a_config = sai::Config::default();

        sai4a_config.slot_count = sai::word::U4(4);
        sai4a_config.slot_enable = 0xFFFF; // All slots
        sai4a_config.data_size = sai::DataSize::Data32;
        // sai4a_config.fifo_threshold = sai::FifoThreshold::Half;
        sai4a_config.frame_sync_offset = sai::FrameSyncOffset::OnFirstBit;
        sai4a_config.frame_sync_active_level_length = sai::word::U7(1);
        sai4a_config.frame_sync_definition = sai::FrameSyncDefinition::StartOfFrame;
        sai4a_config.frame_length = 4 * 32;
        sai4a_config.clock_strobe = sai::ClockStrobe::Rising;
        sai4a_config.bit_order = sai::BitOrder::MsbFirst;

        let _sai4b_config = sai::Config::default();

        let (sai4a, _sai4b) = sai::split_subblocks(&mut sai4_resources.sai);

        let sai4a_driver = sai::Sai::new_asynchronous(
            sai4a,
            &mut sai4_resources.sck_a,
            &mut sai4_resources.sd_a,
            &mut sai4_resources.fs_a,
            &mut sai4_resources.dma_a,
            sai4a_write_buffer,
            sai4a_config,
        );

        sai4a_driver
    }

    let mut sai4a_driver = new_sai4a(&mut sai4_resources, sai4a_write_buffer);
    sai4a_driver.start();

    loop {
        // Receive a buffer from the channel
        let (samples, sample_count) = receiver.receive().await;

        status_led.set_low();
        sai4a_driver.set_mute(false);

        if let Err(_) = sai4a_driver.write(&samples[..*sample_count]).await {
            drop(sai4a_driver);
            sai4a_driver = new_sai4a(&mut sai4_resources, sai4a_write_buffer);
            sai4a_driver.start();
            sai4a_driver.set_mute(true);
            status_led.set_high();
        }

        // Notify the channel that the buffer is now ready to be reused
        receiver.receive_done();
    }
}

#[embassy_executor::task]
async fn amplifier_task(amplifier_resources: AmplifierResources) {
    use blus_fw::tas2780::*;

    let mut pin_nsd = amplifier_resources.pin_nsd;

    info!("Reset amplifiers.");
    pin_nsd.set_low();
    Timer::after_millis(1).await;
    pin_nsd.set_high();

    // Wait for reset
    Timer::after_millis(10).await;

    let i2c_bus = NoopMutex::new(RefCell::new(amplifier_resources.i2c));
    let i2c_bus = I2C_BUS.init(i2c_bus);

    const NOISE_GATE: Option<NoiseGate> = Some(NoiseGate {
        hysteresis: NoiseGateHysteresis::Duration1000ms,
        level: NoiseGateLevel::ThresholdMinus90dBFS,
    });

    const POWER_MODE: PowerMode = PowerMode::Two;
    const AMPLIFIER_LEVEL: Gain = Gain::Gain11_0dBV;

    let mut ic2_device_a = I2cDevice::new(i2c_bus);
    let tas2780_a = Tas2780::new(
        &mut ic2_device_a,
        0x39,
        0,
        Channel::Left,
        AMPLIFIER_LEVEL,
        POWER_MODE,
        NOISE_GATE,
    );

    let mut ic2_device_b = I2cDevice::new(i2c_bus);
    let tas2780_b = Tas2780::new(
        &mut ic2_device_b,
        0x3a,
        1,
        Channel::Left,
        AMPLIFIER_LEVEL,
        POWER_MODE,
        NOISE_GATE,
    );

    let mut ic2_device_c = I2cDevice::new(i2c_bus);
    let tas2780_c = Tas2780::new(
        &mut ic2_device_c,
        0x3d,
        2,
        Channel::Left,
        AMPLIFIER_LEVEL,
        POWER_MODE,
        NOISE_GATE,
    );

    let mut ic2_device_d = I2cDevice::new(i2c_bus);
    let tas2780_d = Tas2780::new(
        &mut ic2_device_d,
        0x3e,
        3,
        Channel::Left,
        AMPLIFIER_LEVEL,
        POWER_MODE,
        NOISE_GATE,
    );

    let mut all_amplifiers = [tas2780_a, tas2780_b, tas2780_c, tas2780_d];

    info!("Initialize TAS2780");
    for amplifier in &mut all_amplifiers {
        amplifier.init().await;
    }

    loop {
        let attenuation_half_db = 200u8 - 2 * (VOLUME_ADC_SIGNAL.wait().await);

        // info!("Attenuation changed to {}", attenuation_half_db);
        for amplifier in &mut all_amplifiers {
            amplifier.set_volume(attenuation_half_db);
            amplifier.enable();
        }
    }
}

#[embassy_executor::task]
async fn volume_control_task(mut adc_resources: AdcResources) {
    let mut adc = adc_resources.adc;
    adc.set_sample_time(adc::SampleTime::CYCLES810_5);

    let mut last_measured = 0u8;

    loop {
        let measured = (100f32 * (adc.blocking_read(&mut adc_resources.pin) as f32) / 65535f32) as u8;

        if last_measured != measured {
            VOLUME_ADC_SIGNAL.signal(measured);
            last_measured = measured;
        }

        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn heartbeat_task(mut status_led: Output<'static>) {
    loop {
        status_led.set_high();
        Timer::after_millis(1000).await;

        status_led.set_low();
        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut peripheral_config = embassy_stm32::Config::default();
    {
        // Uses a 24.576 MHz external oscillator.
        use embassy_stm32::rcc::*;
        peripheral_config.rcc.hse = Some(Hse {
            freq: Hertz(24_576_000),
            mode: HseMode::Bypass,
        });
        peripheral_config.rcc.hsi = Some(HSIPrescaler::DIV1);
        peripheral_config.rcc.csi = true;
        peripheral_config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: false });
        peripheral_config.rcc.pll1 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL80,
            divp: Some(PllDiv::DIV1),  // 491.52 MHz
            divq: Some(PllDiv::DIV80), // 12.288 MHz
            divr: Some(PllDiv::DIV2),  // 245.76 MHz
        });
        peripheral_config.rcc.pll3 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV8,
            mul: PllMul::MUL125,
            divp: Some(PllDiv::DIV2), // 192 MHz
            divq: Some(PllDiv::DIV8), // 48 MHz for USB
            divr: Some(PllDiv::DIV2), // 192 MHz
        });
        peripheral_config.rcc.sys = Sysclk::PLL1_P; // 491.52 Mhz
        peripheral_config.rcc.ahb_pre = AHBPrescaler::DIV2;
        peripheral_config.rcc.apb1_pre = APBPrescaler::DIV2;
        peripheral_config.rcc.apb2_pre = APBPrescaler::DIV2;
        peripheral_config.rcc.apb3_pre = APBPrescaler::DIV2;
        peripheral_config.rcc.apb4_pre = APBPrescaler::DIV2;
        peripheral_config.rcc.voltage_scale = VoltageScale::Scale0;
        peripheral_config.rcc.mux.usbsel = mux::Usbsel::PLL3_Q;
        peripheral_config.rcc.mux.sai1sel = mux::Saisel::PLL1_Q;
        peripheral_config.rcc.mux.adcsel = mux::Adcsel::PLL3_R;
    }
    let p = embassy_stm32::init(peripheral_config);

    let mut led_blue = Output::new(p.PC6, Level::High, Speed::Low);
    let mut led_green = Output::new(p.PC7, Level::High, Speed::Low);
    let mut led_yellow = Output::new(p.PC8, Level::High, Speed::Low);
    let mut led_red = Output::new(p.PC9, Level::High, Speed::Low);

    for led in [&mut led_blue, &mut led_green, &mut led_yellow, &mut led_red] {
        led.set_low();
    }

    let mut ep_out_buffer = [0u8; 1024];
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut state = uac1::State::new();

    // Create the driver, from the HAL.
    let mut usb_config = usb::Config::default();

    // Do not enable vbus_detection with an external HS PHY.
    usb_config.vbus_detection = false;
    // Using a Microchip PHY requires a delay during setup.
    usb_config.xcvrdly = true;

    // Initialize driver for high-speed external PHY.
    let usb_driver = usb::Driver::new_hs_ulpi(
        p.USB_OTG_HS,
        Irqs,
        p.PA5,
        p.PC2,
        p.PC3,
        p.PC0,
        p.PA3,
        p.PB0,
        p.PB1,
        p.PB10,
        p.PB11,
        p.PB12,
        p.PB13,
        p.PB5,
        &mut ep_out_buffer,
        usb_config,
    );

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0x1209, 0xaf02);
    config.manufacturer = Some("elagil");
    config.product = Some("blus-mini mk2");
    config.serial_number = Some("123");
    config.self_powered = true;
    config.max_power = 0;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut builder = embassy_usb::Builder::new(
        usb_driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = uac1::AudioClassOne::new(&mut builder, &mut state, USB_PACKET_SIZE as u16);

    // Build the builder.
    let mut usb_device = builder.build();
    let usb_fut = usb_device.run();

    let sai4_resources = Sai4Resources {
        sai: p.SAI4,
        sck_a: p.PD13,
        sd_a: p.PC1,
        fs_a: p.PD12,
        dma_a: p.BDMA_CH0,
        sck_b: p.PE12,
        sd_b: p.PE11,
        fs_b: p.PE13,
        dma_b: p.BDMA_CH1,
    };

    let amplifier_resources = AmplifierResources {
        i2c: i2c::I2c::new(
            p.I2C1,
            p.PB6,
            p.PB7,
            Irqs,
            p.DMA1_CH4,
            p.DMA1_CH5,
            Hertz(1_000_000),
            Default::default(),
        ),
        pin_nsd: Output::new(p.PC12, Level::High, Speed::Low),
        pin_irqz: Input::new(p.PC13, Pull::None),
    };

    let adc_resources = AdcResources {
        adc: adc::Adc::new(p.ADC1),
        pin: p.PA6,
        dma: p.DMA1_CH0,
    };

    static SAMPLE_BLOCKS: StaticCell<[SampleBlock; SAMPLE_BLOCK_COUNT]> = StaticCell::new();
    let sample_blocks = SAMPLE_BLOCKS.init([([0; SAMPLE_COUNT], 0); SAMPLE_BLOCK_COUNT]);

    static CHANNEL: StaticCell<Channel<'_, NoopRawMutex, SampleBlock>> = StaticCell::new();
    let channel = CHANNEL.init(Channel::new(sample_blocks));
    let (mut sender, receiver) = channel.split();

    // Create PR
    // let mut tim_2 = timer::low_level::Timer::new(p.TIM2);
    // tim_2.set_trigger_source(timer::low_level::TriggerSource::ITR5);

    unwrap!(spawner.spawn(heartbeat_task(led_red)));
    unwrap!(spawner.spawn(audio_routing_task(sai4_resources, receiver, led_blue)));
    unwrap!(spawner.spawn(volume_control_task(adc_resources)));
    unwrap!(spawner.spawn(amplifier_task(amplifier_resources)));

    let receive_fut = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = receive(&mut class, &mut sender).await;
            info!("Disconnected");
        }
    };

    // Run everything concurrently.
    join(usb_fut, receive_fut).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn receive<'d, T: usb::Instance + 'd>(
    class: &mut uac1::AudioClassOne<'d, usb::Driver<'d, T>>,
    sender: &mut Sender<'static, NoopRawMutex, SampleBlock>,
) -> Result<(), Disconnected> {
    loop {
        // Obtain a free buffer from the channel
        let (samples, sample_count) = sender.send().await;

        let mut usb_data = [0u8; USB_PACKET_SIZE];

        let data_size = class.read_packet(&mut usb_data).await?;
        let word_count = data_size / 4;

        for w in 0..word_count {
            let sample: u32 = (usb_data[4 * w] as u32)
                | (usb_data[4 * w + 1] as u32) << 8
                | (usb_data[4 * w + 2] as u32) << 16
                | (usb_data[4 * w + 3] as u32) << 24;

            // Fill the sample buffer with data.
            samples[2 * w] = sample;
            samples[2 * w + 1] = sample;
        }

        *sample_count = 2 * word_count;

        sender.send_done();
    }
}
