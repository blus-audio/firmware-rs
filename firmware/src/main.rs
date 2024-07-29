#![no_std]
#![no_main]

use biquad::ToHertz;
use blus_fw::uac1::{self, ChannelConfig};
use blus_fw::{filters, tas2780};
use core::cell::RefCell;
use defmt::{info, panic, unwrap};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_futures::{
    join::join,
    select::{select, Either},
};
use embassy_stm32::{
    adc::{self, AdcChannel},
    bind_interrupts,
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
use embassy_time::{Duration, Ticker, Timer};
use embassy_usb::driver::EndpointError;
use grounded::uninit::GroundedArrayCell;
use heapless::Vec;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    OTG_HS => usb::InterruptHandler<peripherals::USB_OTG_HS>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

const Q31_SCALING_FACTOR: f32 = 2147483648.0;
const DMA_BUFFER_SIZE: usize = 1024;
const USB_PACKET_SIZE: usize = 96;

const SAMPLE_RATE_HZ: u32 = 48000;
const SAMPLE_SIZE: usize = 4;
const SAMPLE_COUNT: usize = USB_PACKET_SIZE / SAMPLE_SIZE;
const SAMPLE_BLOCK_COUNT: usize = 2;

static PLAY_SIGNAL: Signal<ThreadModeRawMutex, bool> = Signal::new();
static VOLUME_ADC_SIGNAL: Signal<ThreadModeRawMutex, (u8, u8)> = Signal::new();

static I2C_BUS: StaticCell<NoopMutex<RefCell<i2c::I2c<'static, Async>>>> = StaticCell::new();
static FILTERS: StaticCell<filters::Filters> = StaticCell::new();

type SampleBlock = ([f32; SAMPLE_COUNT], usize);

// Accessible by most system masters (Zone D2)
#[link_section = ".sram1"]
static mut ADC1_MEASUREMENT_BUFFER: GroundedArrayCell<u16, 1> = GroundedArrayCell::uninit();
static mut _SAI1A_WRITE_BUFFER: GroundedArrayCell<u8, DMA_BUFFER_SIZE> = GroundedArrayCell::uninit();

// Accessible by BDMA (Zone D3)
#[link_section = ".sram4"]
static mut SAI4A_WRITE_BUFFER: GroundedArrayCell<u32, DMA_BUFFER_SIZE> = GroundedArrayCell::uninit();

#[allow(unused)]
struct AmplifierResources {
    i2c: i2c::I2c<'static, Async>,
    pin_nsd: Output<'static>,
    pin_irqz: Input<'static>,
}

#[allow(unused)]
struct AdcResources<T: adc::Instance> {
    adc: adc::Adc<'static, T>,
    pin: adc::AnyAdcChannel<T>,
    dma: peripherals::DMA1_CH0,
}

#[allow(unused)]
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

fn clip_q63_to_q31(sample: i64) -> i32 {
    if (sample >> 32) as i32 != (sample as i32) >> 31 {
        info!("Clipped audio!");
        0x7FFFFFFF ^ ((sample >> 63) as i32)
    } else {
        sample as i32
    }
}

fn sample_as_u32(sample: f32) -> u32 {
    clip_q63_to_q31((sample * Q31_SCALING_FACTOR) as i64) as u32
}

fn sample_as_f32(sample: u32) -> f32 {
    (sample as i32 as f32) / Q31_SCALING_FACTOR
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
        sample_rate_hz: u32,
    ) -> sai::Sai<'d, peripherals::SAI4, u32> {
        let mut sai4a_config = sai::Config::default();

        sai4a_config.slot_count = sai::word::U4(4);
        sai4a_config.slot_enable = 0xFFFF; // All slots
        sai4a_config.data_size = sai::DataSize::Data32;
        sai4a_config.frame_sync_offset = sai::FrameSyncOffset::OnFirstBit;
        sai4a_config.frame_sync_active_level_length = sai::word::U7(1);
        sai4a_config.frame_sync_definition = sai::FrameSyncDefinition::StartOfFrame;
        sai4a_config.frame_length = 4 * 32;
        sai4a_config.clock_strobe = sai::ClockStrobe::Rising;
        sai4a_config.bit_order = sai::BitOrder::MsbFirst;

        match sample_rate_hz {
            SAMPLE_RATE_HZ => sai4a_config.master_clock_divider = sai::MasterClockDivider::Div2,
            _ => panic!("Unsupported SAI sample rate."),
        }

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

    let mut sai4a_driver = new_sai4a(&mut sai4_resources, sai4a_write_buffer, SAMPLE_RATE_HZ);
    sai4a_driver.start();

    let filters = FILTERS.init(filters::Filters::new(SAMPLE_RATE_HZ.hz()));

    status_led.set_low();
    loop {
        let (samples, sample_count) = receiver.receive().await;

        let mut samples_u32: Vec<u32, { 2 * SAMPLE_COUNT }> = Vec::new();

        status_led.set_high();

        for index in 0..*sample_count / 2 {
            let sample_left = samples[2 * index];
            let sample_right = samples[2 * index + 1];

            let mut sample_a = sample_left;
            let mut sample_b = sample_left;
            let mut sample_c = sample_right;
            let mut sample_d = sample_right;

            sample_a = filters::run_filters(sample_a, &mut filters.biquads_a, filters.gain_a);
            sample_b = filters::run_filters(sample_b, &mut filters.biquads_b, filters.gain_b);
            sample_c = filters::run_filters(sample_c, &mut filters.biquads_c, filters.gain_c);
            sample_d = filters::run_filters(sample_d, &mut filters.biquads_d, filters.gain_d);

            samples_u32.push(sample_as_u32(sample_a)).unwrap();
            samples_u32.push(sample_as_u32(sample_b)).unwrap();
            samples_u32.push(sample_as_u32(sample_c)).unwrap();
            samples_u32.push(sample_as_u32(sample_d)).unwrap();
        }

        status_led.set_low();

        let result = sai4a_driver.write(&samples_u32[..*sample_count * 2]).await;

        // Notify the channel that the buffer is now ready to be reused
        receiver.receive_done();

        if let Err(_) = result {
            PLAY_SIGNAL.signal(false);
            info!("Renew SAI setup.");

            drop(sai4a_driver);
            sai4a_driver = new_sai4a(&mut sai4_resources, sai4a_write_buffer, SAMPLE_RATE_HZ);
            sai4a_driver.start();

            PLAY_SIGNAL.signal(true);
        }
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
    let mut tas2780_a = Tas2780::new(
        &mut ic2_device_a,
        0x39,
        0,
        Channel::Left,
        AMPLIFIER_LEVEL,
        POWER_MODE,
        NOISE_GATE,
    );

    let mut ic2_device_b = I2cDevice::new(i2c_bus);
    let mut tas2780_b = Tas2780::new(
        &mut ic2_device_b,
        0x3a,
        1,
        Channel::Left,
        AMPLIFIER_LEVEL,
        POWER_MODE,
        NOISE_GATE,
    );

    let mut ic2_device_c = I2cDevice::new(i2c_bus);
    let mut tas2780_c = Tas2780::new(
        &mut ic2_device_c,
        0x3d,
        2,
        Channel::Right,
        AMPLIFIER_LEVEL,
        POWER_MODE,
        NOISE_GATE,
    );

    let mut ic2_device_d = I2cDevice::new(i2c_bus);
    let mut tas2780_d = Tas2780::new(
        &mut ic2_device_d,
        0x3e,
        3,
        Channel::Right,
        AMPLIFIER_LEVEL,
        POWER_MODE,
        NOISE_GATE,
    );

    info!("Initialize TAS2780");
    for amplifier in [&mut tas2780_a, &mut tas2780_b, &mut tas2780_c, &mut tas2780_d] {
        amplifier.init().await;
    }

    loop {
        let play_fut = PLAY_SIGNAL.wait();
        let volume_fut = VOLUME_ADC_SIGNAL.wait();

        match select(play_fut, volume_fut).await {
            Either::First(play) => {
                if play {
                    for amplifier in [&mut tas2780_a, &mut tas2780_b, &mut tas2780_c, &mut tas2780_d] {
                        amplifier.enable();
                    }
                } else {
                    // FIXME: Implement disable?
                }
            }
            Either::Second((volume_left, volume_right)) => {
                for amplifier in [&mut tas2780_a, &mut tas2780_b] {
                    amplifier.set_volume(volume_left);
                }

                for amplifier in [&mut tas2780_c, &mut tas2780_d] {
                    amplifier.set_volume(volume_right);
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn volume_control_task(
    mut adc_resources: AdcResources<peripherals::ADC1>,
    control_changed: uac1::ControlChanged<'static>,
    mut status_led: Output<'static>,
) {
    let mut pot_ticker = Ticker::every(Duration::from_hz(1));

    let adc1_measurement_buffer: &mut [u16] = unsafe {
        ADC1_MEASUREMENT_BUFFER.initialize_all_copied(0);
        let (ptr, len) = ADC1_MEASUREMENT_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    // Measure the potentiometer in intervals that are given by the ticker.
    async fn measure(
        ticker: &mut Ticker,
        adc_resources: &mut AdcResources<peripherals::ADC1>,
        measurement_buffer: &mut [u16],
    ) -> u8 {
        ticker.next().await;
        adc_resources
            .adc
            .read(
                &mut adc_resources.dma,
                [(&mut adc_resources.pin, adc::SampleTime::CYCLES810_5)].into_iter(),
                measurement_buffer,
            )
            .await;

        let measured = (100f32 * (measurement_buffer[0] as f32) / 65535f32) as u8;

        measured
    }

    // Converts an attenuation level in 8.8 binary signed fixpoint format to the TAS2780 representation.
    // The TAS2780 understands volume levels from 0 (0 dB) to 0xC8 (-100 dB), where one step is 0.5 dB.
    let attenuation_8q8_db_to_tas2780 = |x: i16| (-x >> 7) as u8;

    loop {
        let control_changed_fut = control_changed.control_changed();
        let pot_measured_fut = measure(&mut pot_ticker, &mut adc_resources, adc1_measurement_buffer);

        match select(pot_measured_fut, control_changed_fut).await {
            Either::First(_measured) => (),
            Either::Second(_) => {
                status_led.set_high();
                let audio_channel_settings = control_changed.audio_settings();

                let volume_left: u8;
                if audio_channel_settings.is_muted[0] {
                    volume_left = tas2780::MUTE_VOLUME;
                } else {
                    volume_left = attenuation_8q8_db_to_tas2780(audio_channel_settings.volume_8q8_db[0]);
                }

                let volume_right: u8;
                if audio_channel_settings.is_muted[0] {
                    volume_right = tas2780::MUTE_VOLUME;
                } else {
                    volume_right = attenuation_8q8_db_to_tas2780(audio_channel_settings.volume_8q8_db[0]);
                }

                VOLUME_ADC_SIGNAL.signal((volume_left, volume_right));

                // Limit the rate of volume updates
                Timer::after_millis(100).await;
                status_led.set_low();
            }
        }
    }
}

#[embassy_executor::task]
async fn heartbeat_task(mut status_led: Output<'static>) {
    loop {
        status_led.set_high();
        Timer::after_millis(100).await;

        status_led.set_low();
        Timer::after_secs(2).await;
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
            divq: Some(PllDiv::DIV20), // 24.576 MHz for SAI4
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

    static EP_OUT_BUFFER: StaticCell<[u8; 2 * USB_PACKET_SIZE]> = StaticCell::new();
    let ep_out_buffer = EP_OUT_BUFFER.init([0u8; 2 * USB_PACKET_SIZE]);

    static CONFIG_DESCRIPTOR: StaticCell<[u8; 128]> = StaticCell::new();
    let config_descriptor = CONFIG_DESCRIPTOR.init([0; 128]);

    static BOS_DESCRIPTOR: StaticCell<[u8; 16]> = StaticCell::new();
    let bos_descriptor = BOS_DESCRIPTOR.init([0; 16]);

    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let control_buf = CONTROL_BUF.init([0; 64]);

    static STATE: StaticCell<uac1::State> = StaticCell::new();
    let state = STATE.init(uac1::State::new());

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
        ep_out_buffer,
        usb_config,
    );

    // Basic USB device configuration
    let mut config = embassy_usb::Config::new(0x1209, 0xaf02);
    config.manufacturer = Some("elagil");
    config.product = Some("blus-mini mk2");
    config.self_powered = true;
    config.max_power = 0;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    let mut builder = embassy_usb::Builder::new(
        usb_driver,
        config,
        config_descriptor,
        bos_descriptor,
        &mut [], // no msos descriptors
        control_buf,
    );

    // Create classes on the builder
    let class = uac1::Uac1::new(
        &mut builder,
        state,
        USB_PACKET_SIZE as u16,
        &[SAMPLE_RATE_HZ],
        &[ChannelConfig::LeftFront, ChannelConfig::RightFront],
        uac1::FeedbackRefreshPeriod::Period8ms,
    );
    let (mut stream, control_changed) = class.split();

    // Build and run the USB device
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
        pin: p.PA6.degrade_adc(),
        dma: p.DMA1_CH0,
    };

    static SAMPLE_BLOCKS: StaticCell<[SampleBlock; SAMPLE_BLOCK_COUNT]> = StaticCell::new();
    let sample_blocks = SAMPLE_BLOCKS.init([([0.0; SAMPLE_COUNT], 0); SAMPLE_BLOCK_COUNT]);

    static CHANNEL: StaticCell<Channel<'_, NoopRawMutex, SampleBlock>> = StaticCell::new();
    let channel = CHANNEL.init(Channel::new(sample_blocks));
    let (mut sender, receiver) = channel.split();

    // let mut tim_2 = timer::low_level::Timer::new(p.TIM2);
    // tim_2.set_trigger_source(timer::low_level::TriggerSource::ITR5);

    unwrap!(spawner.spawn(heartbeat_task(led_red)));
    unwrap!(spawner.spawn(audio_routing_task(sai4_resources, receiver, led_blue)));
    unwrap!(spawner.spawn(volume_control_task(adc_resources, control_changed, led_yellow)));
    unwrap!(spawner.spawn(amplifier_task(amplifier_resources)));

    let receive_fut = async {
        loop {
            stream.wait_connection().await;
            info!("Connected");
            let _ = receive(&mut stream, &mut sender).await;
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
    stream: &mut uac1::Stream<'d, usb::Driver<'d, T>>,
    sender: &mut Sender<'static, NoopRawMutex, SampleBlock>,
) -> Result<(), Disconnected> {
    loop {
        let mut usb_data = [0u8; USB_PACKET_SIZE];

        // let feedback_value: u32 = 393216; // Ideal for 48 kHz, in 16uq16 kHz format?
        // stream
        //     .write_packet(&[
        //         feedback_value as u8,
        //         (feedback_value >> 8) as u8,
        //         (feedback_value >> 16) as u8,
        //         (feedback_value >> 24) as u8,
        //     ])
        //     .await?;
        let data_size = stream.read_packet(&mut usb_data).await?;
        let word_count = data_size / SAMPLE_SIZE;

        if word_count * SAMPLE_SIZE == data_size {
            // Obtain a free buffer from the channel
            let (samples, sample_count) = sender.send().await;

            for w in 0..word_count {
                let byte_offset = w * SAMPLE_SIZE;
                let sample = sample_as_f32(u32::from_le_bytes(
                    usb_data[byte_offset..byte_offset + SAMPLE_SIZE].try_into().unwrap(),
                ));

                // Fill the sample buffer with data.
                samples[w] = sample;
            }

            *sample_count = word_count;

            sender.send_done();
        } else {
            info!("Invalid USB buffer size of {}, skipped.", data_size);
        }
    }
}
