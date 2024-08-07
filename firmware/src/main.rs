#![no_std]
#![no_main]

use core::cell::RefCell;
use core::panic;

use blus_fw::*;
use defmt::{info, unwrap};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_futures::select::{select3, Either3};
use embassy_stm32::adc::{self, AdcChannel};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, i2c, interrupt, peripherals, timer, usb};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::blocking_mutex::{Mutex, NoopMutex};
use embassy_sync::signal::Signal;
use embassy_sync::zerocopy_channel;
use embassy_time::{Duration, Ticker, Timer};
use embassy_usb::class::uac1;
use embassy_usb::class::uac1::speaker::{self, Speaker};
use grounded::uninit::GroundedArrayCell;
use heapless::Vec;
use libm::{fabsf, roundf};
use micromath::F32Ext;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    OTG_HS => usb::InterruptHandler<peripherals::USB_OTG_HS>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

static POT_GAIN_SIGNAL: Signal<ThreadModeRawMutex, f32> = Signal::new();
static TIMER: Mutex<CriticalSectionRawMutex, RefCell<Option<timer::low_level::Timer<peripherals::TIM2>>>> =
    Mutex::new(RefCell::new(None));
static I2C_BUS: StaticCell<NoopMutex<RefCell<i2c::I2c<'static, Async>>>> = StaticCell::new();

// Accessible by most system masters (Zone D2)
#[link_section = ".sram1"]
static mut ADC1_MEASUREMENT_BUFFER: GroundedArrayCell<u16, 1> = GroundedArrayCell::uninit();

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

pub fn db_to_linear(db: f32) -> f32 {
    10.0f32.powf(db / 20.0)
}

pub fn get_filters() -> [AudioFilter<'static>; OUTPUT_CHANNEL_COUNT] {
    use biquad::*;

    type B = BiquadType;
    type C = Coefficients<f32>;

    // Crossover frequency
    let f_co = 1800.hz();

    let fs = SAMPLE_RATE_HZ.hz();

    static BIQUADS_A: StaticCell<[B; 9]> = StaticCell::new();
    static BIQUADS_C: StaticCell<[B; 9]> = StaticCell::new();
    static BIQUADS_B: StaticCell<[B; 6]> = StaticCell::new();
    static BIQUADS_D: StaticCell<[B; 6]> = StaticCell::new();

    let biquads_a = BIQUADS_A.init([
        B::new(C::from_params(Type::AllPass, fs, f_co, 0.6).unwrap()),
        B::new(C {
            a1: -1.9925941047116,
            a2: 0.992621419175639,
            b0: 1.00200843380849,
            b1: -1.99256829254308,
            b2: 0.990638797535668,
        }),
        B::new(C::from_params(Type::PeakingEQ(-2.5), fs, 660.hz(), 2.5).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(1.0), fs, 880.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::HighShelf(-8.0), fs, 1200.hz(), 0.35).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(-2.5), fs, 1300.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(-3.0), fs, 3450.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::LowPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C::from_params(Type::LowPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
    ]);

    let biquads_c = BIQUADS_C.init([
        B::new(C::from_params(Type::AllPass, fs, f_co, 0.6).unwrap()),
        B::new(C {
            a1: -1.9925941047116,
            a2: 0.992621419175639,
            b0: 1.00200843380849,
            b1: -1.99256829254308,
            b2: 0.990638797535668,
        }),
        B::new(C::from_params(Type::PeakingEQ(-2.5), fs, 660.hz(), 2.5).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(1.0), fs, 880.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::HighShelf(-8.0), fs, 1200.hz(), 0.35).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(-2.5), fs, 1300.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(-3.0), fs, 3450.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::LowPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C::from_params(Type::LowPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
    ]);

    let biquads_b = BIQUADS_B.init([
        B::new(C::from_params(Type::PeakingEQ(-9.0), fs, 1700.hz(), 0.3).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(1.0), fs, 7700.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(-1.0), fs, 12000.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(6.0), fs, 18000.hz(), 0.6).unwrap()),
        B::new(C::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
    ]);

    let biquads_d = BIQUADS_D.init([
        B::new(C::from_params(Type::PeakingEQ(-9.0), fs, 1700.hz(), 0.3).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(1.0), fs, 7700.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(-1.0), fs, 12000.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(6.0), fs, 18000.hz(), 0.6).unwrap()),
        B::new(C::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
    ]);

    // Negative gain inverts a channel.
    let gain_a = -db_to_linear(-10.0);
    let gain_b = db_to_linear(-11.5);
    let gain_c = -db_to_linear(-10.0);
    let gain_d = db_to_linear(-11.5);

    let delay_a: usize = 0;
    let delay_b: usize = 6;
    let delay_c: usize = 0;
    let delay_d: usize = 6;

    let f_a = AudioFilter::new(gain_a, delay_a, biquads_a);
    let f_b = AudioFilter::new(gain_b, delay_b, biquads_b);
    let f_c = AudioFilter::new(gain_c, delay_c, biquads_c);
    let f_d = AudioFilter::new(gain_d, delay_d, biquads_d);

    [f_a, f_b, f_c, f_d]
}

#[embassy_executor::task]
async fn volume_task(mut source_subscriber: SourceSubscriber) {
    // Linear gains for different sources.
    let mut pot_gain = 0.0_f32;
    let mut usb_gain_left = 0.0_f32;
    let mut usb_gain_right = 0.0_f32;
    let mut audio_source = AudioSource::None;

    loop {
        let usb_volume_fut = USB_VOLUME_SIGNAL.wait();
        let pot_gain_fut = POT_GAIN_SIGNAL.wait();
        let source_fut = source_subscriber.next_message_pure();

        match select3(usb_volume_fut, pot_gain_fut, source_fut).await {
            Either3::First(volumes) => {
                for volume in volumes {
                    let (gain, channel) = match volume {
                        speaker::Volume::Muted(channel) => (0.0, channel),
                        speaker::Volume::DeciBel(channel, volume_db) => {
                            if volume_db > 0.0 {
                                panic!("Volume must not be positive.")
                            }

                            (db_to_linear(volume_db), channel)
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
            }
            Either3::Second(gain) => {
                pot_gain = gain;
            }
            Either3::Third(_audio_source) => audio_source = _audio_source,
        };

        let (gain_left, gain_right) = match audio_source {
            AudioSource::Usb => (usb_gain_left, usb_gain_right),
            AudioSource::Spdif => (pot_gain, pot_gain),
            _ => (1.0, 1.0),
        };

        GAIN_SIGNAL.signal((gain_left, gain_right));
    }
}

#[embassy_executor::task]
async fn amplifier_task(amplifier_resources: AmplifierResources) {
    use blus_fw::tas2780::*;

    let mut pin_nsd = amplifier_resources.pin_nsd;

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

    info!("Reset amplifiers.");
    pin_nsd.set_low();
    Timer::after_millis(10).await;
    pin_nsd.set_high();

    // Wait for reset
    Timer::after_millis(10).await;

    info!("Initialize TAS2780");
    for amplifier in [&mut tas2780_a, &mut tas2780_b, &mut tas2780_c, &mut tas2780_d] {
        amplifier.init().await;
    }

    loop {
        let play = SAI_ACTIVE_SIGNAL.wait().await;

        if play {
            for amplifier in [&mut tas2780_a, &mut tas2780_b, &mut tas2780_c, &mut tas2780_d] {
                amplifier.enable();
            }
        } else {
            // FIXME: Implement disable?
        }
    }
}

#[embassy_executor::task]
async fn heartbeat_task(mut status_led: Output<'static>) {
    let mut ticker = Ticker::every(Duration::from_hz(1));

    loop {
        ticker.next().await;
        status_led.toggle();
    }
}

#[embassy_executor::task]
async fn potentiometer_task(mut adc_resources: AdcResources<peripherals::ADC1>) {
    let mut ticker = Ticker::every(Duration::from_hz(10));

    let buffer: &mut [u16] = unsafe {
        ADC1_MEASUREMENT_BUFFER.initialize_all_copied(0);
        let (ptr, len) = ADC1_MEASUREMENT_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let mut gain = 0.0;

    loop {
        ticker.next().await;

        adc_resources
            .adc
            .read(
                &mut adc_resources.dma,
                [(&mut adc_resources.pin, adc::SampleTime::CYCLES810_5)].into_iter(),
                buffer,
            )
            .await;

        // Ranges from 0.0 to 1.0.
        let measured = (buffer[0] as f32) / 65535f32;

        // Round to steps of 0.05.
        let rounded = roundf(20.0 * measured) / 20.0;

        if fabsf(gain - measured) >= 0.04 {
            gain = rounded;

            // Make gain exponential
            let exp_gain = gain.powf(2.0);
            POT_GAIN_SIGNAL.signal(exp_gain);
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hi.");

    let mut peripheral_config = embassy_stm32::Config::default();
    {
        // Uses a 24.576 MHz external oscillator.
        use embassy_stm32::rcc::*;
        peripheral_config.rcc.hse = Some(Hse {
            freq: Hertz(24_576_000),
            mode: HseMode::Bypass,
        });
        peripheral_config.rcc.hsi = None;
        peripheral_config.rcc.csi = true;
        peripheral_config.rcc.hsi48 = None;
        peripheral_config.rcc.pll1 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL80,
            divp: Some(PllDiv::DIV2),  // 245.76 MHz
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
        peripheral_config.rcc.sys = Sysclk::PLL1_P;
        peripheral_config.rcc.ahb_pre = AHBPrescaler::DIV2;
        peripheral_config.rcc.apb1_pre = APBPrescaler::DIV2;
        peripheral_config.rcc.apb2_pre = APBPrescaler::DIV2;
        peripheral_config.rcc.apb3_pre = APBPrescaler::DIV2;
        peripheral_config.rcc.apb4_pre = APBPrescaler::DIV2;

        // Voltage scaling
        // 0 (<= 520 MHz)
        // 1 (<= 400 MHz)
        // 2 (<= 300 MHz)
        // 3 (<= 170 MHz)
        peripheral_config.rcc.voltage_scale = VoltageScale::Scale2;
        peripheral_config.rcc.mux.usbsel = mux::Usbsel::PLL3_Q;
        peripheral_config.rcc.mux.sai1sel = mux::Saisel::PLL1_Q;
        peripheral_config.rcc.mux.adcsel = mux::Adcsel::PLL3_R;
    }
    let p = embassy_stm32::init(peripheral_config);

    let mut led_blue = Output::new(p.PC6, Level::Low, Speed::Low);
    let mut led_green = Output::new(p.PC7, Level::Low, Speed::Low);
    let mut led_yellow = Output::new(p.PC8, Level::Low, Speed::Low);
    let mut led_red = Output::new(p.PC9, Level::Low, Speed::Low);

    for led in [&mut led_blue, &mut led_green, &mut led_yellow, &mut led_red] {
        led.set_low();
    }

    info!("USB packet size is {} byte", USB_PACKET_SIZE);
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 128]> = StaticCell::new();
    let config_descriptor = CONFIG_DESCRIPTOR.init([0; 128]);

    static BOS_DESCRIPTOR: StaticCell<[u8; 16]> = StaticCell::new();
    let bos_descriptor = BOS_DESCRIPTOR.init([0; 16]);

    const CONTROL_BUF_SIZE: usize = 64;
    static CONTROL_BUF: StaticCell<[u8; CONTROL_BUF_SIZE]> = StaticCell::new();
    let control_buf = CONTROL_BUF.init([0; CONTROL_BUF_SIZE]);

    const FEEDBACK_BUF_SIZE: usize = 4;
    static EP_OUT_BUFFER: StaticCell<[u8; FEEDBACK_BUF_SIZE + CONTROL_BUF_SIZE + USB_PACKET_SIZE]> = StaticCell::new();
    let ep_out_buffer = EP_OUT_BUFFER.init([0u8; FEEDBACK_BUF_SIZE + CONTROL_BUF_SIZE + USB_PACKET_SIZE]);

    static STATE: StaticCell<speaker::State> = StaticCell::new();
    let state = STATE.init(speaker::State::new());

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

    // Create the UAC1 Speaker class components
    let (stream, feedback, control_changed) = Speaker::new(
        &mut builder,
        state,
        USB_PACKET_SIZE as u16,
        uac1::SampleWidth::Width4Byte,
        &[SAMPLE_RATE_HZ],
        &[uac1::Channel::LeftFront, uac1::Channel::RightFront],
        FEEDBACK_REFRESH_PERIOD,
    );

    // Build and run the USB device
    let usb_device = builder.build();

    let sai4_resources = audio_routing::Sai4Resources {
        sai: p.SAI4,

        mclk_a: p.PE0,
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
            p.DMA2_CH0,
            p.DMA2_CH1,
            Hertz(1_000_000),
            Default::default(),
        ),
        pin_nsd: Output::new(p.PC13, Level::Low, Speed::Low),
        pin_irqz: Input::new(p.PC14, Pull::None),
    };

    let adc_resources = AdcResources {
        adc: adc::Adc::new(p.ADC1),
        pin: p.PA6.degrade_adc(),
        dma: p.DMA1_CH0,
    };

    // Establish a zero-copy channel for transferring received audio samples between tasks
    static SAMPLE_BLOCKS: StaticCell<[SampleBlock; 2]> = StaticCell::new();
    let sample_blocks = SAMPLE_BLOCKS.init([Vec::new(), Vec::new()]);

    static CHANNEL: StaticCell<zerocopy_channel::Channel<'_, NoopRawMutex, SampleBlock>> = StaticCell::new();
    let channel = CHANNEL.init(zerocopy_channel::Channel::new(sample_blocks));
    let (sender, receiver) = channel.split();

    // Trigger on USB SOF (internal signal)
    let mut tim2 = timer::low_level::Timer::new(p.TIM2);
    tim2.set_tick_freq(Hertz(FEEDBACK_COUNTER_TICK_RATE));
    tim2.set_trigger_source(timer::low_level::TriggerSource::ITR5);
    tim2.set_slave_mode(timer::low_level::SlaveMode::TRIGGER_MODE);
    tim2.regs_gp16().dier().modify(|r| r.set_tie(true));
    tim2.start();

    TIMER.lock(|p| p.borrow_mut().replace(tim2));

    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::TIM2);
    }

    let source_subscriber_0 = SOURCE_CHANNEL.subscriber().unwrap();
    let source_subscriber_1 = SOURCE_CHANNEL.subscriber().unwrap();
    let source_publisher = SOURCE_CHANNEL.publisher().unwrap();

    // FIXME: decide, based on input levels.
    source_publisher.publish_immediate(AudioSource::Usb);

    unwrap!(spawner.spawn(usb_audio::control_task(control_changed, led_yellow)));
    unwrap!(spawner.spawn(usb_audio::streaming_task(stream, feedback, sender, usb_device)));
    unwrap!(spawner.spawn(audio_routing::audio_routing_task(
        get_filters(),
        sai4_resources,
        receiver,
        source_subscriber_0,
        led_blue
    )));
    unwrap!(spawner.spawn(volume_task(source_subscriber_1)));
    unwrap!(spawner.spawn(potentiometer_task(adc_resources)));
    unwrap!(spawner.spawn(amplifier_task(amplifier_resources)));
    unwrap!(spawner.spawn(heartbeat_task(led_red)));
}

#[interrupt]
fn TIM2() {
    static mut LAST_TICKS: u32 = 0;
    static mut FRAME_COUNT: usize = 0;

    critical_section::with(|cs| {
        // Read timer counter.
        let ticks = TIMER.borrow(cs).borrow().as_ref().unwrap().regs_gp32().cnt().read();

        // Clear trigger interrupt flag.
        TIMER
            .borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .regs_gp32()
            .sr()
            .modify(|r| r.set_tif(false));

        *FRAME_COUNT += 1;
        if *FRAME_COUNT >= FEEDBACK_REFRESH_PERIOD.frame_count() {
            *FRAME_COUNT = 0;
            FEEDBACK_SIGNAL.signal(ticks - *LAST_TICKS);
            *LAST_TICKS = ticks;
        }
    });
}
