#![no_std]
#![no_main]

use core::cell::RefCell;

use blus_fw::*;

use defmt::{info, unwrap};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_stm32::adc::{self, AdcChannel};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer;
use embassy_stm32::{bind_interrupts, i2c, peripherals, usb};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_sync::signal::Signal;
use embassy_sync::zerocopy_channel;
use embassy_time::{Duration, Ticker, Timer};
use embassy_usb::class::uac1;
use embassy_usb::class::uac1::speaker::{self, Speaker};
use grounded::uninit::GroundedArrayCell;
use heapless::Vec;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    OTG_HS => usb::InterruptHandler<peripherals::USB_OTG_HS>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

static VOLUME_ADC_SIGNAL: Signal<ThreadModeRawMutex, (u8, u8)> = Signal::new();

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

pub fn get_filters() -> (AudioFilter, AudioFilter, AudioFilter, AudioFilter) {
    use biquad::*;
    use micromath::F32Ext;

    type B = BiquadType;
    type C = Coefficients<f32>;

    let f_co = 1800.hz();
    let fs = SAMPLE_RATE_HZ.hz();

    let biquads_a = [
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
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
    ];
    let biquads_c = [
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
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
    ];
    let biquads_b = [
        B::new(C::from_params(Type::PeakingEQ(-9.0), fs, 1700.hz(), 0.3).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(1.0), fs, 7700.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(-1.0), fs, 12000.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(6.0), fs, 18000.hz(), 0.6).unwrap()),
        B::new(C::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
    ];
    let biquads_d = [
        B::new(C::from_params(Type::PeakingEQ(-9.0), fs, 1700.hz(), 0.3).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(1.0), fs, 7700.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(-1.0), fs, 12000.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(6.0), fs, 18000.hz(), 0.6).unwrap()),
        B::new(C::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
    ];

    // Negative gain inverts a channel.
    let gain_a = -10.0f32.powf(-10.0 / 20.0);
    let gain_b = 10.0f32.powf(-11.5 / 20.0);
    let gain_c = -10.0f32.powf(-10.0 / 20.0);
    let gain_d = 10.0f32.powf(-11.5 / 20.0);

    let delay_a: usize = 0;
    let delay_b: usize = 6;
    let delay_c: usize = 0;
    let delay_d: usize = 6;

    let f_a = AudioFilter::new(gain_a, delay_a, biquads_a);
    let f_b = AudioFilter::new(gain_b, delay_b, biquads_b);
    let f_c = AudioFilter::new(gain_c, delay_c, biquads_c);
    let f_d = AudioFilter::new(gain_d, delay_d, biquads_d);

    (f_a, f_b, f_c, f_d)
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
        let play_fut = SAI_ACTIVE_SIGNAL.wait();
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
    control_changed: speaker::ControlChanged<'static>,
    mut status_led: Output<'static>,
) {
    let adc1_measurement_buffer: &mut [u16] = unsafe {
        ADC1_MEASUREMENT_BUFFER.initialize_all_copied(0);
        let (ptr, len) = ADC1_MEASUREMENT_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    async fn measure(adc_resources: &mut AdcResources<peripherals::ADC1>, measurement_buffer: &mut [u16]) -> u8 {
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

    // Define update rates
    let mut control_changed_ticker = Ticker::every(Duration::from_hz(10));
    let mut pot_ticker = Ticker::every(Duration::from_hz(1));

    loop {
        let control_changed_fut = join(control_changed.control_changed(), control_changed_ticker.next());
        let pot_measured_fut = join(measure(&mut adc_resources, adc1_measurement_buffer), pot_ticker.next());

        match select(pot_measured_fut, control_changed_fut).await {
            Either::First((_measured, _)) => (),
            Either::Second(_) => {
                status_led.set_high();

                let volume_left: u8;
                if control_changed.is_muted(uac1::Channel::LeftFront).unwrap() {
                    volume_left = tas2780::MUTE_VOLUME;
                } else {
                    volume_left =
                        attenuation_8q8_db_to_tas2780(control_changed.volume_8q8_db(uac1::Channel::LeftFront).unwrap());
                }

                let volume_right: u8;
                if control_changed.is_muted(uac1::Channel::RightFront).unwrap() {
                    volume_right = tas2780::MUTE_VOLUME;
                } else {
                    volume_right = attenuation_8q8_db_to_tas2780(
                        control_changed.volume_8q8_db(uac1::Channel::RightFront).unwrap(),
                    );
                }

                VOLUME_ADC_SIGNAL.signal((volume_left, volume_right));
                status_led.set_low();
            }
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
        uac1::FeedbackRefreshPeriod::Period8ms,
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

    // Trigger on timer 2 (internal signal)
    // let tim1 = timer::low_level::Timer::new(p.TIM1);
    // tim1.set_trigger_source(timer::low_level::TriggerSource::ITR1);
    // tim1.start();

    // Trigger on USB SOF (internal signal)
    // let mut tim2 = timer::low_level::Timer::new(p.TIM2);
    // tim2.set_tick_freq(Hertz(24_576_000));
    // tim2.set_trigger_source(timer::low_level::TriggerSource::ITR5);
    // tim2.set_slave_mode(timer::low_level::SlaveMode::TRIGGER_MODE);
    // tim2.regs_gp16().dier().modify(|r| r.set_tie(true));
    // tim2.start();

    // unsafe {
    //     cortex_m::peripheral::NVIC::unmask(interrupt::TIM2);
    // }

    unwrap!(spawner.spawn(usb_audio::usb_audio_task(stream, feedback, sender, usb_device)));
    unwrap!(spawner.spawn(audio_routing::audio_routing_task(
        get_filters(),
        sai4_resources,
        receiver,
        led_blue
    )));
    unwrap!(spawner.spawn(heartbeat_task(led_red)));
    unwrap!(spawner.spawn(volume_control_task(adc_resources, control_changed, led_yellow)));
    unwrap!(spawner.spawn(amplifier_task(amplifier_resources)));
}

#[interrupt]
fn TIM2() {}
