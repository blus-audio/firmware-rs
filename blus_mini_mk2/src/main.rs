#![no_std]
#![no_main]

use core::cell::RefCell;

use audio::{self, AudioFilter, AudioSource};
use blus_fw::*;
use core::sync::atomic::Ordering::Relaxed;
use defmt::{debug, info, trace, unwrap};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_stm32::adc::{self, AdcChannel};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::spdifrx::{self, Spdifrx};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, i2c, interrupt, peripherals, timer, usb};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::blocking_mutex::{Mutex, NoopMutex};
use embassy_sync::zerocopy_channel;
use embassy_time::{Duration, Ticker, Timer, WithTimeout};
use embassy_usb::class::uac1;
use embassy_usb::class::uac1::speaker::{self, Speaker};
use grounded::uninit::GroundedArrayCell;
use heapless::Vec;
use micromath::F32Ext;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    OTG_HS => usb::InterruptHandler<peripherals::USB_OTG_HS>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    SPDIF_RX => spdifrx::GlobalInterruptHandler<peripherals::SPDIFRX1>;
});

static TIMER: Mutex<CriticalSectionRawMutex, RefCell<Option<timer::low_level::Timer<peripherals::TIM2>>>> =
    Mutex::new(RefCell::new(None));
static I2C_BUS: StaticCell<NoopMutex<RefCell<i2c::I2c<'static, Async>>>> = StaticCell::new();

// Accessible by most system masters (Zone D2)
#[link_section = ".sram1"]
static mut ADC1_MEASUREMENT_BUFFER: GroundedArrayCell<u16, 1> = GroundedArrayCell::uninit();

// Reserve twice the SPDIF sample count, since the DMA will transfer at
// half-full interrupt (so, at SPDIF_SAMPLE_COUNT * 2 / 2).
#[link_section = ".sram1"]
static mut SPDIFRX_BUFFER: GroundedArrayCell<u32, { SPDIF_SAMPLE_COUNT * 2 }> = GroundedArrayCell::uninit();

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
struct SpdifResources {
    spdifrx: peripherals::SPDIFRX1,
    in_pin: peripherals::PD7,
    dma: peripherals::DMA1_CH1,
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
async fn amplifier_task(amplifier_resources: AmplifierResources) {
    use audio::tas2780::*;

    let mut pin_nsd = amplifier_resources.pin_nsd;

    let i2c_bus = NoopMutex::new(RefCell::new(amplifier_resources.i2c));
    let i2c_bus = I2C_BUS.init(i2c_bus);

    let mut ic2_device_a = I2cDevice::new(i2c_bus);
    let mut tas2780_a = Tas2780::new(&mut ic2_device_a, 0x39);

    let mut ic2_device_b = I2cDevice::new(i2c_bus);
    let mut tas2780_b = Tas2780::new(&mut ic2_device_b, 0x3a);

    let mut ic2_device_c = I2cDevice::new(i2c_bus);
    let mut tas2780_c = Tas2780::new(&mut ic2_device_c, 0x3d);

    let mut ic2_device_d = I2cDevice::new(i2c_bus);
    let mut tas2780_d = Tas2780::new(&mut ic2_device_d, 0x3e);

    debug!("Reset amplifiers.");
    pin_nsd.set_low();
    Timer::after_millis(10).await;
    pin_nsd.set_high();

    // Wait for reset
    Timer::after_millis(10).await;

    tas2780_a
        .init(Config {
            tdm_slot: 3,
            ..Default::default()
        })
        .await;
    tas2780_b
        .init(Config {
            tdm_slot: 0,
            ..Default::default()
        })
        .await;
    tas2780_c
        .init(Config {
            tdm_slot: 1,
            ..Default::default()
        })
        .await;
    tas2780_d
        .init(Config {
            tdm_slot: 2,
            ..Default::default()
        })
        .await;

    loop {
        let source = SAI_ACTIVE_SIGNAL.wait().await;

        if !matches!(source, AudioSource::None) {
            debug!("Initialize TAS2780");

            for amplifier in [&mut tas2780_a, &mut tas2780_b, &mut tas2780_c, &mut tas2780_d] {
                let mut config = amplifier.config();

                match source {
                    AudioSource::Spdif => {
                        config.tdm_word_length = TdmWordLength::Word16Bit;
                        config.tdm_time_slot_length = TdmTimeSlotLength::Slot16Bit;
                    }
                    _ => {
                        config.tdm_word_length = TdmWordLength::Word32Bit;
                        config.tdm_time_slot_length = TdmTimeSlotLength::Slot32Bit;
                    }
                };

                amplifier.init(config).await;
                amplifier.enable();
            }

            AMP_SETUP_SIGNAL.signal(true);
        } else {
            AMP_SETUP_SIGNAL.signal(false);
        }
    }
}

#[embassy_executor::task]
async fn potentiometer_task(mut adc_resources: AdcResources<peripherals::ADC1>) {
    use biquad::*;

    const POT_SAMPLE_RATE_HZ: u64 = 25;
    const POT_CUT_OFF_HZ: f32 = 5.0;

    let mut ticker = Ticker::every(Duration::from_hz(POT_SAMPLE_RATE_HZ));

    let buffer: &mut [u16] = unsafe {
        ADC1_MEASUREMENT_BUFFER.initialize_all_copied(0);
        let (ptr, len) = ADC1_MEASUREMENT_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let coefficients = Coefficients::<f32>::from_params(
        Type::LowPass,
        (POT_SAMPLE_RATE_HZ as f32).hz(),
        POT_CUT_OFF_HZ.hz(),
        0.5,
    )
    .unwrap();

    let mut filter = DirectForm2Transposed::<f32>::new(coefficients);

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

        let mut gain = filter.run((buffer[0] as f32) / 65535f32);

        // Clamping.
        if gain < 0.0 {
            gain = 0.0;
        } else if gain > 1.0 {
            gain = 1.0;
        }

        // Make gain exponential
        let exp_gain = gain.powf(2.0);
        POT_GAIN_SIGNAL.signal(exp_gain);
    }
}

#[embassy_executor::task]
async fn spdif_task(
    mut resources: SpdifResources,
    mut sender: zerocopy_channel::Sender<'static, NoopRawMutex, SpdifSampleBlock>,
) {
    let buffer: &mut [u32] = unsafe {
        SPDIFRX_BUFFER.initialize_all_copied(0);
        let (ptr, len) = SPDIFRX_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    fn new_spdif<'d>(resources: &'d mut SpdifResources, buffer: &'d mut [u32]) -> Spdifrx<'d, peripherals::SPDIFRX1> {
        Spdifrx::new_data_only(
            &mut resources.spdifrx,
            Irqs,
            spdifrx::Config::default(),
            &mut resources.in_pin,
            &mut resources.dma,
            buffer,
        )
    }

    info!("Start S/PDIF");

    let mut spdif = new_spdif(&mut resources, buffer);
    spdif.start();

    let mut ticker = Ticker::every(Duration::from_millis(100));

    loop {
        let mut data = [0u32; SPDIF_SAMPLE_COUNT];
        let result = spdif.read_data(&mut data).await;

        SPDIF_IS_STREAMING.store(result.is_ok(), Relaxed);

        match result {
            Ok(_) => {
                if let Some(send_data) = sender.try_send() {
                    send_data.copy_from_slice(data.as_slice());
                    sender.send_done();
                }
            }
            Err(spdifrx::Error::RingbufferError(_)) => {
                // Limit renewal rate.
                ticker.next().await;

                info!("SPDIFRX ringbuffer error. Renew.");
                drop(spdif);
                spdif = new_spdif(&mut resources, buffer);
                spdif.start();
            }
            Err(_) => {
                // Limit loop period.
                ticker.next().await;
            }
        };
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
            divr: Some(PllDiv::DIV4), // 96 MHz for SPDIFRX (good for up to 136 kHz audio sample rate)
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
        peripheral_config.rcc.mux.spdifrxsel = mux::Spdifrxsel::PLL3_R;
    }
    let p = embassy_stm32::init(peripheral_config);

    let mut core_peri = cortex_m::Peripherals::take().unwrap();

    // Enable instruction cache.
    core_peri.SCB.enable_icache();

    let mut led_blue = Output::new(p.PC6, Level::Low, Speed::Low);
    let mut led_green = Output::new(p.PC7, Level::Low, Speed::Low);
    let mut led_yellow = Output::new(p.PC8, Level::Low, Speed::Low);
    let mut led_red = Output::new(p.PC9, Level::Low, Speed::Low);

    for led in [&mut led_blue, &mut led_green, &mut led_yellow, &mut led_red] {
        led.set_low();
    }

    debug!("USB packet size is {} byte", USB_MAX_PACKET_SIZE);
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    let config_descriptor = CONFIG_DESCRIPTOR.init([0; 256]);

    static BOS_DESCRIPTOR: StaticCell<[u8; 32]> = StaticCell::new();
    let bos_descriptor = BOS_DESCRIPTOR.init([0; 32]);

    const CONTROL_BUF_SIZE: usize = 64;
    static CONTROL_BUF: StaticCell<[u8; CONTROL_BUF_SIZE]> = StaticCell::new();
    let control_buf = CONTROL_BUF.init([0; CONTROL_BUF_SIZE]);

    const FEEDBACK_BUF_SIZE: usize = 4;
    static EP_OUT_BUFFER: StaticCell<[u8; FEEDBACK_BUF_SIZE + CONTROL_BUF_SIZE + USB_MAX_PACKET_SIZE]> =
        StaticCell::new();
    let ep_out_buffer = EP_OUT_BUFFER.init([0u8; FEEDBACK_BUF_SIZE + CONTROL_BUF_SIZE + USB_MAX_PACKET_SIZE]);

    static STATE: StaticCell<speaker::State> = StaticCell::new();
    let state = STATE.init(speaker::State::new());

    // Create the driver, from the HAL.
    let mut usb_config = usb::Config::default();

    // Do not enable vbus_detection with an external HS PHY.
    usb_config.vbus_detection = false;

    // Using a Microchip PHY requires a delay during setup.
    usb_config.xcvrdly = true;

    // Initialize driver for high-speed external PHY.
    #[cfg(feature = "usb_high_speed")]
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

    #[cfg(not(feature = "usb_high_speed"))]
    let usb_driver = usb::Driver::new_fs_ulpi(
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
    let mut config = embassy_usb::Config::new(0x1209, 0xaf03);
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
        USB_MAX_PACKET_SIZE as u16,
        uac1::SampleWidth::Width4Byte,
        &[SAMPLE_RATE_HZ],
        &AUDIO_CHANNELS,
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

    let spdif_resources = SpdifResources {
        spdifrx: p.SPDIFRX1,
        in_pin: p.PD7,
        dma: p.DMA1_CH1,
    };

    // FIXME: deduplicate

    // Establish a zero-copy channel for transferring received audio samples from the USB audio task.
    static USB_SAMPLE_BLOCKS: StaticCell<[UsbSampleBlock; 2]> = StaticCell::new();
    let usb_sample_blocks = USB_SAMPLE_BLOCKS.init([Vec::new(), Vec::new()]);

    static USB_CHANNEL: StaticCell<zerocopy_channel::Channel<'_, NoopRawMutex, UsbSampleBlock>> = StaticCell::new();
    let usb_channel = USB_CHANNEL.init(zerocopy_channel::Channel::new(usb_sample_blocks));
    let (usb_sender, usb_receiver) = usb_channel.split();

    // Establish a zero-copy channel for transferring received audio samples from the S/PDIF task.
    static SPDIF_SAMPLE_BLOCKS: StaticCell<[SpdifSampleBlock; 2]> = StaticCell::new();
    let spdif_sample_blocks = SPDIF_SAMPLE_BLOCKS.init([[0u32; SPDIF_SAMPLE_COUNT]; 2]);

    static SPDIF_CHANNEL: StaticCell<zerocopy_channel::Channel<'_, NoopRawMutex, SpdifSampleBlock>> = StaticCell::new();
    let spdif_channel = SPDIF_CHANNEL.init(zerocopy_channel::Channel::new(spdif_sample_blocks));
    let (spdif_sender, spdif_receiver) = spdif_channel.split();

    // Trigger a capture on USB SOF (internal signal)
    let mut tim2 = timer::low_level::Timer::new(p.TIM2);
    tim2.set_tick_freq(Hertz(FEEDBACK_COUNTER_TICK_RATE));
    tim2.set_trigger_source(timer::low_level::TriggerSource::ITR5);

    const CHANNEL: timer::Channel = timer::Channel::Ch1;
    tim2.set_input_ti_selection(CHANNEL, timer::low_level::InputTISelection::TRC);
    tim2.set_input_capture_prescaler(CHANNEL, 0);
    tim2.set_input_capture_filter(CHANNEL, timer::low_level::FilterValue::FCK_INT_N2);

    // Reset all interrupt flags.
    tim2.regs_gp32().sr().write(|r| r.0 = 0);

    tim2.enable_channel(CHANNEL, true);
    tim2.enable_input_interrupt(CHANNEL, true);

    tim2.start();

    TIMER.lock(|p| p.borrow_mut().replace(tim2));

    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::TIM2);
    }

    // Launch USB audio tasks.
    unwrap!(spawner.spawn(usb_audio::control_task(control_changed)));
    unwrap!(spawner.spawn(usb_audio::streaming_task(stream, usb_sender)));
    unwrap!(spawner.spawn(usb_audio::feedback_task(feedback)));
    unwrap!(spawner.spawn(usb_audio::usb_task(usb_device)));

    // Launch audio routing.
    unwrap!(spawner.spawn(audio_routing::audio_routing_task(
        get_filters(),
        sai4_resources,
        usb_receiver,
        spdif_receiver,
        led_blue,
        led_red,
        led_yellow,
        led_green
    )));

    // Volume control.
    unwrap!(spawner.spawn(potentiometer_task(adc_resources)));

    // Amplifier setup and control.
    unwrap!(spawner.spawn(amplifier_task(amplifier_resources)));

    // S/PDIF data reception.
    unwrap!(spawner.spawn(spdif_task(spdif_resources, spdif_sender)));
}

#[interrupt]
fn TIM2() {
    static mut LAST_TICKS: u32 = 0;
    static mut FRAME_COUNT: usize = 0;

    critical_section::with(|cs| {
        // Read timer counter.
        let timer = TIMER.borrow(cs).borrow().as_ref().unwrap().regs_gp32();

        let status = timer.sr().read();

        const CHANNEL_INDEX: usize = 0;
        if status.ccif(CHANNEL_INDEX) {
            let ticks = timer.ccr(CHANNEL_INDEX).read();

            *FRAME_COUNT += 1;
            if *FRAME_COUNT >= FEEDBACK_REFRESH_PERIOD.frame_count() {
                *FRAME_COUNT = 0;
                FEEDBACK_SIGNAL.signal(ticks.wrapping_sub(*LAST_TICKS));
                *LAST_TICKS = ticks;
            }
        };

        // Clear trigger interrupt flag.
        timer.sr().modify(|r| r.set_tif(false));
    });
}
