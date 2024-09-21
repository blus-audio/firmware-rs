#![no_std]
#![no_main]

use core::cell::RefCell;

use audio::{self};
use audio_routing::I2sResources;
use blus_fw::*;
use defmt::{debug, info, unwrap};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, i2c, interrupt, peripherals, timer, usb};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::blocking_mutex::{Mutex, NoopMutex};
use embassy_sync::zerocopy_channel;
use embassy_time::Timer;
use embassy_usb::class::uac1;
use embassy_usb::class::uac1::speaker::{self, Speaker};
use heapless::Vec;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

static TIMER: Mutex<CriticalSectionRawMutex, RefCell<Option<timer::low_level::Timer<peripherals::TIM2>>>> =
    Mutex::new(RefCell::new(None));
static I2C_BUS: StaticCell<NoopMutex<RefCell<i2c::I2c<'static, Async>>>> = StaticCell::new();

#[allow(unused)]
struct AmplifierResources {
    i2c: i2c::I2c<'static, Async>,
    pin_nsd: Output<'static>,
    pin_irqz: Input<'static>,
}

#[embassy_executor::task]
async fn amplifier_task(amplifier_resources: AmplifierResources) {
    use audio::tas2780::*;

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

    debug!("Reset amplifiers.");
    pin_nsd.set_low();
    Timer::after_millis(10).await;
    pin_nsd.set_high();

    // Wait for reset
    Timer::after_millis(10).await;

    debug!("Initialize TAS2780");
    for amplifier in [&mut tas2780_a, &mut tas2780_b, &mut tas2780_c, &mut tas2780_d] {
        amplifier.init().await;
    }

    loop {
        let play = I2S_ACTIVE_SIGNAL.wait().await;

        if play {
            for amplifier in [&mut tas2780_a, &mut tas2780_b, &mut tas2780_c, &mut tas2780_d] {
                amplifier.enable();
            }
        } else {
            // FIXME: Implement disable?
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
        peripheral_config.rcc.sys = Sysclk::PLL1_P;

        peripheral_config.rcc.ahb_pre = AHBPrescaler::DIV1;
        peripheral_config.rcc.apb1_pre = APBPrescaler::DIV2;
        peripheral_config.rcc.apb2_pre = APBPrescaler::DIV2;

        peripheral_config.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;

        peripheral_config.rcc.pll_src = PllSource::HSE;
        peripheral_config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV16,
            mul: PllMul::MUL250,
            divp: Some(PllPDiv::DIV6),
            divq: Some(PllQDiv::DIV8),
            divr: None,
        });

        peripheral_config.rcc.plli2s = Some(Pll {
            prediv: PllPreDiv::DIV16,
            mul: PllMul::MUL192,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        });
    }
    let p = embassy_stm32::init(peripheral_config);

    let mut core_peri = cortex_m::Peripherals::take().unwrap();

    // Enable instruction cache.
    core_peri.SCB.enable_icache();

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

    usb_config.vbus_detection = true;

    // Initialize driver for high-speed external PHY.
    let usb_driver = usb::Driver::new_fs(p.USB_OTG_FS, Irqs, p.PA12, p.PA11, ep_out_buffer, usb_config);

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
        USB_MAX_PACKET_SIZE as u16,
        uac1::SampleWidth::Width4Byte,
        &[SAMPLE_RATE_HZ],
        &AUDIO_CHANNELS,
        FEEDBACK_REFRESH_PERIOD,
    );

    // Build and run the USB device
    let usb_device = builder.build();

    let amplifier_resources = AmplifierResources {
        i2c: i2c::I2c::new(
            p.I2C1,
            p.PB6,
            p.PB7,
            Irqs,
            p.DMA1_CH6,
            p.DMA1_CH0,
            Hertz(100_000),
            Default::default(),
        ),
        pin_nsd: Output::new(p.PC13, Level::Low, Speed::Low),
        pin_irqz: Input::new(p.PC14, Pull::None),
    };

    let i2s_resources = I2sResources {
        i2s: p.SPI3,
        ck: p.PC10,
        sd: p.PC12,
        ws: p.PA4,
        mck: p.PC7,
        dma: p.DMA1_CH7,
    };

    // Establish a zero-copy channel for transferring received audio samples from the USB audio task.
    static USB_SAMPLE_BLOCKS: StaticCell<[UsbSampleBlock; 2]> = StaticCell::new();
    let usb_sample_blocks = USB_SAMPLE_BLOCKS.init([Vec::new(), Vec::new()]);

    static USB_CHANNEL: StaticCell<zerocopy_channel::Channel<'_, NoopRawMutex, UsbSampleBlock>> = StaticCell::new();
    let usb_channel = USB_CHANNEL.init(zerocopy_channel::Channel::new(usb_sample_blocks));
    let (usb_sender, usb_receiver) = usb_channel.split();

    // Trigger on USB SOF (internal signal)
    let mut tim2 = timer::low_level::Timer::new(p.TIM2);
    tim2.set_tick_freq(Hertz(FEEDBACK_COUNTER_TICK_RATE));
    tim2.set_trigger_source(timer::low_level::TriggerSource::ITR1);
    tim2.set_slave_mode(timer::low_level::SlaveMode::TRIGGER_MODE);
    tim2.regs_gp16().dier().modify(|r| r.set_tie(true));
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
    unwrap!(spawner.spawn(audio_routing::audio_routing_task(i2s_resources, usb_receiver)));

    // Amplifier setup and control.
    // unwrap!(spawner.spawn(amplifier_task(amplifier_resources)));
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
            FEEDBACK_SIGNAL.signal(ticks.wrapping_sub(*LAST_TICKS));
            *LAST_TICKS = ticks;
        }
    });
}
