#![no_std]
#![no_main]

use blus_fw::uac1;
use core::cell::RefCell;
use defmt::{panic, *};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::mode::Async;
use embassy_stm32::{bind_interrupts, i2c, peripherals, sai, usb};
use embassy_stm32::{
    gpio::{Input, Level, Output, Pull, Speed},
    time::Hertz,
};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use grounded::uninit::GroundedArrayCell;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    OTG_HS => usb::InterruptHandler<peripherals::USB_OTG_HS>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

const DMA_BUFFER_SIZE: usize = 1024;
const USB_PACKET_SIZE: usize = 96;

static VOLUME_ADC_SIGNAL: Signal<ThreadModeRawMutex, u8> = Signal::new();
static SAI4A_DMA_SIGNAL: Signal<ThreadModeRawMutex, ([u8; USB_PACKET_SIZE], usize)> = Signal::new();

static I2C_BUS: StaticCell<NoopMutex<RefCell<i2c::I2c<'static, Async>>>> = StaticCell::new();

// Accessible by most system masters (Zone D2)
#[link_section = ".sram1"]
static mut SAI1A_WRITE_BUFFER: GroundedArrayCell<u8, DMA_BUFFER_SIZE> = GroundedArrayCell::uninit();

// Accessible by BDMA (Zone D3)
#[link_section = ".sram4"]
static mut SRAM4: [u8; 16 * 1024] = [0u8; 16 * 1024];

// static mut SAI4A_WRITE_BUFFER: GroundedArrayCell<u8, DMA_BUFFER_SIZE> = GroundedArrayCell::uninit();

#[embassy_executor::task]
async fn sai1a_task(mut sai_driver: sai::Sai<'static, peripherals::SAI1, u8>, mut status_led: Option<Output<'static>>) {
    sai_driver.start();

    if let Some(status_led) = &mut status_led {
        status_led.set_low();
    }

    let data = [0u8; 48];
    loop {
        if let Some(status_led) = &mut status_led {
            status_led.toggle();
        }
        unwrap!(sai_driver.write(&data).await);
    }
}

#[embassy_executor::task]
async fn sai4a_task(mut sai_driver: sai::Sai<'static, peripherals::SAI4, u8>, mut status_led: Option<Output<'static>>) {
    if let Some(status_led) = &mut status_led {
        status_led.set_low();
    }

    let mut dummy_data = [0u8; 256];
    for i in 0..256 {
        dummy_data[i] = i as u8;
    }

    Timer::after_secs(1).await;

    sai_driver.start();
    loop {
        unwrap!(sai_driver.write(&dummy_data).await);
        // let data = SAI4A_DMA_SIGNAL.try_take();

        // match data {
        //     Some((data, length)) => {
        //         if let Some(status_led) = &mut status_led {
        //             status_led.set_high();
        //         }
        //         unwrap!(sai_driver.write(&data[..length]).await);
        //         unwrap!(sai_driver.write(&data[..length]).await);
        //     }
        //     None => {
        //         if let Some(status_led) = &mut status_led {
        //             status_led.set_low();
        //         }
        //         unwrap!(sai_driver.write(&dummy_data).await)
        //     }
        // }
    }
}

#[embassy_executor::task]
async fn tas2780_task(i2c_driver: i2c::I2c<'static, Async>) {
    use blus_fw::tas2780::*;

    let i2c_bus = NoopMutex::new(RefCell::new(i2c_driver));
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
        }
    }
}

#[embassy_executor::task]
async fn volume_control_task(mut adc: Adc<'static, peripherals::ADC1>, mut adc_pin: peripherals::PA6) {
    adc.set_sample_time(SampleTime::CYCLES810_5);

    let mut last_measured = 0u8;

    loop {
        let measured = adc.blocking_read(&mut adc_pin);
        let measured = (100f32 * (measured as f32) / 65535f32) as u8;

        if last_measured != measured {
            VOLUME_ADC_SIGNAL.signal(measured);
            last_measured = measured;
        }

        Timer::after_millis(50).await;
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
    info!("Hello World!");

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

    let mut spk_nshutdown = Output::new(p.PC13, Level::High, Speed::Low);
    let _spk_irqz = Input::new(p.PC12, Pull::None);

    info!("Reset amplifiers.");
    spk_nshutdown.set_low();
    Timer::after_millis(1).await;
    spk_nshutdown.set_high();

    Timer::after_millis(100).await;

    let led_blue = Output::new(p.PC6, Level::High, Speed::Low);
    let mut led_green = Output::new(p.PC7, Level::High, Speed::Low);
    let mut led_yellow = Output::new(p.PC8, Level::High, Speed::Low);
    let led_red = Output::new(p.PC9, Level::High, Speed::Low);

    led_green.set_low();
    led_yellow.set_low();

    unwrap!(spawner.spawn(heartbeat_task(led_red)));

    let adc = Adc::new(p.ADC1);
    let adc_pin = p.PA6;
    unwrap!(spawner.spawn(volume_control_task(adc, adc_pin)));

    // Create the driver, from the HAL.
    let mut ep_out_buffer = [0u8; 2048];
    let mut usb_config = usb::Config::default();

    // Do not enable vbus_detection with an external HS PHY.
    usb_config.vbus_detection = false;
    // Using a Microchip PHY requires a delay during setup.
    usb_config.xcvrdly = true;

    // SAI1 setup
    let sai1a_config = sai::Config::default();
    let _sai1b_config = sai::Config::default();

    // SAI4A outputs to the amplifiers.
    let mut sai4a_config = sai::Config::default();

    sai4a_config.slot_count = sai::word::U4(4);
    sai4a_config.slot_enable = 0xFFFF; // All slots
    sai4a_config.data_size = sai::DataSize::Data32;
    sai4a_config.fifo_threshold = sai::FifoThreshold::Empty;
    sai4a_config.frame_sync_offset = sai::FrameSyncOffset::OnFirstBit;
    sai4a_config.frame_sync_active_level_length = sai::word::U7(1);
    sai4a_config.frame_sync_definition = sai::FrameSyncDefinition::StartOfFrame;
    sai4a_config.frame_length = 4 * 32;
    sai4a_config.clock_strobe = sai::ClockStrobe::Falling;

    let _sai4b_config = sai::Config::default();

    let (sai4a, _sai4b) = sai::split_subblocks(p.SAI4);
    let (sai1a, _sai1b) = sai::split_subblocks(p.SAI1);

    let sai1a_write_buffer: &mut [u8] = unsafe {
        SAI1A_WRITE_BUFFER.initialize_all_copied(0);
        let (ptr, len) = SAI1A_WRITE_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };
    let _sai1a_driver = sai::Sai::new_asynchronous_with_mclk(
        sai1a,
        p.PE5,
        p.PE6,
        p.PE4,
        p.PE2,
        p.DMA1_CH0,
        sai1a_write_buffer,
        sai1a_config,
    );
    // let mut sai1b_driver = sai::Sai::new_synchronous(sai1b, p.PE3, p.DMA1_CH1, &mut dma_buf, sai1a_config);

    // let sai4a_write_buffer: &mut [u8] = unsafe {
    //     SAI4A_WRITE_BUFFER.initialize_all_copied(0);
    //     let (ptr, len) = SAI4A_WRITE_BUFFER.get_ptr_len();
    //     core::slice::from_raw_parts_mut(ptr, len)
    // };
    static SAI4A_WRITE_BUFFER: StaticCell<&mut [u8]> = StaticCell::new();
    let sai4a_write_buffer = SAI4A_WRITE_BUFFER.init(unsafe { &mut SRAM4[..DMA_BUFFER_SIZE] });
    let sai4a_driver = sai::Sai::new_asynchronous(
        sai4a,
        p.PD13,
        p.PC1,
        p.PD12,
        p.BDMA_CH0,
        sai4a_write_buffer,
        sai4a_config,
    );

    // let sai4b_driver =
    //     sai::Sai::new_asynchronous(sai4b, p.PE12, p.PE11, p.PE13, p.BDMA_CH1, &mut dma_buf, sai4b_config);

    // unwrap!(spawner.spawn(sai1a_worker(sai1a_driver, None)));

    // static DATA: StaticCell<&mut [u8]> = StaticCell::new();
    // let data = DATA.init(unsafe { &mut SRAM4[DMA_SAMPLE_COUNT..2 * DMA_SAMPLE_COUNT] });

    unwrap!(spawner.spawn(sai4a_task(sai4a_driver, Some(led_blue))));

    let i2c_driver = i2c::I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        Hertz(100_000),
        Default::default(),
    );

    unwrap!(spawner.spawn(tas2780_task(i2c_driver)));

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
    config.serial_number = Some("12345678");
    config.self_powered = true;
    config.max_power = 0;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = uac1::State::new();

    let mut builder = Builder::new(
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
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    let receive_fut = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = receive(&mut class).await;
            info!("Disconnected");
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
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
) -> Result<(), Disconnected> {
    loop {
        let mut usb_data = [0u8; USB_PACKET_SIZE];
        let n = class.read_packet(&mut usb_data).await?;
        SAI4A_DMA_SIGNAL.signal((usb_data, n));
    }
}
