//! A driver for controlling the TAS2780 class-D amplifier.
use defmt::debug;
use embassy_time::Timer;
use embedded_hal::i2c;

type RegisterAddress = u8;
type RegisterValue = u8;

/// The volume at which the amplifier is muted
pub const MUTED_VOLUME: u8 = 0xFF;

/// The currently active page
const PAGE_REGISTER: RegisterAddress = 0x00;

/// The currently active book
const BOOK_REGISTER: RegisterAddress = 0x7F;

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum TdmWordLength {
    Word16Bit = 0x00,
    Word20Bit = 0x01,
    Word24Bit = 0x02,
    Word32Bit = 0x03,
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum TdmTimeSlotLength {
    Slot16Bit = 0x00,
    Slot24Bit = 0x01,
    Slot32Bit = 0x02,
}

#[repr(u8)]
#[derive(Clone, Copy)]
/// The amplifier gain setting.
pub enum Gain {
    Gain11_0dBV = 0x00,
    Gain11_5dBV = 0x01,
    Gain12_0dBV = 0x02,
    Gain12_5dBV = 0x03,
    Gain13_0dBV = 0x04,
    Gain13_5dBV = 0x05,
    Gain14_0dBV = 0x06,
    Gain14_5dBV = 0x07,
    Gain15_0dBV = 0x08,
    Gain15_5dBV = 0x09,
    Gain16_0dBV = 0x0A,
    Gain16_5dBV = 0x0B,
    Gain17_0dBV = 0x0C,
    Gain17_5dBV = 0x0D,
    Gain18_0dBV = 0x0E,
    Gain18_5dBV = 0x0F,
    Gain19_0dBV = 0x10,
    Gain19_5dBV = 0x11,
    Gain20_0dBV = 0x12,
    Gain20_5dBV = 0x13,
    Gain21_0dBV = 0x14,
}

#[derive(Clone, Copy)]
/// The amplifier playback channel.
pub enum Channel {
    Left,
    Right,
    StereoMix,
}

#[derive(Clone, Copy)]
/// The amplifier power mode.
pub enum PowerMode {
    Zero,
    One,
    Two,
    Three,
}

#[repr(u8)]
#[derive(Clone, Copy)]
/// The duration of the noise gate hysteresis.
pub enum NoiseGateHysteresis {
    Duration400us = 0x0,
    Duration600us = 0x1,
    Duration800us = 0x2,
    Duration2ms = 0x3,
    Duration10ms = 0x4,
    Duration50ms = 0x5,
    Duration100ms = 0x6,
    Duration1000ms = 0x7,
}

#[repr(u8)]
#[derive(Clone, Copy)]
/// The threshold of the noise gate.
pub enum NoiseGateLevel {
    ThresholdMinus90dBFS = 0x0,
    ThresholdMinus100dBFS = 0x1,
    ThresholdMinus110dBFS = 0x2,
    ThresholdMinus120dBFS = 0x3,
}

#[derive(Clone, Copy)]
pub struct NoiseGate {
    pub hysteresis: NoiseGateHysteresis,
    pub level: NoiseGateLevel,
}

#[derive(Clone, Copy)]
pub struct Config {
    pub gain: Gain,
    pub channel: Channel,
    pub tdm_slot: u8,
    pub tdm_word_length: TdmWordLength,
    pub tdm_time_slot_length: TdmTimeSlotLength,
    pub noise_gate: Option<NoiseGate>,
    pub power_mode: PowerMode,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            gain: Gain::Gain11_0dBV,
            tdm_slot: 0,
            tdm_word_length: TdmWordLength::Word32Bit,
            tdm_time_slot_length: TdmTimeSlotLength::Slot32Bit,
            channel: Channel::Left,
            noise_gate: Some(NoiseGate {
                hysteresis: NoiseGateHysteresis::Duration1000ms,
                level: NoiseGateLevel::ThresholdMinus90dBFS,
            }),
            power_mode: PowerMode::Two,
        }
    }
}

/// TAS2780 driver structure.
pub struct Tas2780<'d, I2C> {
    i2c: &'d mut I2C,
    address: i2c::SevenBitAddress,
    config: Config,
    page: Option<u8>,
    book: Option<u8>,
}

impl<'d, I2C> Tas2780<'d, I2C>
where
    I2C: i2c::I2c,
{
    pub fn new(i2c: &'d mut I2C, address: i2c::SevenBitAddress) -> Self {
        Tas2780 {
            i2c,
            address,
            config: Config::default(),
            page: None,
            book: None,
        }
    }

    fn write(&mut self, write: &[u8]) {
        self.i2c.write(self.address, write).expect("Failed to write to TAS2780")
    }

    fn write_register(&mut self, address: RegisterAddress, value: RegisterValue) {
        // Handle special register addresses.
        match address {
            PAGE_REGISTER => self.page = Some(value),
            BOOK_REGISTER => self.book = Some(value),
            _ => (),
        }

        let write = [address, value];
        self.write(&write)
    }

    fn set_page(&mut self, value: RegisterValue) {
        if let Some(page) = self.page {
            if page == value {
                return;
            }
        }

        self.write_register(PAGE_REGISTER, value)
    }

    fn set_book(&mut self, value: RegisterValue) {
        if let Some(book) = self.book {
            if book == value {
                return;
            }
        }

        self.write_register(BOOK_REGISTER, value)
    }

    fn _read(&mut self, address: u8, read: &mut [u8]) {
        let address: [u8; 1] = [address];

        self.i2c
            .write_read(self.address, &address, read)
            .expect("Failed to read from TAS2780")
    }

    /// Set the attenuation in steps of 0.5 dB.
    /// For example, an input of 0 gives an attenuation of 0 dB. An input of 100 gives -50 dB.
    /// Values that exceed -100 dB (an input of 200) mute the amplifier.
    pub fn set_volume(&mut self, attenuation_half_db: u8) {
        self.set_page(0);

        /// Digital volume control
        const DVC_REGISTER: RegisterAddress = 0x1A;

        // FIXME: convert volume
        self.write_register(DVC_REGISTER, attenuation_half_db)
    }

    async fn reset(&mut self) {
        // Return to default page and book.
        self.set_page(0);
        self.set_book(0);

        /// Software reset
        const SOFTWARE_RESET_REGISTER: RegisterAddress = 0x01;

        // Perform soft reset.
        self.write_register(SOFTWARE_RESET_REGISTER, 0x01);

        // Wait for startup.
        Timer::after_millis(1).await;

        self.page = None;
        self.book = None;
    }

    pub fn enable(&mut self) {
        debug!("Enabling TAS2780 at address {}", self.address);

        // Set up power mode, and activate
        match self.config.power_mode {
            PowerMode::Two => {
                self.set_page(0x00);
                self.write_register(0x03, 0b11 << 6 | (self.config.gain as u8) << 1); // PWR_MODE2
                self.write_register(0x04, 0xA1); // Use internal LDO
                self.write_register(0x71, 0x0E); // PVDD undervoltage lockout 6.5 V
                self.write_register(0x02, 0x80); // Power up playback with I-sense, V-sense enabled
            }
            _ => todo!("Unsupported power mode"),
        }
    }

    pub fn config(&self) -> Config {
        self.config
    }

    /// Initialize a TAS2780 amplifier to default settings.
    pub async fn init(&mut self, config: Config) {
        self.config = config;

        debug!("Initializing TAS2780 at address {}", self.address);

        // Pre-reset configuration (as per the datasheet)
        self.set_page(0x01);
        self.write_register(0x37, 0x3A); // Bypass

        self.set_page(0xFD);
        self.write_register(0x0D, 0x0D); // Allow page access
        self.write_register(0x06, 0xC1); // Set Dmin

        self.set_page(0x01);
        self.write_register(0x19, 0xC0); // Force modulation

        self.set_page(0xFD);
        self.write_register(0x0D, 0x0D); // Allow page access
        self.write_register(0x06, 0xD5); // Set Dmin

        self.reset().await;

        // Post-reset configuration (as per the datasheet)
        self.set_page(0x01);
        self.write_register(0x37, 0x3A); // Bypass

        self.set_page(0xFD);
        self.write_register(0x0D, 0x0D); // Allow page access
        self.write_register(0x06, 0xC1); // Set Dmin
        self.write_register(0x06, 0xD5); // Set Dmin

        self.set_page(0x00);

        // Clock-based power features
        const INT_CLK_CFG_REGISTER: RegisterAddress = 0x5C;
        let int_clk_cfg = 0x1 << 7 // Enable clock-based power up/down feature
        | 0x3 << 3; // 52.42 ms clock error detection period

        self.write_register(INT_CLK_CFG_REGISTER, int_clk_cfg);

        // Set up TDM/channel configuration
        let tdm_cfg1: RegisterValue = 0x00;
        let mut tdm_cfg2: RegisterValue = 0x00;
        let mut tdm_cfg3: RegisterValue = 0x00;

        tdm_cfg2 |= (self.config.tdm_time_slot_length as u8) & 0b11;
        tdm_cfg2 |= ((self.config.tdm_word_length as u8) & 0b11) << 2;

        match self.config.channel {
            Channel::Left => {
                tdm_cfg2 |= 0b01 << 4; // RX_SCFG: Mono left channel
                tdm_cfg3 |= self.config.tdm_slot & 0xF;
            }
            Channel::Right => {
                tdm_cfg2 |= 0b10 << 4; // RX_SCFG: Mono right channel
                tdm_cfg3 |= (self.config.tdm_slot & 0xF) << 4;
            }
            Channel::StereoMix => {
                tdm_cfg2 |= 0b11 << 4; // RX_SCFG: Stereo downmix (L+R)/2

                // FIXME: channel selection in tdm_cfg3
            }
        }

        const TDM_CFG1_REGISTER: RegisterAddress = 0x09;
        const TDM_CFG2_REGISTER: RegisterAddress = 0x0A;
        const TDM_CFG3_REGISTER: RegisterAddress = 0x0C;

        self.write_register(TDM_CFG1_REGISTER, tdm_cfg1);
        self.write_register(TDM_CFG2_REGISTER, tdm_cfg2);
        self.write_register(TDM_CFG3_REGISTER, tdm_cfg3);

        // Set up the noise gate, if enabled
        if let Some(noise_gate) = self.config.noise_gate {
            const ENABLE_NOISE_GATE: u8 = 0b1;

            /// Noise gate config register 0
            const NG_CFG0_REGISTER: RegisterAddress = 0x35;

            self.write_register(
                NG_CFG0_REGISTER,
                (noise_gate.hysteresis as u8) << 5 | (noise_gate.level as u8) << 3 | ENABLE_NOISE_GATE << 2 | 0b01,
            );
        }

        // Set up power mode, and activate
        match self.config.power_mode {
            PowerMode::Two => {
                self.set_page(0x01);
                self.write_register(0x17, 0xC0); // SARBurstMask = 0
                self.write_register(0x19, 0x00); // LSR mode
                self.write_register(0x21, 0x00); // Disable comparator histeresis
                self.write_register(0x35, 0x74); // Minimize noise

                self.set_page(0xFD);
                self.write_register(0x0D, 0x0D); // Allow page access
                self.write_register(0x3E, 0x4A); // Optimal Dmin
                self.write_register(0x0D, 0x00); // Remove page access
            }
            _ => todo!("Unsupported power mode"),
        }

        // Set maximum volume (0 dB)
        self.set_volume(0);
    }
}
