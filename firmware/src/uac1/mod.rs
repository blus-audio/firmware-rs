//! USB Audio class
//!
//! This crate provides a USB device class based on "Universal Serial Bus Device
//! Class Definition for Audio Devices", Release 1.0.
use class_codes::*;

use core::cell::{Cell, RefCell};
use core::future::poll_fn;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::task::Poll;
use defmt::trace;
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_sync::waitqueue::WakerRegistration;
use embassy_usb::control::{self, InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::descriptor::{SynchronizationType, UsageType};
use embassy_usb::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut, EndpointType};
use embassy_usb::types::InterfaceNumber;
use embassy_usb::{Builder, Handler};

mod terminal_type;
pub use terminal_type::TerminalType;

mod channel_config;
pub use channel_config::ChannelConfig;

mod class_codes;

/// The number of audio channels (e.g. 2 for left+right)
const AUDIO_CHANNEL_COUNT: usize = 2;

/// Arbitrary unique identifier for the input unit
const INPUT_UNIT_ID: u8 = 0x01;

/// Arbitrary unique identifier for the feature unit
const FEATURE_UNIT_ID: u8 = 0x02;

/// Arbitrary unique identifier for the output unit
const OUTPUT_UNIT_ID: u8 = 0x03;

// Volume settings go from -25600 to 0, in steps of 256.
// Therefore, the volume settings are 8q8 values in units of dB.
const VOLUME_STEPS_PER_DB: i16 = 256;
const MIN_VOLUME_DB: i16 = -100;
const MAX_VOLUME_DB: i16 = 0;

/// Internal state for USB Audio
pub struct State<'a> {
    control: MaybeUninit<Control<'a>>,
    shared: SharedControl,
}

impl<'a> Default for State<'a> {
    fn default() -> Self {
        Self::new()
    }
}

impl<'a> State<'a> {
    /// Create a new `State`.
    pub fn new() -> Self {
        Self {
            control: MaybeUninit::uninit(),
            shared: SharedControl::default(),
        }
    }
}

pub struct AudioClassOne<'d, D: Driver<'d>> {
    streaming_endpoint: D::EndpointOut,
    feedback_endpoint: D::EndpointIn,
    control: &'d SharedControl,
}

impl<'d, D: Driver<'d>> AudioClassOne<'d, D> {
    /// Creates a new `AudioClassOne` with the provided UsbBus, state, and `max_packet_size` in bytes.
    ///
    /// Use `split` afterwards, in order to option the usable `Stream` and `ControlChanged` instances.
    pub fn new(builder: &mut Builder<'d, D>, state: &'d mut State<'d>, max_packet_size: u16) -> Self {
        let mut func = builder.function(AUDIO_FUNCTION, FUNCTION_SUBCLASS_UNDEFINED, PROTOCOL_NONE);

        // Audio control interface (mandatory) [UAC 4.3.1]
        let mut interface = func.interface();
        let control_interface = interface.interface_number().into();
        let streaming_interface = u8::from(control_interface) + 1;
        let mut alt = interface.alt_setting(USB_AUDIO_CLASS, USB_AUDIOCONTROL_SUBCLASS, PROTOCOL_NONE, None);

        // Terminal topology:
        // Input terminal (receives audio stream) -> Feature Unit (mute and volume) -> Output terminal (e.g. towards speaker)

        // Input Terminal Descriptor [UAC 3.3.2.1]
        // Audio input
        let terminal_type: u16 = TerminalType::UsbStreaming.into();
        let channel_config_left: u16 = ChannelConfig::LeftFront.into();
        let channel_config_right: u16 = ChannelConfig::RightFront.into();
        let channel_config = channel_config_left | channel_config_right;

        let input_terminal_descriptor = [
            INPUT_TERMINAL, // bDescriptorSubtype.
            INPUT_UNIT_ID,  // bTerminalID.
            terminal_type as u8,
            (terminal_type >> 8) as u8, // wTerminalType.
            0x00,                       // bAssocTerminal (none).
            AUDIO_CHANNEL_COUNT as u8,  // bNrChannels.
            channel_config as u8,
            (channel_config >> 8) as u8, // wChannelConfig.
            0x00,                        // iChannelNames (none).
            0x00,                        // iTerminal (none).
        ];

        // Output Terminal Descriptor [UAC 4.3.2.2]
        // Speaker output
        let terminal_type: u16 = TerminalType::OutSpeaker.into();
        let output_terminal_descriptor = [
            OUTPUT_TERMINAL, // bDescriptorSubtype.
            OUTPUT_UNIT_ID,  // bTerminalID.
            terminal_type as u8,
            (terminal_type >> 8) as u8, // wTerminalType.
            0x00,                       // bAssocTerminal (none).
            FEATURE_UNIT_ID,            // bSourceID (the feature unit).
            0x00,                       // iTerminal (none).
        ];

        // Feature Unit Descriptor [UAC 4.3.2.5]
        // Mute and volume control
        let controls = MUTE_CONTROL | VOLUME_CONTROL;
        let feature_unit_descriptor = [
            FEATURE_UNIT,         // bDescriptorSubtype (Feature Unit).
            FEATURE_UNIT_ID,      // bUnitID.
            INPUT_UNIT_ID,        // bSourceID.
            1,                    // bControlSize (one byte per control).
            FU_CONTROL_UNDEFINED, // Master controls.
            controls,             // Channel 0 controls
            controls,             // Channel 1 controls
            0x00,                 // iFeature (none)
        ];

        // Class-specific AC Interface Descriptor [UAC 4.3.2]
        let total_length = 122;

        // FIXME: Calculate the length.
        // (2 + input_terminal_descriptor.len())
        //     + (2 + output_terminal_descriptor.len())
        //     + (2 + feature_unit_descriptor.len())
        //     + 9
        //     + 28
        //     + 18; // The size of the interface descriptor itself.

        let interface_descriptor = [
            HEADER_SUBTYPE, // bDescriptorSubtype (Header)
            ADC_VERSION as u8,
            (ADC_VERSION >> 8) as u8, // bcdADC
            total_length as u8,
            (total_length >> 8) as u8, // wTotalLength
            0x01,                      // bInCollection (1 streaming interface) // FIXME: variable?
            streaming_interface,       // baInterfaceNr
        ];

        alt.descriptor(CS_INTERFACE, &interface_descriptor);
        alt.descriptor(CS_INTERFACE, &input_terminal_descriptor);
        alt.descriptor(CS_INTERFACE, &feature_unit_descriptor);
        alt.descriptor(CS_INTERFACE, &output_terminal_descriptor);

        // Audio streaming interface, zero bandwidth [UAC 4.5.1]
        let mut interface = func.interface();
        let alt = interface.alt_setting(USB_AUDIO_CLASS, USB_AUDIOSTREAMING_SUBCLASS, PROTOCOL_NONE, None);
        drop(alt);

        // Audio streaming interface, operational [UAC 4.5.1]
        let mut alt = interface.alt_setting(USB_AUDIO_CLASS, USB_AUDIOSTREAMING_SUBCLASS, PROTOCOL_NONE, None);

        alt.descriptor(
            CS_INTERFACE,
            &[
                AS_GENERAL,    // bDescriptorSubtype.
                INPUT_UNIT_ID, // bTerminalLink.
                0x00,          // bDelay (none).
                PCM as u8,
                (PCM >> 8) as u8, // wFormatTag (PCM format).
            ],
        );

        alt.descriptor(
            CS_INTERFACE,
            &[
                FORMAT_TYPE,               // bDescriptorSubtype.
                FORMAT_TYPE_I,             // bFormatType.
                AUDIO_CHANNEL_COUNT as u8, // bNrChannels.
                4u8,                       // bSubframeSize. 32 bit.
                32u8,                      // bBitResolution.
                0x01,                      // bSamFreqType (discrete).
                (48000 & 0xFF) as u8,
                ((48000 >> 8) & 0xFF) as u8,
                ((48000 >> 16) & 0xFF) as u8, // Audio sampling frequency, 48 kHz.
            ],
        );

        let streaming_endpoint = alt.alloc_endpoint_out(EndpointType::Isochronous, max_packet_size, 1);
        let feedback_endpoint = alt.alloc_endpoint_in(
            EndpointType::Isochronous,
            4, // Feedback packets are 32 bit, of which 24 are filled.
            1,
        );

        // Write the descriptor for the streaming endpoint, after knowing the address of the feedback endpoint.
        alt.endpoint_descriptor(
            streaming_endpoint.info(),
            SynchronizationType::Asynchronous,
            UsageType::DataEndpoint,
            &[
                0x00,                                 // bRefresh (0).
                feedback_endpoint.info().addr.into(), // bSynchAddress (the feedback endpoint).
            ],
        );

        alt.descriptor(
            CS_ENDPOINT,
            &[
                AS_GENERAL, // bDescriptorSubtype (General).
                0x01,       // bmAttributes - support sampling frequency adjustment.
                0x02,       // bLockDelayUnits (PCM sample count).
                0x0000 as u8,
                (0x0000 >> 8) as u8, // bLockDelay (0).
            ],
        );

        // Write the feedback endpoint descriptor after the streaming endpoint descriptor. This is mandatory.
        alt.endpoint_descriptor(
            feedback_endpoint.info(),
            SynchronizationType::NoSynchronization,
            UsageType::FeedbackEndpoint, // FIXME: check?
            &[
                0x03, // bRefresh.
                0x00, // bSynchAddress (none).
            ],
        );

        // Free up the builder.
        drop(func);
        let control = state.control.write(Control {
            shared: &state.shared,
            streaming_endpoint_address: streaming_endpoint.info().addr.into(),
            control_interface,
        });
        builder.handler(control);

        let control = &state.shared;

        AudioClassOne {
            streaming_endpoint,
            feedback_endpoint,
            control,
        }
    }

    /// Split the class into stream, and control
    ///
    /// Allows concurrent streaming, and watching for control changes.
    pub fn split(self) -> (Stream<'d, D>, ControlChanged<'d>) {
        (
            Stream {
                feedback_endpoint: self.feedback_endpoint,
                streaming_endpoint: self.streaming_endpoint,
            },
            ControlChanged { control: self.control },
        )
    }
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AudioSettings {
    /// Channel mute states.
    pub is_muted: [bool; AUDIO_CHANNEL_COUNT],
    /// Channel volume levels in 8.8 format (in dB).
    pub volume_8q8_db: [i16; AUDIO_CHANNEL_COUNT],
}

impl Default for AudioSettings {
    fn default() -> Self {
        AudioSettings {
            is_muted: [true; AUDIO_CHANNEL_COUNT],
            volume_8q8_db: [0; AUDIO_CHANNEL_COUNT],
        }
    }
}

struct Control<'a> {
    control_interface: InterfaceNumber,
    streaming_endpoint_address: u8,
    shared: &'a SharedControl,
}

/// Shared data between Control and the Audio Class
struct SharedControl {
    audio_settings: CriticalSectionMutex<Cell<AudioSettings>>,

    /// The audio sample rate in Hz.
    sample_rate_hz: AtomicU32,

    waker: RefCell<WakerRegistration>,
    changed: AtomicBool,
}

impl Default for SharedControl {
    fn default() -> Self {
        SharedControl {
            audio_settings: CriticalSectionMutex::new(Cell::new(AudioSettings::default())),
            sample_rate_hz: AtomicU32::new(0),
            waker: RefCell::new(WakerRegistration::new()),
            changed: AtomicBool::new(false),
        }
    }
}

impl SharedControl {
    async fn changed(&self) {
        poll_fn(|context| {
            if self.changed.load(Ordering::Relaxed) {
                self.changed.store(false, Ordering::Relaxed);
                Poll::Ready(())
            } else {
                self.waker.borrow_mut().register(context.waker());
                Poll::Pending
            }
        })
        .await;
    }
}

/// UAC1 stream for reading audio frames, and writing feedback information
///
/// You can obtain a `Stream` with [`AudioClassOne::split`]
pub struct Stream<'d, D: Driver<'d>> {
    streaming_endpoint: D::EndpointOut,
    feedback_endpoint: D::EndpointIn,
}

impl<'d, D: Driver<'d>> Stream<'d, D> {
    /// Writes a single packet into the IN endpoint
    pub async fn write_packet(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        self.feedback_endpoint.write(data).await
    }

    /// Reads a single packet from the OUT endpoint
    pub async fn read_packet(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        self.streaming_endpoint.read(data).await
    }

    /// Waits for the USB host to enable this interface
    pub async fn wait_connection(&mut self) {
        self.streaming_endpoint.wait_enabled().await;
        self.feedback_endpoint.wait_enabled().await;
    }

    // Gets the maximum packet size in bytes for the streaming endpoint
    pub fn max_packet_size(&self) -> u16 {
        self.streaming_endpoint.info().max_packet_size
    }
}

/// UAC1 control status change monitor
///
/// You can obtain a `ControlChanged` with [`AudioClassOne::split`]
pub struct ControlChanged<'d> {
    control: &'d SharedControl,
}

impl<'d> ControlChanged<'d> {
    /// Return the audio channel settings
    pub fn audio_settings(&self) -> AudioSettings {
        let audio_settings = self.control.audio_settings.lock(|x| x.get());

        audio_settings
    }

    /// Return a future for when the control settings change
    pub async fn control_changed(&self) {
        self.control.changed().await;
    }
}

impl<'a> Control<'a> {
    fn shared(&mut self) -> &'a SharedControl {
        self.shared
    }

    fn changed(&mut self) {
        self.shared().changed.store(true, Ordering::Relaxed);
        self.shared().waker.borrow_mut().wake();
    }

    fn interface_set_mute_state(
        &mut self,
        audio_settings: &mut AudioSettings,
        channel_index: u8,
        data: &[u8],
    ) -> OutResponse {
        let mute_state = data[0] != 0;

        match channel_index as usize {
            1..=AUDIO_CHANNEL_COUNT => {
                audio_settings.is_muted[channel_index as usize - 1] = mute_state;
            }
            _ => {
                trace!("Failed to set channel {} mute state: {}", channel_index, mute_state);
                return OutResponse::Rejected;
            }
        }

        trace!("Set channel {} mute state: {}", channel_index, mute_state);
        OutResponse::Accepted
    }

    fn interface_set_volume(
        &mut self,
        audio_settings: &mut AudioSettings,
        channel_index: u8,
        data: &[u8],
    ) -> OutResponse {
        let volume = i16::from_ne_bytes(data[..2].try_into().expect("Failed to read volume."));

        match channel_index as usize {
            1..=AUDIO_CHANNEL_COUNT => {
                audio_settings.volume_8q8_db[channel_index as usize - 1] = volume;
            }
            _ => {
                trace!("Failed to set channel {} volume: {}", channel_index, volume);
                return OutResponse::Rejected;
            }
        }

        trace!("Set channel {} volume: {}", channel_index, volume);
        OutResponse::Accepted
    }

    fn interface_set_request(&mut self, req: control::Request, data: &[u8]) -> Option<OutResponse> {
        let interface_index = req.index as u8;
        let entity_index = (req.index >> 8) as u8;
        let channel_index = req.value as u8;
        let control_unit = (req.value >> 8) as u8;

        if interface_index != self.control_interface.into() {
            trace!("Unhandled interface set request for interface {}", interface_index);
            return None;
        }

        if entity_index != FEATURE_UNIT_ID {
            trace!("Unsupported interface set request for entity {}", entity_index);
            return Some(OutResponse::Rejected);
        }

        if req.request != SET_CUR {
            return Some(OutResponse::Rejected);
        }

        let mut audio_settings = self.shared().audio_settings.lock(|x| x.get());
        let response = match control_unit {
            MUTE_CONTROL => self.interface_set_mute_state(&mut audio_settings, channel_index, data),
            VOLUME_CONTROL => self.interface_set_volume(&mut audio_settings, channel_index, data),
            _ => OutResponse::Rejected,
        };

        if response == OutResponse::Rejected {
            return Some(response);
        }

        // Store updated settings
        self.shared().audio_settings.lock(|x| x.set(audio_settings));

        self.changed();

        Some(OutResponse::Accepted)
    }

    fn endpoint_set_request(&mut self, req: control::Request, data: &[u8]) -> Option<OutResponse> {
        let control_selector = (req.value >> 8) as u8;
        let endpoint_address = req.index as u8;

        if endpoint_address != self.streaming_endpoint_address {
            trace!(
                "Unhandled endpoint set request for endpoint {} and control {} with data {}",
                endpoint_address,
                control_selector,
                data
            );
            return None;
        }

        if control_selector != SAMPLING_FREQ_CONTROL {
            trace!(
                "Unsupported endpoint set request for control selector {}",
                control_selector
            );
            return Some(OutResponse::Rejected);
        }

        let sample_rate_hz: u32 = (data[0] as u32) | (data[1] as u32) << 8 | (data[2] as u32) << 16;
        self.shared().sample_rate_hz.store(sample_rate_hz, Ordering::Relaxed);
        trace!("Set endpoint {} sample rate to {} Hz", endpoint_address, sample_rate_hz);

        Some(OutResponse::Accepted)
    }

    fn interface_get_request<'r>(&'r mut self, req: Request, buf: &'r mut [u8]) -> Option<InResponse<'r>> {
        let interface_index = req.index as u8;
        let entity_index = (req.index >> 8) as u8;
        let channel_index = req.value as u8;
        let control_unit = (req.value >> 8) as u8;

        if interface_index != self.control_interface.into() {
            trace!("Unhandled interface get request for interface {}.", interface_index);
            return None;
        }

        if entity_index != FEATURE_UNIT_ID {
            // Only this function unit can be handled at the moment.
            trace!("Unsupported interface get request for entity {}.", entity_index);
            return Some(InResponse::Rejected);
        }

        let audio_settings = self.shared().audio_settings.lock(|x| x.get());

        match req.request {
            GET_CUR => match control_unit {
                VOLUME_CONTROL => {
                    let volume: i16;

                    match channel_index as usize {
                        1..=AUDIO_CHANNEL_COUNT => volume = audio_settings.volume_8q8_db[channel_index as usize - 1],
                        _ => return Some(InResponse::Rejected),
                    }

                    buf[0] = volume as u8;
                    buf[1] = (volume >> 8) as u8;

                    trace!("Got channel {} volume: {}.", channel_index, volume);
                    return Some(InResponse::Accepted(&buf[..2]));
                }
                MUTE_CONTROL => {
                    let mute_state: bool;

                    match channel_index as usize {
                        1..=AUDIO_CHANNEL_COUNT => mute_state = audio_settings.is_muted[channel_index as usize - 1],
                        _ => return Some(InResponse::Rejected),
                    }

                    buf[0] = mute_state.into();
                    trace!("Got channel {} mute state: {}.", channel_index, mute_state);
                    return Some(InResponse::Accepted(&buf[..1]));
                }
                _ => return Some(InResponse::Rejected),
            },
            GET_MIN => match control_unit {
                VOLUME_CONTROL => {
                    let min_volume = MIN_VOLUME_DB * VOLUME_STEPS_PER_DB;
                    buf[0] = min_volume as u8;
                    buf[1] = (min_volume >> 8) as u8;
                    return Some(InResponse::Accepted(&buf[..2]));
                }
                _ => return Some(InResponse::Rejected),
            },
            GET_MAX => match control_unit {
                VOLUME_CONTROL => {
                    let max_volume = MAX_VOLUME_DB * VOLUME_STEPS_PER_DB;
                    buf[0] = max_volume as u8;
                    buf[1] = (max_volume >> 8) as u8;
                    return Some(InResponse::Accepted(&buf[..2]));
                }
                _ => return Some(InResponse::Rejected),
            },
            GET_RES => match control_unit {
                VOLUME_CONTROL => {
                    buf[0] = VOLUME_STEPS_PER_DB as u8;
                    buf[1] = (VOLUME_STEPS_PER_DB >> 8) as u8;
                    return Some(InResponse::Accepted(&buf[..2]));
                }
                _ => return Some(InResponse::Rejected),
            },
            _ => return Some(InResponse::Rejected),
        }
    }

    fn endpoint_get_request<'r>(&'r mut self, req: Request, buf: &'r mut [u8]) -> Option<InResponse<'r>> {
        let control_selector = (req.value >> 8) as u8;
        let endpoint_address = req.index as u8;

        if endpoint_address != self.streaming_endpoint_address {
            trace!("Unhandled endpoint get request for endpoint {}.", endpoint_address);
            return None;
        }

        if control_selector != SAMPLING_FREQ_CONTROL as u8 {
            trace!(
                "Unsupported endpoint get request for control selector {}.",
                control_selector
            );
            return Some(InResponse::Rejected);
        }

        let sample_rate_hz = self.shared().sample_rate_hz.load(Ordering::Relaxed);

        buf[0] = (sample_rate_hz & 0xFF) as u8;
        buf[1] = ((sample_rate_hz >> 8) & 0xFF) as u8;
        buf[2] = ((sample_rate_hz >> 16) & 0xFF) as u8;

        Some(InResponse::Accepted(&buf[..3]))
    }
}

impl<'d> Handler for Control<'d> {
    fn reset(&mut self) {
        let shared = self.shared();
        shared.audio_settings.lock(|x| x.set(AudioSettings::default()));

        shared.changed.store(true, Ordering::Relaxed);
        shared.waker.borrow_mut().wake();
    }

    // Suspend occurs, for example, when disconnecting the USB cable.
    fn suspended(&mut self, suspended: bool) {
        if suspended {
            self.reset()
        }
    }

    // Handle control set requests.
    fn control_out(&mut self, req: control::Request, data: &[u8]) -> Option<OutResponse> {
        match req.request_type {
            RequestType::Standard => match req.request {
                Request::SET_INTERFACE => Some(OutResponse::Accepted),
                _ => Some(OutResponse::Rejected),
            },
            RequestType::Class => match req.recipient {
                Recipient::Interface => self.interface_set_request(req, data),
                Recipient::Endpoint => self.endpoint_set_request(req, data),
                _ => Some(OutResponse::Rejected),
            },
            _ => None,
        }
    }

    // Handle control get requests.
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        match req.request_type {
            RequestType::Class => match req.recipient {
                Recipient::Interface => self.interface_get_request(req, buf),
                Recipient::Endpoint => self.endpoint_get_request(req, buf),
                _ => None,
            },
            _ => None,
        }
    }
}
