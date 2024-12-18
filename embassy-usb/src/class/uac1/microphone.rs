//! USB Audio Class 1.0 - Microphone device
//!
//! Provides a class with a single audio streaming interface (device to host),
//! that advertises itself as a microphone. Includes explicit sample rate feedback.
//!
//! Various aspects of the audio stream can be configured, for example:
//! - sample rate
//! - sample resolution
//! - audio channel count and assignment

use core::cell::RefCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::WakerRegistration;
use heapless::Vec;

use super::class_codes::*;
use super::terminal_type::TerminalType;
use super::{Channel, ChannelConfig, SampleWidth};
use crate::control::{self, InResponse, OutResponse, Recipient, Request, RequestType};
use crate::descriptor::{SynchronizationType, UsageType};
use crate::driver::{Driver, Endpoint, EndpointError, EndpointIn};
use crate::types::InterfaceNumber;
use crate::{Builder, Handler};

/// Maximum allowed sampling rate (3 bytes) in Hz.
const MAX_SAMPLE_RATE_HZ: u32 = 0x7FFFFF;

/// Arbitrary unique identifier for the input unit.
const INPUT_UNIT_ID: u8 = 0x01;

/// Arbitrary unique identifier for the output unit.
const OUTPUT_UNIT_ID: u8 = 0x02;

// Maximum number of supported discrete sample rates.
const MAX_SAMPLE_RATE_COUNT: usize = 10;

/// Internal state for the USB Audio Class.
pub struct State<'d> {
    control: Option<Control<'d>>,
    shared: SharedControl<'d>,
}

impl<'d> Default for State<'d> {
    fn default() -> Self {
        Self::new()
    }
}

impl<'d> State<'d> {
    /// Create a new `State`.
    pub fn new() -> Self {
        Self {
            control: None,
            shared: SharedControl::default(),
        }
    }
}

/// Implementation of the USB audio class 1.0 Microphone
pub struct Microphone<'d, D: Driver<'d>> {
    phantom: PhantomData<&'d D>,
}

impl<'d, D: Driver<'d>> Microphone<'d, D> {
    /// Creates a new [`Microphone`] device, split into a stream and control change notifier.
    ///
    /// The packet size should be chosen, based on the expected transfer size of samples per (micro)frame.
    /// For example, a stereo stream at 32 bit resolution and 48 kHz sample rate yields packets of 384 byte for
    /// full-speed USB (1 ms frame interval) or 48 byte for high-speed USB (125 us microframe interval).
    ///
    /// # Arguments
    ///
    /// * `builder` - The builder for the class.
    /// * `state` - The internal state of the class.
    /// * `max_packet_size` - The maximum packet size per (micro)frame.
    /// * `resolution` - The audio sample resolution.
    /// * `sample_rates_hz` - The supported sample rates in Hz.
    /// * `channels` - The advertised audio channels (up to 12). Entries must be unique, or this function panics.
    pub fn new(
        builder: &mut Builder<'d, D>,
        state: &'d mut State<'d>,
        max_packet_size: u16,
        resolution: SampleWidth,
        sample_rates_hz: &[u32],
        channels: &'d [Channel],
    ) -> (Stream<'d, D>, /*Feedback<'d, D>,*/ ControlMonitor<'d>) {
        // The class and subclass fields of the IAD aren't required to match the class and subclass fields of
        // the interfaces in the interface collection that the IAD describes. Microsoft recommends that
        // the first interface of the collection has class and subclass fields that match the class and
        // subclass fields of the IAD.
        let mut func = builder.function(USB_AUDIO_CLASS, USB_AUDIOCONTROL_SUBCLASS, PROTOCOL_NONE);

        // Audio control interface (mandatory) [UAC 4.3.1]
        let mut interface = func.interface();
        let control_interface = interface.interface_number().into();
        defmt::info!("control if {:x}", control_interface);
        let streaming_interface = u8::from(control_interface) + 1;
        defmt::info!("stream if {:x}", streaming_interface);
        let mut alt = interface.alt_setting(USB_AUDIO_CLASS, USB_AUDIOCONTROL_SUBCLASS, PROTOCOL_NONE, None);

        // Terminal topology:
        // Input terminal (receives audio stream) -> Feature Unit (mute and volume) -> Output terminal (e.g. towards speaker)

        // =======================================
        // Input Terminal Descriptor [UAC 4.3.2.1]
        // Microphone Input
        let terminal_type: u16 = TerminalType::InMicrophone.into();

        // Assemble channel configuration field
        let mut channel_config: u16 = ChannelConfig::None.into();
        for channel in channels {
            let channel: u16 = channel.get_channel_config().into();

            if channel_config & channel != 0 {
                panic!("Invalid channel config, duplicate channel {}.", channel);
            }
            channel_config |= channel;
        }

        let input_terminal_descriptor = [
            INPUT_TERMINAL,              // bDescriptorSubtype
            INPUT_UNIT_ID,               // bTerminalID
            terminal_type as u8,         //
            (terminal_type >> 8) as u8,  // wTerminalType
            0x00,                        // bAssocTerminal (none)
            channels.len() as u8,        // bNrChannels
            channel_config as u8,        //
            (channel_config >> 8) as u8, // wChannelConfig
            0x00,                        // iChannelNames (none)
            0x00,                        // iTerminal (none)
        ];

        // ========================================
        // Output Terminal Descriptor [UAC 4.3.2.2]
        // Audio Output
        let terminal_type: u16 = TerminalType::UsbStreaming.into();
        let output_terminal_descriptor = [
            OUTPUT_TERMINAL,            // bDescriptorSubtype
            OUTPUT_UNIT_ID,             // bTerminalID
            terminal_type as u8,        //
            (terminal_type >> 8) as u8, // wTerminalType
            0x00,                       // bAssocTerminal (none)
            INPUT_UNIT_ID,              // bSourceID (the input unit)
            0x00,                       // iTerminal (none)
        ];

        // ===============================================
        // Format desciptor [UAC 4.5.3]
        // Used later, for operational streaming interface
        let mut format_descriptor: Vec<u8, { 6 + 3 * MAX_SAMPLE_RATE_COUNT }> = Vec::from_slice(&[
            FORMAT_TYPE,               // bDescriptorSubtype
            FORMAT_TYPE_I,             // bFormatType
            channels.len() as u8,      // bNrChannels
            resolution as u8,          // bSubframeSize
            resolution.in_bit() as u8, // bBitResolution
        ])
        .unwrap();

        format_descriptor.push(sample_rates_hz.len() as u8).unwrap();

        for sample_rate_hz in sample_rates_hz {
            assert!(*sample_rate_hz <= MAX_SAMPLE_RATE_HZ);
            format_descriptor.push((sample_rate_hz & 0xFF) as u8).unwrap();
            format_descriptor.push(((sample_rate_hz >> 8) & 0xFF) as u8).unwrap();
            format_descriptor.push(((sample_rate_hz >> 16) & 0xFF) as u8).unwrap();
        }

        // ==================================================
        // Class-specific AC Interface Descriptor [UAC 4.3.2]
        const DESCRIPTOR_HEADER_SIZE: usize = 2;
        const INTERFACE_DESCRIPTOR_SIZE: usize = 7;

        let mut total_descriptor_length = 0;

        for size in [
            INTERFACE_DESCRIPTOR_SIZE,
            input_terminal_descriptor.len(),
            output_terminal_descriptor.len(),
        ] {
            total_descriptor_length += size + DESCRIPTOR_HEADER_SIZE;
        }

        let interface_descriptor: [u8; INTERFACE_DESCRIPTOR_SIZE] = [
            HEADER_SUBTYPE,                       // bDescriptorSubtype (Header)
            ADC_VERSION as u8,                    //
            (ADC_VERSION >> 8) as u8,             // bcdADC
            total_descriptor_length as u8,        //
            (total_descriptor_length >> 8) as u8, // wTotalLength
            0x01,                                 // bInCollection (1 streaming interface)
            streaming_interface,                  // baInterfaceNr
        ];

        defmt::info!("if desc {:x}", interface_descriptor);
        alt.descriptor(CS_INTERFACE, &interface_descriptor);
        defmt::info!("in desc {:x}", input_terminal_descriptor);
        alt.descriptor(CS_INTERFACE, &input_terminal_descriptor);
        defmt::info!("out desc {:x}", output_terminal_descriptor);
        alt.descriptor(CS_INTERFACE, &output_terminal_descriptor);

        // =====================================================
        // Audio streaming interface, zero-bandwidth [UAC 4.5.1]
        let mut interface = func.interface();
        let alt = interface.alt_setting(USB_AUDIO_CLASS, USB_AUDIOSTREAMING_SUBCLASS, PROTOCOL_NONE, None);
        drop(alt);

        // ==================================================
        // Audio streaming interface, operational [UAC 4.5.1]
        let mut alt = interface.alt_setting(USB_AUDIO_CLASS, USB_AUDIOSTREAMING_SUBCLASS, PROTOCOL_NONE, None);

        alt.descriptor(
            CS_INTERFACE,
            &[
                AS_GENERAL,       // bDescriptorSubtype
                OUTPUT_UNIT_ID,   // bTerminalLink
                0x01,             // bDelay (none)
                PCM as u8,        //
                (PCM >> 8) as u8, // wFormatTag (PCM format)
            ],
        );

        defmt::info!("format desc {=[u8]:#02x}", format_descriptor);
        alt.descriptor(CS_INTERFACE, &format_descriptor);

        let streaming_endpoint = alt.endpoint_isochronous_in(
            max_packet_size,
            1,
            SynchronizationType::NoSynchronization,
            UsageType::DataEndpoint,
            &[
                0x00, // bRefresh (0)
                0x00, // bSynchAddress (unused)
            ],
        );

        alt.descriptor(
            CS_ENDPOINT,
            &[
                AS_GENERAL, // bDescriptorSubtype (General)
                0x00,       // bmAttributes (none)
                0x00,       // bLockDelayUnits (unused)
                0x0000 as u8,
                (0x0000 >> 8) as u8, // wLockDelay (unused)
            ],
        );

        // Free up the builder.
        drop(func);

        // Store channel information
        state.shared.channels = channels;

        state.control = Some(Control {
            shared: &state.shared,
            streaming_endpoint_address: streaming_endpoint.info().addr.into(),
            control_interface_number: control_interface,
        });

        builder.handler(state.control.as_mut().unwrap());

        let control = &state.shared;

        (Stream { streaming_endpoint }, ControlMonitor { shared: control })
    }
}

struct Control<'d> {
    control_interface_number: InterfaceNumber,
    streaming_endpoint_address: u8,
    shared: &'d SharedControl<'d>,
}

/// Shared data between [`Control`] and the [`Speaker`] class.
struct SharedControl<'d> {
    /// Channel assignments.
    channels: &'d [Channel],

    /// The audio sample rate in Hz.
    sample_rate_hz: AtomicU32,

    // Notification mechanism.
    waker: RefCell<WakerRegistration>,
    changed: AtomicBool,
}

impl<'d> Default for SharedControl<'d> {
    fn default() -> Self {
        SharedControl {
            channels: &[],
            sample_rate_hz: AtomicU32::new(0),
            waker: RefCell::new(WakerRegistration::new()),
            changed: AtomicBool::new(false),
        }
    }
}

impl<'d> SharedControl<'d> {
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

/// Used for reading audio frames.
pub struct Stream<'d, D: Driver<'d>> {
    streaming_endpoint: D::EndpointIn,
}

impl<'d, D: Driver<'d>> Stream<'d, D> {
    /// Reads a single packet from the OUT endpoint
    pub async fn write_packet(&mut self, data: &mut [u8]) -> Result<(), EndpointError> {
        self.streaming_endpoint.write(data).await
    }

    /// Waits for the USB host to enable this interface
    pub async fn wait_connection(&mut self) {
        self.streaming_endpoint.wait_enabled().await;
    }
}

/// Used for writing sample rate information over the feedback endpoint.
pub struct Feedback<'d, D: Driver<'d>> {
    feedback_endpoint: D::EndpointIn,
}

impl<'d, D: Driver<'d>> Feedback<'d, D> {
    /// Writes a single packet into the IN endpoint.
    pub async fn write_packet(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        self.feedback_endpoint.write(data).await
    }

    /// Waits for the USB host to enable this interface.
    pub async fn wait_connection(&mut self) {
        self.feedback_endpoint.wait_enabled().await;
    }
}

/// Control status change monitor
///
/// Await [`ControlMonitor::changed`] for being notified of configuration changes. Afterwards, the updated
/// configuration settings can be read with [`ControlMonitor::sample_rate_hz`].
pub struct ControlMonitor<'d> {
    shared: &'d SharedControl<'d>,
}

impl<'d> ControlMonitor<'d> {
    /// Get the streaming endpoint's sample rate in Hz.
    pub fn sample_rate_hz(&self) -> u32 {
        self.shared.sample_rate_hz.load(Ordering::Relaxed)
    }

    /// Return a future for when the control settings change.
    pub async fn changed(&self) {
        self.shared.changed().await;
    }
}

impl<'d> Control<'d> {
    fn changed(&mut self) {
        self.shared.changed.store(true, Ordering::Relaxed);
        self.shared.waker.borrow_mut().wake();
    }

    fn interface_set_request(&mut self, req: control::Request, data: &[u8]) -> Option<OutResponse> {
        let interface_number = req.index as u8;
        let entity_index = (req.index >> 8) as u8;
        let _ = data; // not currently using the data

        if interface_number != self.control_interface_number.into() {
            debug!("Unhandled interface set request for interface {}", interface_number);
            return None;
        }

        // Not currently handling any interface requests for any entity
        debug!("Unsupported interface set request for entity {}.", entity_index);
        return Some(OutResponse::Rejected);
    }

    fn endpoint_set_request(&mut self, req: control::Request, data: &[u8]) -> Option<OutResponse> {
        let control_selector = (req.value >> 8) as u8;
        let endpoint_address = req.index as u8;

        if endpoint_address != self.streaming_endpoint_address {
            debug!(
                "Unhandled endpoint set request for endpoint {} and control {} with data {}",
                endpoint_address, control_selector, data
            );
            return None;
        }

        if control_selector != SAMPLING_FREQ_CONTROL {
            debug!(
                "Unsupported endpoint set request for control selector {}",
                control_selector
            );
            return Some(OutResponse::Rejected);
        }

        let sample_rate_hz: u32 = (data[0] as u32) | (data[1] as u32) << 8 | (data[2] as u32) << 16;
        self.shared.sample_rate_hz.store(sample_rate_hz, Ordering::Relaxed);

        debug!("Set endpoint {} sample rate to {} Hz", endpoint_address, sample_rate_hz);

        self.changed();

        Some(OutResponse::Accepted)
    }

    fn interface_get_request<'r>(&'r mut self, req: Request, buf: &'r mut [u8]) -> Option<InResponse<'r>> {
        let interface_number = req.index as u8;
        let entity_index = (req.index >> 8) as u8;
        let _ = buf; // not currently returning any data

        if interface_number != self.control_interface_number.into() {
            debug!("Unhandled interface get request for interface {}.", interface_number);
            return None;
        }

        // Not currently handling any interface requests for any entity
        debug!("Unsupported interface get request for entity {}.", entity_index);
        return Some(InResponse::Rejected);
    }

    fn endpoint_get_request<'r>(&'r mut self, req: Request, buf: &'r mut [u8]) -> Option<InResponse<'r>> {
        let control_selector = (req.value >> 8) as u8;
        let endpoint_address = req.index as u8;

        if endpoint_address != self.streaming_endpoint_address {
            debug!("Unhandled endpoint get request for endpoint {}.", endpoint_address);
            return None;
        }

        if control_selector != SAMPLING_FREQ_CONTROL as u8 {
            debug!(
                "Unsupported endpoint get request for control selector {}.",
                control_selector
            );
            return Some(InResponse::Rejected);
        }

        let sample_rate_hz = self.shared.sample_rate_hz.load(Ordering::Relaxed);
        debug!("sample_rate_hz: {}", sample_rate_hz);
        buf[0] = (sample_rate_hz & 0xFF) as u8;
        buf[1] = ((sample_rate_hz >> 8) & 0xFF) as u8;
        buf[2] = ((sample_rate_hz >> 16) & 0xFF) as u8;

        Some(InResponse::Accepted(&buf[..3]))
    }
}

impl<'d> Handler for Control<'d> {
    /// Called when the USB device has been enabled or disabled.
    fn enabled(&mut self, enabled: bool) {
        debug!("USB device enabled: {}", enabled);
    }

    /// Called when the host has set the address of the device to `addr`.
    fn addressed(&mut self, addr: u8) {
        debug!("Host set address to: {}", addr);
    }

    /// Called when the host has enabled or disabled the configuration of the device.
    fn configured(&mut self, configured: bool) {
        debug!("USB device configured: {}", configured);
    }

    /// Called when remote wakeup feature is enabled or disabled.
    fn remote_wakeup_enabled(&mut self, enabled: bool) {
        debug!("USB remote wakeup enabled: {}", enabled);
    }

    /// Called when a "set alternate setting" control request is done on the interface.
    fn set_alternate_setting(&mut self, iface: InterfaceNumber, alternate_setting: u8) {
        debug!(
            "USB set interface number {} to alt setting {}.",
            iface, alternate_setting
        );
    }

    /// Called after a USB reset after the bus reset sequence is complete.
    fn reset(&mut self) {
        let shared = self.shared;
        shared.changed.store(true, Ordering::Relaxed);
        shared.waker.borrow_mut().wake();
    }

    /// Called when the bus has entered or exited the suspend state.
    fn suspended(&mut self, suspended: bool) {
        debug!("USB device suspended: {}", suspended);
    }

    // Handle control set requests.
    fn control_out(&mut self, req: control::Request, data: &[u8]) -> Option<OutResponse> {
        match req.request_type {
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
