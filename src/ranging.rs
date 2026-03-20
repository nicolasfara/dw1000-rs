//! Caller-driven tag and anchor ranging state machine.

use heapless::Vec;

use crate::config::{RxOptions, TxOptions};
use crate::device::{DeviceIdentity, Peer, PeerSnapshot, RxFrame, ShortAddress, Timestamps};
use crate::error::{Error, ProtocolError};
use crate::protocol::{
    decode_poll_targets, decode_range_timings, encode_discovery_blink, encode_poll,
    encode_poll_ack, encode_range, encode_range_failed, encode_range_report, encode_ranging_init,
    parse_frame, Frame, FrameKind, PollTarget, RangeReportPayload, RangeTiming,
};
use crate::time::DwTime;

const MAX_FRAME_LEN: usize = 127;

/// Role of the ranging state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Role {
    /// Discovery initiator and range calculator.
    Tag,
    /// Discovery responder.
    Anchor,
}

/// User-facing ranging event.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RangingEvent {
    /// A blink frame was observed.
    BlinkReceived(PeerSnapshot),
    /// A peer was added to the table.
    NewPeer(PeerSnapshot),
    /// A peer was pruned due to inactivity.
    PeerInactive(ShortAddress),
    /// A new range measurement is available.
    RangeUpdated(PeerSnapshot),
    /// A ranging-init message was received.
    RangingInitReceived(ShortAddress),
}

/// Ranging configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RangingConfig {
    /// Local identity used for frame addressing.
    pub identity: DeviceIdentity,
    /// Reply delay used by the protocol, in microseconds.
    pub reply_delay_us: u16,
    /// Inactivity timeout, in milliseconds.
    pub reset_period_ms: u32,
    /// Periodic timer tick, in milliseconds.
    pub timer_period_ms: u32,
    /// Optional exponential moving average factor.
    pub range_filter: Option<u16>,
}

impl RangingConfig {
    /// Default tag/anchor configuration.
    pub const fn new(identity: DeviceIdentity) -> Self {
        Self {
            identity,
            reply_delay_us: 7000,
            reset_period_ms: 200,
            timer_period_ms: 80,
            range_filter: None,
        }
    }
}

/// Radio operations required by the ranging node.
pub trait RangingRadio<SpiE, PinE> {
    /// Starts the receiver with the supplied options.
    fn start_receive(&mut self, options: RxOptions) -> Result<(), Error<SpiE, PinE>>;
    /// Transmits a raw frame.
    fn transmit(&mut self, frame: &[u8], options: TxOptions) -> Result<(), Error<SpiE, PinE>>;
    /// Reads an RX frame into the supplied buffer.
    fn read_frame<'a>(&mut self, buffer: &'a mut [u8]) -> Result<RxFrame<'a>, Error<SpiE, PinE>>;
    /// Reads the latest timestamps.
    fn read_timestamps(&mut self) -> Result<Timestamps, Error<SpiE, PinE>>;
    /// Computes a delayed transmit time relative to now.
    fn compute_delayed_time(&mut self, delay: DwTime) -> Result<DwTime, Error<SpiE, PinE>>;
}

impl<SPI, IRQ, RST, SpiE, PinE> RangingRadio<SpiE, PinE> for crate::dw1000::Dw1000<SPI, IRQ, RST>
where
    SPI: embedded_hal::spi::SpiDevice<Error = SpiE>,
    IRQ: embedded_hal::digital::InputPin<Error = PinE>,
    RST: embedded_hal::digital::OutputPin<Error = PinE>,
{
    fn start_receive(&mut self, options: RxOptions) -> Result<(), Error<SpiE, PinE>> {
        crate::dw1000::Dw1000::start_receive(self, options)
    }

    fn transmit(&mut self, frame: &[u8], options: TxOptions) -> Result<(), Error<SpiE, PinE>> {
        crate::dw1000::Dw1000::transmit(self, frame, options)
    }

    fn read_frame<'a>(&mut self, buffer: &'a mut [u8]) -> Result<RxFrame<'a>, Error<SpiE, PinE>> {
        crate::dw1000::Dw1000::read_frame(self, buffer)
    }

    fn read_timestamps(&mut self) -> Result<Timestamps, Error<SpiE, PinE>> {
        crate::dw1000::Dw1000::read_timestamps(self)
    }

    fn compute_delayed_time(&mut self, delay: DwTime) -> Result<DwTime, Error<SpiE, PinE>> {
        crate::dw1000::Dw1000::compute_delayed_time(self, delay)
    }
}

/// Tag/anchor ranging state machine.
#[derive(Debug)]
pub struct RangingNode<const N: usize> {
    role: Role,
    config: RangingConfig,
    peers: Vec<Peer, N>,
    sequence: u8,
    expected: FrameKind,
    last_activity_ms: u32,
    blink_counter: u8,
    last_tick_ms: u32,
    last_tx_kind: Option<FrameKind>,
    last_tx_destination: ShortAddress,
    poll_acknowledged: Vec<ShortAddress, N>,
}

impl<const N: usize> RangingNode<N> {
    /// Creates a ranging node.
    pub fn new(role: Role, config: RangingConfig) -> Self {
        Self {
            role,
            config,
            peers: Vec::new(),
            sequence: 0,
            expected: match role {
                Role::Tag => FrameKind::PollAck,
                Role::Anchor => FrameKind::Poll,
            },
            last_activity_ms: 0,
            blink_counter: 0,
            last_tick_ms: 0,
            last_tx_kind: None,
            last_tx_destination: ShortAddress::BROADCAST,
            poll_acknowledged: Vec::new(),
        }
    }

    /// Starts the node and arms permanent receive mode.
    pub fn start<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        now_ms: u32,
    ) -> Result<(), Error<SpiE, PinE>>
    where
        R: RangingRadio<SpiE, PinE>,
    {
        self.last_activity_ms = now_ms;
        self.last_tick_ms = now_ms;
        radio.start_receive(RxOptions {
            delayed_time: None,
            permanent: true,
        })
    }

    /// Resets protocol state and re-arms permanent receive without pruning peers.
    pub fn recover_link<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        now_ms: u32,
    ) -> Result<(), Error<SpiE, PinE>>
    where
        R: RangingRadio<SpiE, PinE>,
    {
        self.reset_protocol_state();
        self.last_activity_ms = now_ms;
        self.last_tick_ms = now_ms;
        radio.start_receive(RxOptions {
            delayed_time: None,
            permanent: true,
        })
    }

    /// Handles a completed transmission.
    pub fn on_tx_done<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
    ) -> Result<Option<RangingEvent>, Error<SpiE, PinE>>
    where
        R: RangingRadio<SpiE, PinE>,
    {
        let Some(kind) = self.last_tx_kind else {
            return Ok(None);
        };
        let timestamps = radio.read_timestamps()?;
        match (self.role, kind) {
            (Role::Anchor, FrameKind::PollAck) => {
                if let Some(peer) = self.peer_mut(self.last_tx_destination) {
                    peer.poll_ack_sent = timestamps.tx;
                }
            }
            (Role::Tag, FrameKind::Poll) => {
                if self.last_tx_destination.is_broadcast() {
                    for peer in self.peers.iter_mut() {
                        peer.poll_sent = timestamps.tx;
                    }
                } else if let Some(peer) = self.peer_mut(self.last_tx_destination) {
                    peer.poll_sent = timestamps.tx;
                }
            }
            (Role::Tag, FrameKind::Range) => {
                if self.last_tx_destination.is_broadcast() {
                    for peer in self.peers.iter_mut() {
                        peer.range_sent = timestamps.tx;
                    }
                } else if let Some(peer) = self.peer_mut(self.last_tx_destination) {
                    peer.range_sent = timestamps.tx;
                }
            }
            _ => {}
        }
        Ok(None)
    }

    /// Handles an incoming frame.
    pub fn on_rx<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        now_ms: u32,
        buffer: &mut [u8],
    ) -> Result<Option<RangingEvent>, Error<SpiE, PinE>>
    where
        R: RangingRadio<SpiE, PinE>,
    {
        let frame = radio.read_frame(buffer)?;
        self.last_activity_ms = now_ms;
        match parse_frame(frame.bytes)? {
            Frame::Blink(blink) if self.role == Role::Anchor => {
                self.reset_protocol_state();
                let peer = self.ensure_peer(Some(blink.source_eui), blink.source_short, now_ms)?;
                let snapshot = PeerSnapshot::from(peer);
                self.send_ranging_init(radio, blink.source_eui, blink.source_short)?;
                Ok(Some(RangingEvent::BlinkReceived(snapshot)))
            }
            Frame::RangingInit(header) if self.role == Role::Tag => {
                self.reset_protocol_state();
                let peer = self.ensure_peer(None, header.source, now_ms)?;
                Ok(Some(RangingEvent::RangingInitReceived(peer.short_address)))
            }
            Frame::Poll { header, payload } if self.role == Role::Anchor => {
                if self.expected != FrameKind::Poll {
                    self.reset_protocol_state();
                }
                let mut targets = [PollTarget {
                    short_address: ShortAddress::new(0),
                    reply_delay_us: 0,
                }; N];
                let count = decode_poll_targets(payload, &mut targets)?;
                let local = self.config.identity.short_address;
                let Some(target) = targets[..count]
                    .iter()
                    .find(|target| target.short_address == local)
                    .copied()
                else {
                    return Ok(None);
                };
                let peer = self
                    .peer_mut(header.source)
                    .ok_or(ProtocolError::UnknownPeer)?;
                peer.poll_received = frame.timestamp;
                peer.last_activity_ms = now_ms;
                peer.reply_delay_us = target.reply_delay_us;
                let destination = peer.short_address;
                self.expected = FrameKind::Range;
                self.send_poll_ack(radio, destination, target.reply_delay_us)?;
                Ok(None)
            }
            Frame::PollAck { header } if self.role == Role::Tag => {
                if self.expected != FrameKind::PollAck {
                    self.reset_protocol_state();
                    return Ok(None);
                }
                let short_address = {
                    let peer = self
                        .peer_mut(header.source)
                        .ok_or(ProtocolError::UnknownPeer)?;
                    peer.poll_ack_received = frame.timestamp;
                    peer.last_activity_ms = now_ms;
                    peer.short_address
                };
                if !self.poll_acknowledged.contains(&short_address) {
                    self.poll_acknowledged
                        .push(short_address)
                        .map_err(|_| ProtocolError::PeerTableFull)?;
                }
                if self.poll_acknowledged.len() == self.peers.len() {
                    self.expected = FrameKind::RangeReport;
                    self.send_range(radio, None)?;
                }
                Ok(None)
            }
            Frame::Range { header, payload } if self.role == Role::Anchor => {
                if self.expected != FrameKind::Range {
                    self.reset_protocol_state();
                    return Ok(None);
                }
                let mut timings = [RangeTiming {
                    short_address: ShortAddress::new(0),
                    poll_sent: DwTime::zero(),
                    poll_ack_received: DwTime::zero(),
                    range_sent: DwTime::zero(),
                }; N];
                let count = decode_range_timings(payload, &mut timings)?;
                let local = self.config.identity.short_address;
                let Some(timing) = timings[..count]
                    .iter()
                    .find(|timing| timing.short_address == local)
                    .copied()
                else {
                    return Ok(None);
                };
                self.expected = FrameKind::Poll;
                let range_filter = self.config.range_filter;
                let (destination, reply_delay_us, report, snapshot) = {
                    let peer = self
                        .peer_mut(header.source)
                        .ok_or(ProtocolError::UnknownPeer)?;
                    peer.range_received = frame.timestamp;
                    peer.poll_sent = timing.poll_sent;
                    peer.poll_ack_received = timing.poll_ack_received;
                    peer.range_sent = timing.range_sent;
                    peer.metrics = frame.metrics;
                    peer.last_activity_ms = now_ms;
                    let tof = DwTime::asymmetric_tof(
                        peer.poll_sent,
                        peer.poll_received,
                        peer.poll_ack_sent,
                        peer.poll_ack_received,
                        peer.range_sent,
                        peer.range_received,
                    );
                    let distance = filtered_range(range_filter, peer.range_m, tof.as_meters());
                    peer.range_m = distance;
                    (
                        peer.short_address,
                        peer.reply_delay_us,
                        RangeReportPayload {
                            poll_received: peer.poll_received,
                            poll_ack_sent: peer.poll_ack_sent,
                            range_received: peer.range_received,
                            receive_power_dbm: frame.metrics.receive_power_dbm,
                        },
                        PeerSnapshot::from(&*peer),
                    )
                };
                self.send_range_report(radio, destination, report, reply_delay_us)?;
                Ok(Some(RangingEvent::RangeUpdated(snapshot)))
            }
            Frame::RangeReport { header, payload } if self.role == Role::Tag => {
                if self.expected != FrameKind::RangeReport {
                    self.reset_protocol_state();
                    return Ok(None);
                }
                let range_filter = self.config.range_filter;
                let snapshot = {
                    let peer = self
                        .peer_mut(header.source)
                        .ok_or(ProtocolError::UnknownPeer)?;
                    let tof = DwTime::asymmetric_tof(
                        peer.poll_sent,
                        payload.poll_received,
                        payload.poll_ack_sent,
                        peer.poll_ack_received,
                        peer.range_sent,
                        payload.range_received,
                    );
                    peer.range_m = filtered_range(range_filter, peer.range_m, tof.as_meters());
                    peer.metrics.receive_power_dbm = payload.receive_power_dbm;
                    peer.metrics.first_path_power_dbm = frame.metrics.first_path_power_dbm;
                    peer.metrics.quality = frame.metrics.quality;
                    peer.last_activity_ms = now_ms;
                    PeerSnapshot::from(&*peer)
                };
                Ok(Some(RangingEvent::RangeUpdated(snapshot)))
            }
            Frame::RangeFailed { header } if self.role == Role::Tag => {
                self.reset_protocol_state();
                self.peer_mut(header.source)
                    .ok_or(ProtocolError::UnknownPeer)?
                    .last_activity_ms = now_ms;
                Ok(None)
            }
            _ => Ok(None),
        }
    }

    /// Periodic maintenance and discovery tick.
    pub fn tick<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        now_ms: u32,
    ) -> Result<Option<RangingEvent>, Error<SpiE, PinE>>
    where
        R: RangingRadio<SpiE, PinE>,
    {
        if let Some(position) = self.peers.iter().position(|peer| {
            now_ms.saturating_sub(peer.last_activity_ms) > self.config.reset_period_ms
        }) {
            let short = self.peers.swap_remove(position).short_address;
            self.reset_protocol_state();
            return Ok(Some(RangingEvent::PeerInactive(short)));
        }

        if now_ms.saturating_sub(self.last_tick_ms) < self.config.timer_period_ms {
            return Ok(None);
        }
        self.last_tick_ms = now_ms;

        if self.role == Role::Tag {
            if !self.peers.is_empty() && self.blink_counter != 0 {
                self.expected = FrameKind::PollAck;
                self.poll_acknowledged.clear();
                self.send_poll(radio, None)?;
            } else {
                self.send_blink(radio)?;
            }
            self.blink_counter = if self.blink_counter >= 20 {
                0
            } else {
                self.blink_counter + 1
            };
        }
        Ok(None)
    }

    /// Returns the current peer snapshots.
    pub fn peers(&self) -> impl Iterator<Item = PeerSnapshot> + '_ {
        self.peers.iter().map(PeerSnapshot::from)
    }

    /// Returns the role this node was configured with.
    pub const fn role(&self) -> Role {
        self.role
    }

    /// Returns a lightweight snapshot of the most recent transmit state.
    pub fn tx_debug_snapshot(&self) -> (u8, Option<FrameKind>) {
        (self.sequence, self.last_tx_kind)
    }

    fn send_blink<R, SpiE, PinE>(&mut self, radio: &mut R) -> Result<(), Error<SpiE, PinE>>
    where
        R: RangingRadio<SpiE, PinE>,
    {
        let mut frame = [0u8; MAX_FRAME_LEN];
        let len = encode_discovery_blink(
            self.next_sequence(),
            self.config.identity.eui,
            self.config.identity.short_address,
            &mut frame,
        )?;
        self.last_tx_kind = Some(FrameKind::Blink);
        self.last_tx_destination = ShortAddress::BROADCAST;
        radio.transmit(&frame[..len], TxOptions::default())
    }

    fn send_ranging_init<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        destination_eui: crate::device::Eui64,
        destination_short: ShortAddress,
    ) -> Result<(), Error<SpiE, PinE>>
    where
        R: RangingRadio<SpiE, PinE>,
    {
        let mut frame = [0u8; MAX_FRAME_LEN];
        let len = encode_ranging_init(
            self.next_sequence(),
            self.config.identity.short_address,
            destination_eui,
            &mut frame,
        )?;
        self.last_tx_kind = Some(FrameKind::RangingInit);
        self.last_tx_destination = destination_short;
        radio.transmit(&frame[..len], TxOptions::default())
    }

    fn send_poll<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        destination: Option<ShortAddress>,
    ) -> Result<(), Error<SpiE, PinE>>
    where
        R: RangingRadio<SpiE, PinE>,
    {
        let mut frame = [0u8; MAX_FRAME_LEN];
        let mut targets = [PollTarget {
            short_address: ShortAddress::new(0),
            reply_delay_us: 0,
        }; N];
        let destination = destination.unwrap_or(ShortAddress::BROADCAST);
        let count = if destination.is_broadcast() {
            for (index, peer) in self.peers.iter_mut().enumerate() {
                peer.reply_delay_us = ((2 * index + 1) as u16) * self.config.reply_delay_us;
                targets[index] = PollTarget {
                    short_address: peer.short_address,
                    reply_delay_us: peer.reply_delay_us,
                };
            }
            self.peers.len()
        } else {
            let reply_delay = self.config.reply_delay_us;
            let peer = self
                .peer_mut(destination)
                .ok_or(ProtocolError::UnknownPeer)?;
            peer.reply_delay_us = reply_delay;
            targets[0] = PollTarget {
                short_address: peer.short_address,
                reply_delay_us: peer.reply_delay_us,
            };
            1
        };
        let len = encode_poll(
            self.next_sequence(),
            self.config.identity.short_address,
            destination,
            &targets[..count],
            &mut frame,
        )?;
        self.last_tx_kind = Some(FrameKind::Poll);
        self.last_tx_destination = destination;
        radio.transmit(&frame[..len], TxOptions::default())
    }

    fn send_poll_ack<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        destination: ShortAddress,
        reply_delay_us: u16,
    ) -> Result<(), Error<SpiE, PinE>>
    where
        R: RangingRadio<SpiE, PinE>,
    {
        let mut frame = [0u8; MAX_FRAME_LEN];
        let len = encode_poll_ack(
            self.next_sequence(),
            self.config.identity.short_address,
            destination,
            &mut frame,
        )?;
        self.last_tx_kind = Some(FrameKind::PollAck);
        self.last_tx_destination = destination;
        radio.transmit(
            &frame[..len],
            TxOptions {
                delayed_time: Some(DwTime::from_micros(reply_delay_us as f32)),
                wait_for_response: false,
            },
        )
    }

    fn send_range<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        destination: Option<ShortAddress>,
    ) -> Result<(), Error<SpiE, PinE>>
    where
        R: RangingRadio<SpiE, PinE>,
    {
        let mut frame = [0u8; MAX_FRAME_LEN];
        let mut timings = [RangeTiming {
            short_address: ShortAddress::new(0),
            poll_sent: DwTime::zero(),
            poll_ack_received: DwTime::zero(),
            range_sent: DwTime::zero(),
        }; N];
        let destination = destination.unwrap_or(ShortAddress::BROADCAST);
        let delay = DwTime::from_micros(self.config.reply_delay_us as f32);
        let range_sent = radio.compute_delayed_time(delay)?;
        let count = if destination.is_broadcast() {
            for (index, peer) in self.peers.iter().enumerate() {
                timings[index] = RangeTiming {
                    short_address: peer.short_address,
                    poll_sent: peer.poll_sent,
                    poll_ack_received: peer.poll_ack_received,
                    range_sent,
                };
            }
            self.peers.len()
        } else {
            let peer = self.peer(destination).ok_or(ProtocolError::UnknownPeer)?;
            timings[0] = RangeTiming {
                short_address: peer.short_address,
                poll_sent: peer.poll_sent,
                poll_ack_received: peer.poll_ack_received,
                range_sent,
            };
            1
        };
        let len = encode_range(
            self.next_sequence(),
            self.config.identity.short_address,
            destination,
            &timings[..count],
            &mut frame,
        )?;
        self.last_tx_kind = Some(FrameKind::Range);
        self.last_tx_destination = destination;
        radio.transmit(
            &frame[..len],
            TxOptions {
                delayed_time: Some(delay),
                wait_for_response: false,
            },
        )
    }

    fn send_range_report<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        destination: ShortAddress,
        payload: RangeReportPayload,
        reply_delay_us: u16,
    ) -> Result<(), Error<SpiE, PinE>>
    where
        R: RangingRadio<SpiE, PinE>,
    {
        let mut frame = [0u8; MAX_FRAME_LEN];
        let len = encode_range_report(
            self.next_sequence(),
            self.config.identity.short_address,
            destination,
            payload,
            &mut frame,
        )?;
        self.last_tx_kind = Some(FrameKind::RangeReport);
        self.last_tx_destination = destination;
        radio.transmit(
            &frame[..len],
            TxOptions {
                delayed_time: Some(DwTime::from_micros(reply_delay_us as f32)),
                wait_for_response: false,
            },
        )
    }

    #[allow(dead_code)]
    fn send_range_failed<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        destination: ShortAddress,
    ) -> Result<(), Error<SpiE, PinE>>
    where
        R: RangingRadio<SpiE, PinE>,
    {
        let mut frame = [0u8; MAX_FRAME_LEN];
        let len = encode_range_failed(
            self.next_sequence(),
            self.config.identity.short_address,
            destination,
            &mut frame,
        )?;
        self.last_tx_kind = Some(FrameKind::RangeFailed);
        self.last_tx_destination = destination;
        radio.transmit(&frame[..len], TxOptions::default())
    }

    fn ensure_peer(
        &mut self,
        eui: Option<crate::device::Eui64>,
        short_address: ShortAddress,
        now_ms: u32,
    ) -> Result<&Peer, ProtocolError> {
        if let Some(index) = self
            .peers
            .iter()
            .position(|peer| peer.short_address == short_address)
        {
            let peer = &mut self.peers[index];
            peer.last_activity_ms = now_ms;
            if peer.eui.is_none() {
                peer.eui = eui;
            }
            return Ok(peer);
        }
        self.peers
            .push(Peer::new(eui, short_address))
            .map_err(|_| ProtocolError::PeerTableFull)?;
        let last = self.peers.last_mut().ok_or(ProtocolError::PeerTableFull)?;
        last.last_activity_ms = now_ms;
        Ok(last)
    }

    fn peer(&self, short_address: ShortAddress) -> Option<&Peer> {
        self.peers
            .iter()
            .find(|peer| peer.short_address == short_address)
    }

    fn peer_mut(&mut self, short_address: ShortAddress) -> Option<&mut Peer> {
        self.peers
            .iter_mut()
            .find(|peer| peer.short_address == short_address)
    }

    fn next_sequence(&mut self) -> u8 {
        let sequence = self.sequence;
        self.sequence = self.sequence.wrapping_add(1);
        sequence
    }

    fn reset_protocol_state(&mut self) {
        self.expected = match self.role {
            Role::Tag => FrameKind::PollAck,
            Role::Anchor => FrameKind::Poll,
        };
        self.last_tx_kind = None;
        self.last_tx_destination = ShortAddress::BROADCAST;
        self.poll_acknowledged.clear();
    }
}

fn filtered_range(range_filter: Option<u16>, previous: f32, distance: f32) -> f32 {
    if let Some(filter) = range_filter {
        if previous != 0.0 {
            let k = 2.0 / (filter as f32 + 1.0);
            return distance * k + previous * (1.0 - k);
        }
    }
    distance
}

#[cfg(test)]
mod tests {
    use core::convert::Infallible;

    use super::*;
    use crate::device::{DeviceIdentity, Eui64, PanId, ShortAddress};

    #[derive(Default)]
    struct RecordingRadio {
        receive_calls: Vec<RxOptions, 4>,
    }

    impl RangingRadio<Infallible, Infallible> for RecordingRadio {
        fn start_receive(
            &mut self,
            options: RxOptions,
        ) -> Result<(), Error<Infallible, Infallible>> {
            self.receive_calls.push(options).unwrap();
            Ok(())
        }

        fn transmit(
            &mut self,
            _frame: &[u8],
            _options: TxOptions,
        ) -> Result<(), Error<Infallible, Infallible>> {
            unreachable!("recover_link does not transmit")
        }

        fn read_frame<'a>(
            &mut self,
            _buffer: &'a mut [u8],
        ) -> Result<RxFrame<'a>, Error<Infallible, Infallible>> {
            unreachable!("recover_link does not read frames")
        }

        fn read_timestamps(&mut self) -> Result<Timestamps, Error<Infallible, Infallible>> {
            unreachable!("recover_link does not read timestamps")
        }

        fn compute_delayed_time(
            &mut self,
            _delay: DwTime,
        ) -> Result<DwTime, Error<Infallible, Infallible>> {
            unreachable!("recover_link does not compute delayed times")
        }
    }

    fn identity(short: u16, eui: [u8; 8]) -> DeviceIdentity {
        DeviceIdentity::new(
            PanId::new(0xDECA),
            ShortAddress::new(short),
            Eui64::new(eui),
        )
    }

    #[test]
    fn recover_link_preserves_peers_and_resets_protocol_state() {
        let mut node = RangingNode::<4>::new(
            Role::Tag,
            RangingConfig::new(identity(0x1234, [0, 1, 2, 3, 4, 5, 6, 7])),
        );
        let mut radio = RecordingRadio::default();

        node.ensure_peer(None, ShortAddress::new(0x4321), 10)
            .unwrap();
        node.expected = FrameKind::RangeReport;
        node.last_activity_ms = 11;
        node.last_tick_ms = 12;
        node.last_tx_kind = Some(FrameKind::Range);
        node.last_tx_destination = ShortAddress::new(0x4321);
        node.poll_acknowledged
            .push(ShortAddress::new(0x4321))
            .unwrap();

        node.recover_link(&mut radio, 99).unwrap();

        assert_eq!(node.peers.len(), 1);
        assert_eq!(node.expected, FrameKind::PollAck);
        assert_eq!(node.last_activity_ms, 99);
        assert_eq!(node.last_tick_ms, 99);
        assert_eq!(node.last_tx_kind, None);
        assert_eq!(node.last_tx_destination, ShortAddress::BROADCAST);
        assert!(node.poll_acknowledged.is_empty());
        assert_eq!(
            radio.receive_calls.as_slice(),
            &[RxOptions {
                delayed_time: None,
                permanent: true,
            }]
        );
    }
}
