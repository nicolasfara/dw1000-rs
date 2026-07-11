//! Caller-driven tag and anchor ranging state machine.
//!
//! The protocol logic is implemented as pure state transitions that return a
//! [`Reply`] directive; thin sync/async executors perform the actual radio
//! I/O. This keeps a single logic path shared between the blocking and async
//! drivers.

use heapless::Vec;

use crate::config::{RxOptions, TxOptions};
use crate::device::{DeviceIdentity, Eui64, Peer, PeerSnapshot, RxFrame, ShortAddress, Timestamps};
use crate::error::{Error, ProtocolError};
use crate::protocol::{
    decode_poll_targets, decode_range_timings, encode_discovery_blink, encode_poll,
    encode_poll_ack, encode_range, encode_range_report, encode_ranging_init, encode_schedule_sync,
    parse_frame, BlinkFrame, Frame, FrameHeader, FrameKind, PollTarget, RangeReportPayload,
    RangeTiming, RangingInitFrame, ScheduleSyncFrame,
};
use crate::time::{DelayedTime, DwTime};

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
    /// A coordinator schedule-sync message was received.
    ScheduleSyncReceived(ShortAddress),
    /// A tag exchange did not complete before the configured timeout.
    ExchangeTimedOut,
}

/// Deterministic schedule shared by tags and their anchor coordinator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RangingSchedule {
    /// TDMA slot used by this tag.
    pub tag_slot: u8,
    /// Number of TDMA slots shared by tags on the same PAN.
    pub tag_slot_count: u8,
    /// Duration of a TDMA slot, in milliseconds.
    pub tag_slot_ms: u32,
    /// Maximum time to collect poll acknowledgements or range reports.
    pub session_timeout_ms: u32,
    /// Minimum interval between tag ranging attempts and coordinator syncs.
    pub range_period_ms: u32,
}

impl RangingSchedule {
    /// Default schedule for one tag and up to four anchors.
    pub const fn new() -> Self {
        Self {
            tag_slot: 0,
            tag_slot_count: 1,
            tag_slot_ms: 250,
            session_timeout_ms: 160,
            range_period_ms: 80,
        }
    }
}

impl Default for RangingSchedule {
    fn default() -> Self {
        Self::new()
    }
}

/// Ranging configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RangingConfig {
    /// Local identity used for frame addressing.
    pub identity: DeviceIdentity,
    /// Reply delay used by the protocol, in microseconds.
    pub reply_delay_us: u16,
    /// Delayed reply to a discovery blink, in microseconds.
    ///
    /// Set this to a distinct non-zero value on each anchor to prevent their
    /// ranging-init frames from colliding during multi-anchor discovery.
    pub discovery_reply_delay_us: u16,
    /// Inactivity timeout, in milliseconds.
    pub reset_period_ms: u32,
    /// Periodic timer tick, in milliseconds.
    pub timer_period_ms: u32,
    /// Maximum duration of one poll/range exchange, in milliseconds.
    ///
    /// When it expires the tag resets the exchange and polls again in its
    /// next slot; anchors accept the new poll directly, so no extra blink
    /// is transmitted.
    pub exchange_timeout_ms: u32,
    /// Optional exponential moving average factor.
    pub range_filter: Option<u16>,
    /// Shared coordinator schedule.
    pub schedule: RangingSchedule,
    /// Whether this anchor broadcasts the coordinator schedule.
    pub anchor_is_coordinator: bool,
}

impl RangingConfig {
    /// Default tag/anchor configuration.
    pub const fn new(identity: DeviceIdentity) -> Self {
        Self {
            identity,
            reply_delay_us: 7000,
            discovery_reply_delay_us: 0,
            reset_period_ms: 200,
            timer_period_ms: 80,
            exchange_timeout_ms: 500,
            range_filter: None,
            schedule: RangingSchedule::new(),
            anchor_is_coordinator: false,
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
    /// Schedules a delayed TX/RX activation relative to now.
    fn schedule_delayed(&mut self, delay: DwTime) -> Result<DelayedTime, Error<SpiE, PinE>>;
}

/// Async radio operations required by the ranging node.
#[allow(async_fn_in_trait)]
pub trait AsyncRangingRadio<SpiE, PinE> {
    /// Starts the receiver with the supplied options.
    async fn start_receive(&mut self, options: RxOptions) -> Result<(), Error<SpiE, PinE>>;
    /// Transmits a raw frame.
    async fn transmit(&mut self, frame: &[u8], options: TxOptions)
        -> Result<(), Error<SpiE, PinE>>;
    /// Reads an RX frame into the supplied buffer.
    async fn read_frame<'a>(
        &mut self,
        buffer: &'a mut [u8],
    ) -> Result<RxFrame<'a>, Error<SpiE, PinE>>;
    /// Reads the latest timestamps.
    async fn read_timestamps(&mut self) -> Result<Timestamps, Error<SpiE, PinE>>;
    /// Schedules a delayed TX/RX activation relative to now.
    async fn schedule_delayed(&mut self, delay: DwTime) -> Result<DelayedTime, Error<SpiE, PinE>>;
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

    fn schedule_delayed(&mut self, delay: DwTime) -> Result<DelayedTime, Error<SpiE, PinE>> {
        crate::dw1000::Dw1000::schedule_delayed(self, delay)
    }
}

impl<SPI, IRQ, RST, SpiE, PinE> AsyncRangingRadio<SpiE, PinE>
    for crate::async_dw1000::AsyncDw1000<SPI, IRQ, RST>
where
    SPI: embedded_hal_async::spi::SpiDevice<Error = SpiE>,
    IRQ: embedded_hal::digital::InputPin<Error = PinE>
        + embedded_hal_async::digital::Wait<Error = PinE>,
    RST: embedded_hal::digital::OutputPin<Error = PinE>,
{
    async fn start_receive(&mut self, options: RxOptions) -> Result<(), Error<SpiE, PinE>> {
        crate::async_dw1000::AsyncDw1000::start_receive(self, options).await
    }

    async fn transmit(
        &mut self,
        frame: &[u8],
        options: TxOptions,
    ) -> Result<(), Error<SpiE, PinE>> {
        crate::async_dw1000::AsyncDw1000::transmit(self, frame, options).await
    }

    async fn read_frame<'a>(
        &mut self,
        buffer: &'a mut [u8],
    ) -> Result<RxFrame<'a>, Error<SpiE, PinE>> {
        crate::async_dw1000::AsyncDw1000::read_frame(self, buffer).await
    }

    async fn read_timestamps(&mut self) -> Result<Timestamps, Error<SpiE, PinE>> {
        crate::async_dw1000::AsyncDw1000::read_timestamps(self).await
    }

    async fn schedule_delayed(&mut self, delay: DwTime) -> Result<DelayedTime, Error<SpiE, PinE>> {
        crate::async_dw1000::AsyncDw1000::schedule_delayed(self, delay).await
    }
}

/// Encoded frame ready for transmission.
struct PendingTx {
    frame: [u8; MAX_FRAME_LEN],
    len: usize,
}

/// Transmission directive produced by the pure protocol logic.
enum Reply {
    /// Nothing to send.
    None,
    /// Send a ranging-init frame, optionally after a scheduled delay.
    RangingInit {
        eui: Eui64,
        short: ShortAddress,
        reply_delay_us: u32,
    },
    /// Send a delayed poll-ack frame.
    PollAck {
        destination: ShortAddress,
        reply_delay_us: u32,
    },
    /// Send a delayed broadcast range frame embedding the predicted transmit
    /// timestamp.
    Range { reply_delay_us: u32 },
    /// Send a delayed range report.
    RangeReport {
        destination: ShortAddress,
        payload: RangeReportPayload,
        reply_delay_us: u32,
    },
}

/// Action requested by the periodic tick.
enum TickAction {
    None,
    Blink,
    Poll,
    Range,
    ScheduleSync,
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
    range_reports_received: Vec<ShortAddress, N>,
    exchange_started_ms: u32,
    session_deadline_ms: Option<u32>,
    last_range_start_ms: u32,
    last_schedule_sync_ms: u32,
    schedule_epoch_ms: Option<u32>,
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
                Role::Tag => FrameKind::Blink,
                Role::Anchor => FrameKind::Poll,
            },
            last_activity_ms: 0,
            blink_counter: 0,
            last_tick_ms: 0,
            last_tx_kind: None,
            last_tx_destination: ShortAddress::BROADCAST,
            poll_acknowledged: Vec::new(),
            range_reports_received: Vec::new(),
            exchange_started_ms: 0,
            session_deadline_ms: None,
            last_range_start_ms: 0,
            last_schedule_sync_ms: 0,
            schedule_epoch_ms: None,
        }
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

    // ------------------------------------------------------------------
    // Pure protocol logic shared by the sync and async frontends.
    // ------------------------------------------------------------------

    fn start_session_state(&mut self, now_ms: u32) {
        self.last_activity_ms = now_ms;
        self.last_tick_ms = now_ms;
        if self.role == Role::Anchor && self.config.anchor_is_coordinator {
            self.last_schedule_sync_ms =
                now_ms.wrapping_sub(self.config.schedule.range_period_ms / 2);
        }
    }

    /// Records the actual transmit timestamp for the last sent frame.
    fn record_tx_timestamp(&mut self, tx: DwTime) {
        let Some(kind) = self.last_tx_kind else {
            return;
        };
        match (self.role, kind) {
            (Role::Anchor, FrameKind::PollAck) => {
                if let Some(peer) = self.peer_mut(self.last_tx_destination) {
                    peer.poll_ack_sent = tx;
                }
            }
            (Role::Tag, FrameKind::Poll) => {
                if self.last_tx_destination.is_broadcast() {
                    for peer in self.peers.iter_mut() {
                        peer.poll_sent = tx;
                    }
                } else if let Some(peer) = self.peer_mut(self.last_tx_destination) {
                    peer.poll_sent = tx;
                }
            }
            (Role::Tag, FrameKind::Range) => {
                if self.last_tx_destination.is_broadcast() {
                    for index in 0..self.poll_acknowledged.len() {
                        let short = self.poll_acknowledged[index];
                        if let Some(peer) = self.peer_mut(short) {
                            peer.range_sent = tx;
                        }
                    }
                } else if let Some(peer) = self.peer_mut(self.last_tx_destination) {
                    peer.range_sent = tx;
                }
            }
            _ => {}
        }
    }

    /// Applies an incoming frame to the state machine and returns the reply
    /// directive plus an optional user-facing event.
    fn process_frame(
        &mut self,
        frame: &RxFrame<'_>,
        now_ms: u32,
    ) -> Result<(Reply, Option<RangingEvent>), ProtocolError> {
        self.last_activity_ms = now_ms;
        match parse_frame(frame.bytes)? {
            Frame::Blink(blink) if self.role == Role::Anchor => {
                let Some(snapshot) = self.accept_blink(blink, now_ms)? else {
                    return Ok((Reply::None, None));
                };
                Ok((
                    Reply::RangingInit {
                        eui: blink.source_eui,
                        short: blink.source_short,
                        reply_delay_us: u32::from(self.config.discovery_reply_delay_us),
                    },
                    Some(RangingEvent::BlinkReceived(snapshot)),
                ))
            }
            Frame::ScheduleSync(sync) => {
                let Some(short_address) = self.observe_schedule_sync(sync) else {
                    return Ok((Reply::None, None));
                };
                if self.role == Role::Tag {
                    self.accept_schedule_sync(sync, now_ms);
                }
                Ok((
                    Reply::None,
                    Some(RangingEvent::ScheduleSyncReceived(short_address)),
                ))
            }
            Frame::RangingInit(init) if self.role == Role::Tag => {
                let Some(short_address) = self.accept_ranging_init(init, now_ms)? else {
                    return Ok((Reply::None, None));
                };
                self.reset_protocol_state();
                Ok((
                    Reply::None,
                    Some(RangingEvent::RangingInitReceived(short_address)),
                ))
            }
            Frame::Poll { header, payload } if self.role == Role::Anchor => {
                if !self.matches_short_destination(header.destination, true) {
                    return Ok((Reply::None, None));
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
                    return Ok((Reply::None, None));
                };
                // A poll addressed to this anchor opens a fresh exchange for
                // that tag only. The exchange state lives on the peer, so
                // several tags can interleave exchanges without resetting
                // each other, and an anchor that restarted rejoins from the
                // poll itself instead of waiting for a discovery blink.
                let peer = self.ensure_peer(None, header.source, now_ms)?;
                if !observe_peer_sequence(peer, header.sequence) {
                    return Ok((Reply::None, None));
                }
                peer.poll_received = frame.timestamp;
                peer.poll_received_ms = now_ms;
                peer.awaiting_range = true;
                peer.last_activity_ms = now_ms;
                peer.reply_delay_us = target.reply_delay_us;
                let destination = peer.short_address;
                Ok((
                    Reply::PollAck {
                        destination,
                        reply_delay_us: target.reply_delay_us,
                    },
                    None,
                ))
            }
            Frame::PollAck { header } if self.role == Role::Tag => {
                let Some(peer_short) = self.accept_header(header, false, now_ms)? else {
                    return Ok((Reply::None, None));
                };
                // A stray acknowledgement must not destroy an exchange that
                // has already moved on; the exchange timeout handles stalls.
                if self.expected != FrameKind::PollAck {
                    return Ok((Reply::None, None));
                }
                let short_address = {
                    let Some(peer) = self.peer_mut(peer_short) else {
                        return Ok((Reply::None, None));
                    };
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
                    self.begin_range_report_phase(now_ms);
                    return Ok((
                        Reply::Range {
                            reply_delay_us: u32::from(self.config.reply_delay_us),
                        },
                        None,
                    ));
                }
                Ok((Reply::None, None))
            }
            Frame::Range { header, payload } if self.role == Role::Anchor => {
                let Some(peer_short) = self.accept_header(header, true, now_ms)? else {
                    return Ok((Reply::None, None));
                };
                // Only answer a range that matches a poll this anchor
                // acknowledged recently; anything else is stale and would
                // mix timestamps from two different exchanges.
                let freshness_bound_ms = self.config.schedule.session_timeout_ms.saturating_mul(2);
                let fresh = self
                    .peer_mut(peer_short)
                    .map(|peer| {
                        let fresh = peer.awaiting_range
                            && now_ms.wrapping_sub(peer.poll_received_ms) <= freshness_bound_ms;
                        peer.awaiting_range = false;
                        fresh
                    })
                    .unwrap_or(false);
                if !fresh {
                    return Ok((Reply::None, None));
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
                    // The tag never received this anchor's poll-ack: the
                    // exchange is over for this anchor (the pending flag was
                    // cleared above), so just wait for the next poll.
                    return Ok((Reply::None, None));
                };
                let range_filter = self.config.range_filter;
                let (destination, reply_delay_us, report, snapshot) = {
                    let Some(peer) = self.peer_mut(peer_short) else {
                        return Ok((Reply::None, None));
                    };
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
                Ok((
                    Reply::RangeReport {
                        destination,
                        payload: report,
                        reply_delay_us,
                    },
                    Some(RangingEvent::RangeUpdated(snapshot)),
                ))
            }
            Frame::RangeReport { header, payload } if self.role == Role::Tag => {
                let Some(peer_short) = self.accept_header(header, false, now_ms)? else {
                    return Ok((Reply::None, None));
                };
                // Same as poll-acks: a late report must not reset the state
                // machine while a new exchange is being set up.
                if self.expected != FrameKind::RangeReport {
                    return Ok((Reply::None, None));
                }
                if !self.poll_acknowledged.contains(&peer_short) {
                    return Ok((Reply::None, None));
                }
                let range_filter = self.config.range_filter;
                let snapshot = {
                    let Some(peer) = self.peer_mut(peer_short) else {
                        return Ok((Reply::None, None));
                    };
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
                if !self.range_reports_received.contains(&peer_short) {
                    self.range_reports_received
                        .push(peer_short)
                        .map_err(|_| ProtocolError::PeerTableFull)?;
                }
                if self.range_reports_received.len() == self.poll_acknowledged.len() {
                    self.reset_protocol_state();
                }
                Ok((Reply::None, Some(RangingEvent::RangeUpdated(snapshot))))
            }
            Frame::RangeFailed { header } if self.role == Role::Tag => {
                let Some(peer_short) = self.accept_header(header, false, now_ms)? else {
                    return Ok((Reply::None, None));
                };
                self.reset_protocol_state();
                if let Some(peer) = self.peer_mut(peer_short) {
                    peer.last_activity_ms = now_ms;
                }
                Ok((Reply::None, None))
            }
            _ => Ok((Reply::None, None)),
        }
    }

    /// Advances the periodic maintenance/discovery logic.
    fn process_tick(&mut self, now_ms: u32) -> (Option<RangingEvent>, TickAction) {
        if let Some(position) = self.peers.iter().position(|peer| {
            now_ms.saturating_sub(peer.last_activity_ms) > self.config.reset_period_ms
        }) {
            let short = self.peers.swap_remove(position).short_address;
            self.reset_protocol_state();
            return (Some(RangingEvent::PeerInactive(short)), TickAction::None);
        }

        if self.role == Role::Tag && self.session_expired(now_ms) {
            return match self.expected {
                FrameKind::PollAck if !self.poll_acknowledged.is_empty() => {
                    self.begin_range_report_phase(now_ms);
                    (None, TickAction::Range)
                }
                FrameKind::PollAck | FrameKind::RangeReport => {
                    // Reset without transmitting: the next poll (or the
                    // periodic discovery blink) goes out in this tag's own
                    // slot instead of trampling another tag's exchange.
                    self.reset_protocol_state();
                    (Some(RangingEvent::ExchangeTimedOut), TickAction::None)
                }
                _ => (None, TickAction::None),
            };
        }

        if now_ms.saturating_sub(self.last_tick_ms) < self.config.timer_period_ms {
            return (None, TickAction::None);
        }
        self.last_tick_ms = now_ms;

        if self.role == Role::Anchor {
            // Broadcast the schedule preferably while no exchange is in
            // progress and the channel has been quiet for a tick period, so
            // the sync frame does not collide with poll-acks or reports.
            // These are soft gates only: a stalled exchange (range frame
            // lost, tag switched off) or a busy channel may delay the sync
            // but must never starve it, because tags stay silent until they
            // hear the schedule.
            if self.config.anchor_is_coordinator {
                let since_sync_ms = now_ms.wrapping_sub(self.last_schedule_sync_ms);
                if since_sync_ms >= self.config.schedule.range_period_ms {
                    let overdue =
                        since_sync_ms >= self.config.schedule.range_period_ms.saturating_mul(4);
                    let freshness_bound_ms =
                        self.config.schedule.session_timeout_ms.saturating_mul(2);
                    let exchange_active = self.peers.iter().any(|peer| {
                        peer.awaiting_range
                            && now_ms.wrapping_sub(peer.poll_received_ms) <= freshness_bound_ms
                    });
                    let quiet =
                        now_ms.wrapping_sub(self.last_activity_ms) >= self.config.timer_period_ms;
                    if overdue || (!exchange_active && quiet) {
                        self.last_schedule_sync_ms = now_ms;
                        return (None, TickAction::ScheduleSync);
                    }
                }
            }
            return (None, TickAction::None);
        }

        if self.expected != FrameKind::Blink {
            if now_ms.wrapping_sub(self.exchange_started_ms) >= self.config.exchange_timeout_ms {
                self.reset_protocol_state();
                return (Some(RangingEvent::ExchangeTimedOut), TickAction::None);
            }
            return (None, TickAction::None);
        }

        if !self.tag_slot_is_open(now_ms)
            || now_ms.wrapping_sub(self.last_range_start_ms) < self.config.schedule.range_period_ms
        {
            return (None, TickAction::None);
        }

        self.last_range_start_ms = now_ms;
        let action = if !self.peers.is_empty() && self.blink_counter != 0 {
            self.begin_poll_ack_phase(now_ms);
            TickAction::Poll
        } else {
            TickAction::Blink
        };
        self.blink_counter = if self.blink_counter >= 20 {
            0
        } else {
            self.blink_counter + 1
        };
        (None, action)
    }

    // ------------------------------------------------------------------
    // Frame builders (pure; update the last-TX bookkeeping).
    // ------------------------------------------------------------------

    fn build_blink(&mut self) -> Result<PendingTx, ProtocolError> {
        let mut frame = [0u8; MAX_FRAME_LEN];
        let len = encode_discovery_blink(
            self.next_sequence(),
            self.config.identity.eui,
            self.config.identity.short_address,
            &mut frame,
        )?;
        self.last_tx_kind = Some(FrameKind::Blink);
        self.last_tx_destination = ShortAddress::BROADCAST;
        Ok(PendingTx { frame, len })
    }

    fn build_ranging_init(
        &mut self,
        destination_eui: Eui64,
        destination_short: ShortAddress,
    ) -> Result<PendingTx, ProtocolError> {
        let mut frame = [0u8; MAX_FRAME_LEN];
        let len = encode_ranging_init(
            self.next_sequence(),
            self.config.identity.short_address,
            destination_eui,
            &mut frame,
        )?;
        self.last_tx_kind = Some(FrameKind::RangingInit);
        self.last_tx_destination = destination_short;
        Ok(PendingTx { frame, len })
    }

    fn build_schedule_sync(&mut self, now_ms: u32) -> Result<PendingTx, ProtocolError> {
        let mut frame = [0u8; MAX_FRAME_LEN];
        let len = encode_schedule_sync(
            self.next_sequence(),
            self.config.identity.short_address,
            ShortAddress::BROADCAST,
            now_ms,
            self.config.schedule.tag_slot_count,
            self.config.schedule.tag_slot_ms,
            &mut frame,
        )?;
        self.last_tx_kind = Some(FrameKind::ScheduleSync);
        self.last_tx_destination = ShortAddress::BROADCAST;
        Ok(PendingTx { frame, len })
    }

    /// Builds a broadcast poll and assigns staggered reply slots to peers.
    fn build_poll(&mut self) -> Result<PendingTx, ProtocolError> {
        let mut frame = [0u8; MAX_FRAME_LEN];
        let mut targets = [PollTarget {
            short_address: ShortAddress::new(0),
            reply_delay_us: 0,
        }; N];
        let base_reply_delay_us = self.config.reply_delay_us;
        for (index, peer) in self.peers.iter_mut().enumerate() {
            peer.reply_delay_us = scheduled_reply_delay_us(u32::from(base_reply_delay_us), index)?;
            targets[index] = PollTarget {
                short_address: peer.short_address,
                reply_delay_us: peer.reply_delay_us,
            };
        }
        let count = self.peers.len();
        let len = encode_poll(
            self.next_sequence(),
            self.config.identity.short_address,
            ShortAddress::BROADCAST,
            &targets[..count],
            &mut frame,
        )?;
        self.last_tx_kind = Some(FrameKind::Poll);
        self.last_tx_destination = ShortAddress::BROADCAST;
        Ok(PendingTx { frame, len })
    }

    fn build_poll_ack(&mut self, destination: ShortAddress) -> Result<PendingTx, ProtocolError> {
        let mut frame = [0u8; MAX_FRAME_LEN];
        let len = encode_poll_ack(
            self.next_sequence(),
            self.config.identity.short_address,
            destination,
            &mut frame,
        )?;
        self.last_tx_kind = Some(FrameKind::PollAck);
        self.last_tx_destination = destination;
        Ok(PendingTx { frame, len })
    }

    /// Builds a broadcast range frame. `predicted_tx` is the scheduled
    /// transmit timestamp (DX time + antenna delay) embedded in the payload
    /// so anchors compute with the exact send time.
    fn build_range(&mut self, predicted_tx: DwTime) -> Result<PendingTx, ProtocolError> {
        let mut frame = [0u8; MAX_FRAME_LEN];
        let mut timings = [RangeTiming {
            short_address: ShortAddress::new(0),
            poll_sent: DwTime::zero(),
            poll_ack_received: DwTime::zero(),
            range_sent: DwTime::zero(),
        }; N];
        let mut count = 0;
        for short in self.poll_acknowledged.iter().copied() {
            let Some(peer) = self.peers.iter().find(|peer| peer.short_address == short) else {
                continue;
            };
            timings[count] = RangeTiming {
                short_address: peer.short_address,
                poll_sent: peer.poll_sent,
                poll_ack_received: peer.poll_ack_received,
                range_sent: predicted_tx,
            };
            count += 1;
        }
        let len = encode_range(
            self.next_sequence(),
            self.config.identity.short_address,
            ShortAddress::BROADCAST,
            &timings[..count],
            &mut frame,
        )?;
        self.last_tx_kind = Some(FrameKind::Range);
        self.last_tx_destination = ShortAddress::BROADCAST;
        Ok(PendingTx { frame, len })
    }

    fn build_range_report(
        &mut self,
        destination: ShortAddress,
        payload: RangeReportPayload,
    ) -> Result<PendingTx, ProtocolError> {
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
        Ok(PendingTx { frame, len })
    }

    // ------------------------------------------------------------------
    // Peer bookkeeping.
    // ------------------------------------------------------------------

    fn ensure_peer(
        &mut self,
        eui: Option<Eui64>,
        short_address: ShortAddress,
        now_ms: u32,
    ) -> Result<&mut Peer, ProtocolError> {
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

    fn accept_blink(
        &mut self,
        blink: BlinkFrame,
        now_ms: u32,
    ) -> Result<Option<PeerSnapshot>, ProtocolError> {
        let peer = self.ensure_peer(Some(blink.source_eui), blink.source_short, now_ms)?;
        if !observe_peer_sequence(peer, blink.sequence) {
            return Ok(None);
        }
        // A blink restarts discovery for this tag, so drop any exchange
        // still pending with it.
        peer.awaiting_range = false;
        Ok(Some(PeerSnapshot::from(&*peer)))
    }

    fn accept_ranging_init(
        &mut self,
        init: RangingInitFrame,
        now_ms: u32,
    ) -> Result<Option<ShortAddress>, ProtocolError> {
        if !self.matches_eui_destination(init.destination_eui) {
            return Ok(None);
        }
        let peer = self.ensure_peer(None, init.source_short, now_ms)?;
        if !observe_peer_sequence(peer, init.sequence) {
            return Ok(None);
        }
        Ok(Some(peer.short_address))
    }

    fn accept_header(
        &mut self,
        header: FrameHeader,
        allow_broadcast: bool,
        now_ms: u32,
    ) -> Result<Option<ShortAddress>, ProtocolError> {
        if !self.matches_short_destination(header.destination, allow_broadcast) {
            return Ok(None);
        }
        let Some(peer) = self.peer_mut(header.source) else {
            return Ok(None);
        };
        if !observe_peer_sequence(peer, header.sequence) {
            return Ok(None);
        }
        peer.last_activity_ms = now_ms;
        Ok(Some(peer.short_address))
    }

    fn matches_short_destination(&self, destination: ShortAddress, allow_broadcast: bool) -> bool {
        destination == self.config.identity.short_address
            || (allow_broadcast && destination.is_broadcast())
    }

    fn matches_eui_destination(&self, destination: Eui64) -> bool {
        destination == self.config.identity.eui
    }

    fn observe_schedule_sync(&self, sync: ScheduleSyncFrame) -> Option<ShortAddress> {
        self.matches_short_destination(sync.header.destination, true)
            .then_some(sync.header.source)
    }

    fn accept_schedule_sync(&mut self, sync: ScheduleSyncFrame, now_ms: u32) {
        // Dedup against an existing peer entry, but never create one here:
        // the coordinator may not know this tag yet, and a peer added before
        // discovery would be polled without ever acknowledging.
        if let Some(peer) = self.peer_mut(sync.header.source) {
            peer.last_activity_ms = now_ms;
            if !observe_peer_sequence(peer, sync.header.sequence) {
                return;
            }
        }
        let frame_ms = u32::from(sync.tag_slot_count).saturating_mul(sync.tag_slot_ms);
        self.schedule_epoch_ms = Some(if frame_ms == 0 {
            now_ms
        } else {
            now_ms.wrapping_sub(sync.epoch_ms % frame_ms)
        });
    }

    fn reset_protocol_state(&mut self) {
        self.expected = match self.role {
            Role::Tag => FrameKind::Blink,
            Role::Anchor => FrameKind::Poll,
        };
        for peer in self.peers.iter_mut() {
            peer.awaiting_range = false;
        }
        self.last_tx_kind = None;
        self.last_tx_destination = ShortAddress::BROADCAST;
        self.poll_acknowledged.clear();
        self.range_reports_received.clear();
        self.exchange_started_ms = 0;
        self.session_deadline_ms = None;
    }

    fn begin_poll_ack_phase(&mut self, now_ms: u32) {
        self.expected = FrameKind::PollAck;
        self.poll_acknowledged.clear();
        self.range_reports_received.clear();
        self.exchange_started_ms = now_ms;
        self.session_deadline_ms =
            Some(now_ms.wrapping_add(self.config.schedule.session_timeout_ms));
    }

    fn begin_range_report_phase(&mut self, now_ms: u32) {
        self.expected = FrameKind::RangeReport;
        self.range_reports_received.clear();
        self.session_deadline_ms =
            Some(now_ms.wrapping_add(self.config.schedule.session_timeout_ms));
    }

    fn session_expired(&self, now_ms: u32) -> bool {
        self.session_deadline_ms
            .map(|deadline| now_ms.wrapping_sub(deadline) < 0x8000_0000)
            .unwrap_or(false)
    }

    fn tag_slot_is_open(&self, now_ms: u32) -> bool {
        let count = u32::from(self.config.schedule.tag_slot_count.max(1));
        let slot_ms = self.config.schedule.tag_slot_ms;
        if count <= 1 || slot_ms == 0 {
            return true;
        }
        // Never transmit before the coordinator schedule is known: an
        // unsynced tag would collide with every other tag on the PAN.
        let Some(epoch) = self.schedule_epoch_ms else {
            return false;
        };
        let tag_slot = u32::from(self.config.schedule.tag_slot).min(count - 1);
        let elapsed_in_frame = now_ms.wrapping_sub(epoch) % (count * slot_ms);
        if elapsed_in_frame / slot_ms != tag_slot {
            return false;
        }
        // Only start an exchange while enough of the slot remains for both
        // collection phases, so it cannot spill into the next tag's slot.
        // Slots too short to ever satisfy the budget degrade to the bare
        // slot check instead of silencing the tag entirely.
        let exchange_budget_ms = self.config.schedule.session_timeout_ms.saturating_mul(2);
        let remaining_ms = slot_ms - elapsed_in_frame % slot_ms;
        exchange_budget_ms >= slot_ms || remaining_ms >= exchange_budget_ms
    }
}

// ----------------------------------------------------------------------
// Blocking executor.
// ----------------------------------------------------------------------

impl<const N: usize> RangingNode<N> {
    /// Starts the node and arms permanent receive mode.
    pub fn start<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        now_ms: u32,
    ) -> Result<(), Error<SpiE, PinE>>
    where
        R: RangingRadio<SpiE, PinE>,
    {
        self.start_session_state(now_ms);
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
        self.start_session_state(now_ms);
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
        if self.last_tx_kind.is_none() {
            return Ok(None);
        }
        let timestamps = radio.read_timestamps()?;
        self.record_tx_timestamp(timestamps.tx);
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
        let (reply, event) = self.process_frame(&frame, now_ms)?;
        self.execute_reply(radio, reply)?;
        Ok(event)
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
        let (event, action) = self.process_tick(now_ms);
        match action {
            TickAction::None => {}
            TickAction::Blink => {
                let tx = self.build_blink()?;
                radio.transmit(&tx.frame[..tx.len], TxOptions::default())?;
            }
            TickAction::Poll => {
                let tx = self.build_poll()?;
                radio.transmit(&tx.frame[..tx.len], TxOptions::default())?;
            }
            TickAction::Range => self.execute_reply(
                radio,
                Reply::Range {
                    reply_delay_us: u32::from(self.config.reply_delay_us),
                },
            )?,
            TickAction::ScheduleSync => {
                let tx = self.build_schedule_sync(now_ms)?;
                radio.transmit(&tx.frame[..tx.len], TxOptions::default())?;
            }
        }
        Ok(event)
    }

    fn execute_reply<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        reply: Reply,
    ) -> Result<(), Error<SpiE, PinE>>
    where
        R: RangingRadio<SpiE, PinE>,
    {
        match reply {
            Reply::None => Ok(()),
            Reply::RangingInit {
                eui,
                short,
                reply_delay_us,
            } => {
                let tx = self.build_ranging_init(eui, short)?;
                if reply_delay_us == 0 {
                    radio.transmit(&tx.frame[..tx.len], TxOptions::default())
                } else {
                    let scheduled =
                        radio.schedule_delayed(DwTime::from_micros(reply_delay_us as f32))?;
                    radio.transmit(&tx.frame[..tx.len], delayed_tx_options(scheduled))
                }
            }
            Reply::PollAck {
                destination,
                reply_delay_us,
            } => {
                let scheduled =
                    radio.schedule_delayed(DwTime::from_micros(reply_delay_us as f32))?;
                let tx = self.build_poll_ack(destination)?;
                radio.transmit(&tx.frame[..tx.len], delayed_tx_options(scheduled))
            }
            Reply::Range { reply_delay_us } => {
                let scheduled =
                    radio.schedule_delayed(DwTime::from_micros(reply_delay_us as f32))?;
                let tx = self.build_range(scheduled.predicted_tx_timestamp())?;
                radio.transmit(&tx.frame[..tx.len], delayed_tx_options(scheduled))
            }
            Reply::RangeReport {
                destination,
                payload,
                reply_delay_us,
            } => {
                let scheduled =
                    radio.schedule_delayed(DwTime::from_micros(reply_delay_us as f32))?;
                let tx = self.build_range_report(destination, payload)?;
                radio.transmit(&tx.frame[..tx.len], delayed_tx_options(scheduled))
            }
        }
    }
}

// ----------------------------------------------------------------------
// Async executor.
// ----------------------------------------------------------------------

impl<const N: usize> RangingNode<N> {
    /// Starts the node and arms permanent receive mode on an async radio.
    pub async fn start_async<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        now_ms: u32,
    ) -> Result<(), Error<SpiE, PinE>>
    where
        R: AsyncRangingRadio<SpiE, PinE>,
    {
        self.start_session_state(now_ms);
        radio
            .start_receive(RxOptions {
                delayed_time: None,
                permanent: true,
            })
            .await
    }

    /// Resets protocol state and re-arms permanent receive on an async radio.
    pub async fn recover_link_async<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        now_ms: u32,
    ) -> Result<(), Error<SpiE, PinE>>
    where
        R: AsyncRangingRadio<SpiE, PinE>,
    {
        self.reset_protocol_state();
        self.start_session_state(now_ms);
        radio
            .start_receive(RxOptions {
                delayed_time: None,
                permanent: true,
            })
            .await
    }

    /// Handles a completed transmission for an async radio.
    pub async fn on_tx_done_async<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
    ) -> Result<Option<RangingEvent>, Error<SpiE, PinE>>
    where
        R: AsyncRangingRadio<SpiE, PinE>,
    {
        if self.last_tx_kind.is_none() {
            return Ok(None);
        }
        let timestamps = radio.read_timestamps().await?;
        self.record_tx_timestamp(timestamps.tx);
        Ok(None)
    }

    /// Handles an incoming frame for an async radio.
    pub async fn on_rx_async<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        now_ms: u32,
        buffer: &mut [u8],
    ) -> Result<Option<RangingEvent>, Error<SpiE, PinE>>
    where
        R: AsyncRangingRadio<SpiE, PinE>,
    {
        let frame = radio.read_frame(buffer).await?;
        let (reply, event) = self.process_frame(&frame, now_ms)?;
        self.execute_reply_async(radio, reply).await?;
        Ok(event)
    }

    /// Periodic maintenance and discovery tick for an async radio.
    pub async fn tick_async<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        now_ms: u32,
    ) -> Result<Option<RangingEvent>, Error<SpiE, PinE>>
    where
        R: AsyncRangingRadio<SpiE, PinE>,
    {
        let (event, action) = self.process_tick(now_ms);
        match action {
            TickAction::None => {}
            TickAction::Blink => {
                let tx = self.build_blink()?;
                radio
                    .transmit(&tx.frame[..tx.len], TxOptions::default())
                    .await?;
            }
            TickAction::Poll => {
                let tx = self.build_poll()?;
                radio
                    .transmit(&tx.frame[..tx.len], TxOptions::default())
                    .await?;
            }
            TickAction::Range => {
                self.execute_reply_async(
                    radio,
                    Reply::Range {
                        reply_delay_us: u32::from(self.config.reply_delay_us),
                    },
                )
                .await?;
            }
            TickAction::ScheduleSync => {
                let tx = self.build_schedule_sync(now_ms)?;
                radio
                    .transmit(&tx.frame[..tx.len], TxOptions::default())
                    .await?;
            }
        }
        Ok(event)
    }

    async fn execute_reply_async<R, SpiE, PinE>(
        &mut self,
        radio: &mut R,
        reply: Reply,
    ) -> Result<(), Error<SpiE, PinE>>
    where
        R: AsyncRangingRadio<SpiE, PinE>,
    {
        match reply {
            Reply::None => Ok(()),
            Reply::RangingInit {
                eui,
                short,
                reply_delay_us,
            } => {
                let tx = self.build_ranging_init(eui, short)?;
                if reply_delay_us == 0 {
                    radio
                        .transmit(&tx.frame[..tx.len], TxOptions::default())
                        .await
                } else {
                    let scheduled = radio
                        .schedule_delayed(DwTime::from_micros(reply_delay_us as f32))
                        .await?;
                    radio
                        .transmit(&tx.frame[..tx.len], delayed_tx_options(scheduled))
                        .await
                }
            }
            Reply::PollAck {
                destination,
                reply_delay_us,
            } => {
                let scheduled = radio
                    .schedule_delayed(DwTime::from_micros(reply_delay_us as f32))
                    .await?;
                let tx = self.build_poll_ack(destination)?;
                radio
                    .transmit(&tx.frame[..tx.len], delayed_tx_options(scheduled))
                    .await
            }
            Reply::Range { reply_delay_us } => {
                let scheduled = radio
                    .schedule_delayed(DwTime::from_micros(reply_delay_us as f32))
                    .await?;
                let tx = self.build_range(scheduled.predicted_tx_timestamp())?;
                radio
                    .transmit(&tx.frame[..tx.len], delayed_tx_options(scheduled))
                    .await
            }
            Reply::RangeReport {
                destination,
                payload,
                reply_delay_us,
            } => {
                let scheduled = radio
                    .schedule_delayed(DwTime::from_micros(reply_delay_us as f32))
                    .await?;
                let tx = self.build_range_report(destination, payload)?;
                radio
                    .transmit(&tx.frame[..tx.len], delayed_tx_options(scheduled))
                    .await
            }
        }
    }
}

const fn delayed_tx_options(scheduled: DelayedTime) -> TxOptions {
    TxOptions {
        delayed_time: Some(scheduled),
        wait_for_response: false,
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

fn scheduled_reply_delay_us(base_reply_delay_us: u32, index: usize) -> Result<u32, ProtocolError> {
    let slot = index
        .checked_mul(2)
        .and_then(|value| value.checked_add(1))
        .ok_or(ProtocolError::ReplyDelayOverflow)?;
    let multiplier = u32::try_from(slot).map_err(|_| ProtocolError::ReplyDelayOverflow)?;
    base_reply_delay_us
        .checked_mul(multiplier)
        .ok_or(ProtocolError::ReplyDelayOverflow)
}

fn observe_peer_sequence(peer: &mut Peer, sequence: u8) -> bool {
    let is_fresh = match peer.last_sequence {
        None => true,
        Some(last_sequence) => {
            let delta = sequence.wrapping_sub(last_sequence);
            delta != 0 && delta < 128
        }
    };
    if is_fresh {
        peer.last_sequence = Some(sequence);
    }
    is_fresh
}
