use defmt::{info, warn};
use dw1000_rs::registers::status;
use dw1000_rs::{
    AntennaDelay, DeviceIdentity, Error, OperatingMode, RadioConfig, RangingConfig, RangingEvent,
    RangingNode, Role, RxError, SysStatus,
};
use dw1000_rs::protocol::FrameKind;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Ticker};

use crate::board::Board;
use crate::nodes::NodeConfig;
use crate::{
    DW1000_LINK_RECOVERY_TIMEOUT_MS, PEER_INACTIVITY_TIMEOUT_MS, RANGING_EXCHANGE_TIMEOUT_MS,
    RANGING_PERIOD_MS, RANGING_REPLY_DELAY_US, RANGING_SESSION_TIMEOUT_MS, RX_BUFFER_LEN,
    TAG_SLOT_COUNT, TAG_SLOT_MS,
};

const ANCHOR_HEARTBEAT_PERIOD_MS: u32 = 1_000;

pub(crate) struct RangingApp<const N: usize> {
    board: Board,
    role: Role,
    identity: DeviceIdentity,
    anchor_is_coordinator: bool,
    radio_config: RadioConfig,
    node: RangingNode<N>,
    rx_buffer: [u8; RX_BUFFER_LEN],
    ticker: Ticker,
    last_radio_activity_ms: u32,
    last_range_update_ms: u32,
    last_anchor_heartbeat_ms: u32,
    last_schedule_sync_source: Option<dw1000_rs::ShortAddress>,
    last_schedule_sync_ms: Option<u32>,
    schedule_sync_count: u32,
    schedule_sync_tx_count: u32,
    rx_event_count: u32,
}

pub(crate) struct AppInitError {
    board: Board,
    message: &'static str,
}

struct RecoverReason {
    now_ms: u32,
    message: &'static str,
}

// These are the IRQ causes for which `read_frame` can either return a frame
// or report a concrete RX error. Detection-progress bits are intentionally
// excluded because a frame is not ready to read yet.
const RX_WORK_EVENTS: SysStatus = SysStatus(
    status::RX_FRAME_READY.0
        | status::RX_FRAME_GOOD.0
        | status::ALL_RX_ERRORS.0
        | status::ALL_RX_TIMEOUTS.0,
);

enum StepOutcome {
    Idle,
    RadioActivity,
    RangeUpdated,
}

impl StepOutcome {
    fn merge(self, other: Self) -> Self {
        match (self, other) {
            (Self::RangeUpdated, _) | (_, Self::RangeUpdated) => Self::RangeUpdated,
            (Self::RadioActivity, _) | (_, Self::RadioActivity) => Self::RadioActivity,
            _ => Self::Idle,
        }
    }
}

impl<const N: usize> RangingApp<N> {
    pub(crate) async fn new(
        mut board: Board,
        role: Role,
        node_config: NodeConfig,
    ) -> Result<Self, AppInitError> {
        let identity = node_config.identity;
        let antenna_delay = node_config.antenna_delay;
        board.announce_start(role, identity, antenna_delay);

        let radio_config = default_radio_config(identity, antenna_delay);
        let ranging_config = default_ranging_config(&node_config);
        let mut node = RangingNode::<N>::new(role, ranging_config);

        if board
            .radio
            .init(&mut embassy_time::Delay, &radio_config)
            .await
            .is_err()
        {
            return Err(AppInitError {
                board,
                message: "dw1000 init failed",
            });
        }
        if board.radio.enable_leds().await.is_err() {
            return Err(AppInitError {
                board,
                message: "failed to enable DW1000 RX/TX leds",
            });
        }
        let now = now_ms();
        if node.start_async(&mut board.radio, now).await.is_err() {
            return Err(AppInitError {
                board,
                message: "ranging start failed",
            });
        }
        if role == Role::Anchor {
            info!(
                "anchor coordinator={=bool}",
                node_config.anchor_is_coordinator
            );
        }

        if role == Role::Anchor {
            info!(
                "anchor listening short={=u16} coordinator={=bool} slot={=u8}",
                identity.short_address.raw(),
                anchor_is_coordinator,
                schedule.anchor_slot
            );
        }

        Ok(Self {
            board,
            role,
            identity,
            anchor_is_coordinator: node_config.anchor_is_coordinator,
            radio_config,
            node,
            rx_buffer: [0; RX_BUFFER_LEN],
            ticker: Ticker::every(Duration::from_millis(ranging_config.timer_period_ms as u64)),
            last_radio_activity_ms: now,
            last_range_update_ms: now,
            last_anchor_heartbeat_ms: now,
            last_schedule_sync_source: None,
            last_schedule_sync_ms: None,
            schedule_sync_count: 0,
            schedule_sync_tx_count: 0,
            rx_event_count: 0,
        })
    }

    pub(crate) async fn run_forever(mut self) -> ! {
        loop {
            self.step().await;
            self.recover_stalled_link_if_needed().await;
        }
    }

    async fn step(&mut self) {
        // Capture the timestamp after the await: the select can park for a
        // whole ticker period, and TDMA slot scheduling needs a fresh clock.
        match select(self.ticker.next(), self.board.radio.wait_for_irq()).await {
            Either::First(_) => {
                let now = now_ms();
                self.handle_tick(now).await;
            }
            Either::Second(irq_result) => {
                let now = now_ms();
                self.handle_irq(irq_result, now).await;
            }
        }
    }

    async fn handle_tick(&mut self, now: u32) {
        let tx_before = self.node.tx_debug_snapshot();
        match self.node.tick_async(&mut self.board.radio, now).await {
            Ok(event) => {
                let tx_after = self.node.tx_debug_snapshot();
                let mut outcome = StepOutcome::Idle;
                if tx_after != tx_before || event.is_some() {
                    outcome = StepOutcome::RadioActivity;
                }
                if self.role == Role::Anchor
                    && self.anchor_is_coordinator
                    && tx_after.0 != tx_before.0
                    && tx_after.1 == Some(FrameKind::ScheduleSync)
                {
                    self.schedule_sync_tx_count = self.schedule_sync_tx_count.wrapping_add(1);
                }
                if self.handle_event(event, now) {
                    outcome = StepOutcome::RangeUpdated;
                }
                self.record_outcome(now, outcome);
                self.log_anchor_heartbeat(now);
            }
            Err(Error::DelayedSendTooLate) => {
                // The driver aborted the send and re-armed receive; the next
                // tick retries without a full radio recovery.
                warn!("delayed send scheduled too late");
                self.record_outcome(now, StepOutcome::RadioActivity);
            }
            Err(_) => self.recover(RecoverReason::new(now, "tick failed")).await,
        }
    }

    async fn handle_irq(
        &mut self,
        irq_result: Result<
            (),
            Error<impl embedded_hal::spi::Error, impl embedded_hal::digital::Error>,
        >,
        now: u32,
    ) {
        if irq_result.is_err() {
            self.recover(RecoverReason::new(now, "irq wait failed"))
                .await;
            return;
        }

        if let Err(reason) = self.drain_irq().await {
            self.recover(reason).await;
        }
    }

    async fn drain_irq(&mut self) -> Result<(), RecoverReason> {
        loop {
            let now = now_ms();
            let irq_status = self
                .board
                .radio
                .read_sys_status()
                .await
                .map_err(|_| RecoverReason::new(now, "status read failed"))?;

            let mut outcome = StepOutcome::Idle;

            if irq_status.contains(status::TX_FRAME_SENT) {
                outcome = outcome.merge(StepOutcome::RadioActivity);
                let event = self
                    .node
                    .on_tx_done_async(&mut self.board.radio)
                    .await
                    .map_err(|_| RecoverReason::new(now, "tx handling failed"))?;
                if self.handle_event(event, now) {
                    outcome = StepOutcome::RangeUpdated;
                }
            }

            if has_rx_work(irq_status) {
                self.rx_event_count = self.rx_event_count.wrapping_add(1);
                match self
                    .node
                    .on_rx_async(&mut self.board.radio, now, &mut self.rx_buffer)
                    .await
                {
                    Ok(event) => {
                        outcome = outcome.merge(StepOutcome::RadioActivity);
                        if self.handle_event(event, now) {
                            outcome = StepOutcome::RangeUpdated;
                        }
                    }
                    Err(Error::Receive(error)) => {
                        outcome = outcome.merge(StepOutcome::RadioActivity);
                        warn!("rx error: {=str}", rx_error_label(error));
                    }
                    Err(Error::Protocol(_)) => {
                        outcome = outcome.merge(StepOutcome::RadioActivity);
                        warn!("protocol error");
                    }
                    Err(Error::DelayedSendTooLate) => {
                        // The driver aborted the reply and re-armed receive;
                        // the exchange timeout takes care of the retry.
                        outcome = outcome.merge(StepOutcome::RadioActivity);
                        warn!("delayed reply scheduled too late");
                    }
                    Err(_) => return Err(RecoverReason::new(now, "rx handling failed")),
                }
            }

            self.record_outcome(now, outcome);

            if irq_status != SysStatus::EMPTY
                && self.board.radio.clear_events(irq_status).await.is_err()
            {
                return Err(RecoverReason::new(now, "clear events failed"));
            }

            match self.board.radio.irq_asserted() {
                Ok(true) => {}
                Ok(false) => return Ok(()),
                Err(_) => return Err(RecoverReason::new(now, "irq state read failed")),
            }
        }
    }

    async fn recover(&mut self, reason: RecoverReason) {
        self.board
            .recover_or_fault(
                &mut self.node,
                &self.radio_config,
                reason.now_ms,
                reason.message,
            )
            .await;
        self.last_radio_activity_ms = reason.now_ms;
        self.last_range_update_ms = reason.now_ms;
    }

    async fn recover_stalled_link_if_needed(&mut self) {
        let now = now_ms();
        let radio_gap_ms = now.wrapping_sub(self.last_radio_activity_ms);
        let range_gap_ms = now.wrapping_sub(self.last_range_update_ms);
        if self.node.peers().next().is_some()
            && (radio_gap_ms >= DW1000_LINK_RECOVERY_TIMEOUT_MS
                || range_gap_ms >= DW1000_LINK_RECOVERY_TIMEOUT_MS)
        {
            warn!(
                "recovering link radio_gap={=u32} range_gap={=u32}",
                radio_gap_ms, range_gap_ms
            );
            self.recover(RecoverReason::new(now, "stalled link")).await;
        }
    }

    fn record_outcome(&mut self, now: u32, outcome: StepOutcome) {
        match outcome {
            StepOutcome::Idle => {}
            StepOutcome::RadioActivity => {
                self.last_radio_activity_ms = now;
            }
            StepOutcome::RangeUpdated => {
                self.last_radio_activity_ms = now;
                self.last_range_update_ms = now;
            }
        }
    }

    fn log_anchor_heartbeat(&mut self, now: u32) {
        if self.role != Role::Anchor
            || now.wrapping_sub(self.last_anchor_heartbeat_ms) < ANCHOR_HEARTBEAT_PERIOD_MS
        {
            return;
        }

        self.last_anchor_heartbeat_ms = now;
        let sync_age_ms = self
            .last_schedule_sync_ms
            .map(|last_sync| now.wrapping_sub(last_sync))
            .unwrap_or(u32::MAX);
        info!(
            "anchor alive short={=u16} coordinator={=bool} sync_tx={=u32} sync_rx={=u32} sync_age_ms={=u32} rx_events={=u32}",
            self.identity.short_address.raw(),
            self.anchor_is_coordinator,
            self.schedule_sync_tx_count,
            self.schedule_sync_count,
            sync_age_ms,
            self.rx_event_count
        );
    }

    fn handle_event(&mut self, event: Option<RangingEvent>, now: u32) -> bool {
        match event {
            Some(RangingEvent::BlinkReceived(snapshot)) => {
                info!(
                    "{=str} blink from {=u16}",
                    role_label(self.role),
                    snapshot.short_address.raw()
                );
                false
            }
            Some(RangingEvent::NewPeer(snapshot)) => {
                info!(
                    "{=str} peer {=u16}",
                    role_label(self.role),
                    snapshot.short_address.raw()
                );
                false
            }
            Some(RangingEvent::PeerInactive(short)) => {
                warn!(
                    "{=str} peer inactive {=u16}",
                    role_label(self.role),
                    short.raw()
                );
                false
            }
            Some(RangingEvent::RangeUpdated(snapshot)) => {
                if self.role == Role::Tag {
                    info!(
                        "uwb_range tag={=u16} anchor={=u16} range_m={=f32} quality={=f32}",
                        self.identity.short_address.raw(),
                        snapshot.short_address.raw(),
                        snapshot.range_m,
                        snapshot.quality
                    );
                }
                true
            }
            Some(RangingEvent::RangingInitReceived(short)) => {
                info!("tag ranging init from {=u16}", short.raw());
                false
            }
            Some(RangingEvent::ScheduleSyncReceived(short)) => {
                self.schedule_sync_count = self.schedule_sync_count.wrapping_add(1);
                self.last_schedule_sync_ms = Some(now);
                if self.last_schedule_sync_source != Some(short) {
                    self.last_schedule_sync_source = Some(short);
                    info!(
                        "{=str} coordinator sync from {=u16}",
                        role_label(self.role),
                        short.raw()
                    );
                }
                false
            }
            Some(RangingEvent::ExchangeTimedOut) => {
                warn!("{=str} ranging exchange timed out", role_label(self.role));
                false
            }
            None => false,
        }
    }
}

impl AppInitError {
    pub(crate) async fn fault_loop(mut self) -> ! {
        self.board.fault_loop(self.message).await
    }
}

impl RecoverReason {
    const fn new(now_ms: u32, message: &'static str) -> Self {
        Self { now_ms, message }
    }
}

fn default_radio_config(identity: DeviceIdentity, antenna_delay: AntennaDelay) -> RadioConfig {
    let mut config = RadioConfig::from_mode(identity, OperatingMode::LongDataRangeAccuracy);
    config.antenna_delay = antenna_delay;
    config
}

fn default_ranging_config(node_config: &NodeConfig) -> RangingConfig {
    let mut config = RangingConfig::new(node_config.identity);
    config.reply_delay_us = RANGING_REPLY_DELAY_US;
    config.discovery_reply_delay_us = node_config.discovery_reply_delay_us;
    config.exchange_timeout_ms = RANGING_EXCHANGE_TIMEOUT_MS;
    config.reset_period_ms = PEER_INACTIVITY_TIMEOUT_MS;
    config.schedule.tag_slot = node_config.tag_slot;
    config.schedule.tag_slot_count = TAG_SLOT_COUNT;
    config.schedule.tag_slot_ms = TAG_SLOT_MS;
    config.schedule.session_timeout_ms = RANGING_SESSION_TIMEOUT_MS;
    config.schedule.range_period_ms = RANGING_PERIOD_MS;
    config.anchor_is_coordinator = node_config.anchor_is_coordinator;
    config
}

fn now_ms() -> u32 {
    Instant::now().as_millis() as u32
}

fn role_label(role: Role) -> &'static str {
    match role {
        Role::Tag => "tag",
        Role::Anchor => "anchor",
    }
}

fn has_rx_work(irq_status: SysStatus) -> bool {
    irq_status.intersects(RX_WORK_EVENTS)
}

fn rx_error_label(error: RxError) -> &'static str {
    match error {
        RxError::FrameNotReady => "frame not ready",
        RxError::LeadingEdgeDetection => "leading edge detection",
        RxError::FrameCheck => "frame check",
        RxError::Header => "phy header",
        RxError::ReedSolomon => "reed solomon",
        RxError::Timeout => "frame timeout",
        RxError::SfdTimeout => "sfd timeout",
        RxError::PreambleTimeout => "preamble timeout",
        RxError::Overrun => "overrun",
        RxError::FrameFiltered => "frame filtered",
    }
}
