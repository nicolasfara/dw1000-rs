use defmt::{info, warn};
use dw1000_rs::registers::status;
use dw1000_rs::{
    AntennaDelay, DeviceIdentity, Error, OperatingMode, RadioConfig, RangingConfig, RangingEvent,
    RangingNode, Role, SysStatus,
};
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Ticker};

use crate::board::Board;
use crate::{
    DW1000_LINK_RECOVERY_TIMEOUT_MS, PEER_INACTIVITY_TIMEOUT_MS, RANGING_REPLY_DELAY_US,
    RX_BUFFER_LEN,
};

pub(crate) struct RangingApp<const N: usize> {
    board: Board,
    role: Role,
    radio_config: RadioConfig,
    node: RangingNode<N>,
    rx_buffer: [u8; RX_BUFFER_LEN],
    ticker: Ticker,
    last_radio_activity_ms: u32,
    last_range_update_ms: u32,
}

pub(crate) struct AppInitError {
    board: Board,
    message: &'static str,
}

struct RecoverReason {
    now_ms: u32,
    message: &'static str,
}

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
        identity: DeviceIdentity,
        antenna_delay: AntennaDelay,
    ) -> Result<Self, AppInitError> {
        board.announce_start(role, identity, antenna_delay);

        let radio_config = default_radio_config(identity, antenna_delay);
        let ranging_config = default_ranging_config(identity);
        let mut node = RangingNode::<N>::new(role, ranging_config);

        if board.radio.init(&mut embassy_time::Delay, &radio_config).await.is_err() {
            return Err(AppInitError {
                board,
                message: "dw1000 init failed",
            });
        }
        let now = now_ms();
        if node.start_async(&mut board.radio, now).await.is_err() {
            return Err(AppInitError {
                board,
                message: "ranging start failed",
            });
        }

        Ok(Self {
            board,
            role,
            radio_config,
            node,
            rx_buffer: [0; RX_BUFFER_LEN],
            ticker: Ticker::every(Duration::from_millis(ranging_config.timer_period_ms as u64)),
            last_radio_activity_ms: now,
            last_range_update_ms: now,
        })
    }

    pub(crate) async fn run_forever(mut self) -> ! {
        loop {
            self.step().await;
            self.recover_stalled_link_if_needed().await;
        }
    }

    async fn step(&mut self) {
        let now = now_ms();
        match select(self.ticker.next(), self.board.radio.wait_for_irq()).await {
            Either::First(_) => self.handle_tick(now).await,
            Either::Second(irq_result) => self.handle_irq(irq_result, now).await,
        }
    }

    async fn handle_tick(&mut self, now: u32) {
        let tx_before = self.node.tx_debug_snapshot();
        match self.node.tick_async(&mut self.board.radio, now).await {
            Ok(event) => {
                let mut outcome = StepOutcome::Idle;
                if self.node.tx_debug_snapshot() != tx_before || event.is_some() {
                    outcome = StepOutcome::RadioActivity;
                }
                if self.handle_event(event) {
                    outcome = StepOutcome::RangeUpdated;
                }
                self.record_outcome(now, outcome);
            }
            Err(_) => self.recover(RecoverReason::new(now, "tick failed")).await,
        }
    }

    async fn handle_irq(
        &mut self,
        irq_result: Result<(), Error<impl embedded_hal::spi::Error, impl embedded_hal::digital::Error>>,
        now: u32,
    ) {
        if irq_result.is_err() {
            self.recover(RecoverReason::new(now, "irq wait failed")).await;
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
                if self.handle_event(event) {
                    outcome = StepOutcome::RangeUpdated;
                }
            }

            if has_rx_work(irq_status) {
                match self
                    .node
                    .on_rx_async(&mut self.board.radio, now, &mut self.rx_buffer)
                    .await
                {
                    Ok(event) => {
                        outcome = outcome.merge(StepOutcome::RadioActivity);
                        if self.handle_event(event) {
                            outcome = StepOutcome::RangeUpdated;
                        }
                    }
                    Err(Error::Receive(_)) => {
                        outcome = outcome.merge(StepOutcome::RadioActivity);
                        warn!("rx error");
                    }
                    Err(Error::Protocol(_)) => {
                        outcome = outcome.merge(StepOutcome::RadioActivity);
                        warn!("protocol error");
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
            .recover_or_fault(&mut self.node, &self.radio_config, reason.now_ms, reason.message)
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
                radio_gap_ms,
                range_gap_ms
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

    fn handle_event(&self, event: Option<RangingEvent>) -> bool {
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
                        "tag distance anchor={=u16} range_m={=f32}",
                        snapshot.short_address.raw(),
                        snapshot.range_m
                    );
                }
                true
            }
            Some(RangingEvent::RangingInitReceived(short)) => {
                info!("tag ranging init from {=u16}", short.raw());
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

fn default_ranging_config(identity: DeviceIdentity) -> RangingConfig {
    let mut config = RangingConfig::new(identity);
    config.reply_delay_us = RANGING_REPLY_DELAY_US;
    config.reset_period_ms = PEER_INACTIVITY_TIMEOUT_MS;
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
    irq_status.contains(status::RX_FRAME_READY)
        || irq_status.contains(status::RX_FRAME_GOOD)
        || irq_status.contains(status::RX_FRAME_CHECK_ERROR)
        || irq_status.contains(status::RX_REED_SOLOMON_ERROR)
        || irq_status.contains(status::RX_TIMEOUT)
        || irq_status.contains(status::RX_HEADER_ERROR)
        || irq_status.contains(status::LDE_ERROR)
}
