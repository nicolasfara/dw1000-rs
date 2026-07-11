#![allow(missing_docs)]

extern crate alloc;

use alloc::collections::VecDeque;

use dw1000_rs::protocol::{
    decode_poll_targets, decode_range_timings, detect_frame_kind, encode_discovery_blink,
    encode_poll, encode_poll_ack, encode_range, encode_range_report, encode_ranging_init,
    encode_schedule_sync, parse_frame, Frame, FrameKind, PollTarget, RangeReportPayload,
    RangeTiming,
};
use dw1000_rs::ranging::RangingRadio;
use dw1000_rs::{
    DelayedTime, DeviceIdentity, DwTime, Error, Eui64, PanId, RangingConfig, RangingEvent,
    RangingNode, Role, RxFrame, RxOptions, ShortAddress, SignalMetrics, SysStatus, Timestamps,
    TxOptions,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct MockError;

#[derive(Debug, Clone)]
struct QueuedFrame {
    bytes: Vec<u8>,
    timestamp: DwTime,
    metrics: SignalMetrics,
}

#[derive(Debug, Clone, Default)]
struct MockRadio {
    rx_frames: VecDeque<QueuedFrame>,
    transmitted: Vec<(Vec<u8>, TxOptions)>,
    receive_modes: Vec<RxOptions>,
    next_timestamps: VecDeque<Timestamps>,
    delayed_times: VecDeque<DwTime>,
}

impl MockRadio {
    fn push_rx(&mut self, bytes: &[u8], timestamp: DwTime, metrics: SignalMetrics) {
        self.rx_frames.push_back(QueuedFrame {
            bytes: bytes.to_vec(),
            timestamp,
            metrics,
        });
    }

    fn last_tx(&self) -> &[u8] {
        &self.transmitted.last().unwrap().0
    }
}

impl RangingRadio<MockError, MockError> for MockRadio {
    fn start_receive(&mut self, options: RxOptions) -> Result<(), Error<MockError, MockError>> {
        self.receive_modes.push(options);
        Ok(())
    }

    fn transmit(
        &mut self,
        frame: &[u8],
        options: TxOptions,
    ) -> Result<(), Error<MockError, MockError>> {
        self.transmitted.push((frame.to_vec(), options));
        Ok(())
    }

    fn read_frame<'a>(
        &mut self,
        buffer: &'a mut [u8],
    ) -> Result<RxFrame<'a>, Error<MockError, MockError>> {
        let queued = self.rx_frames.pop_front().unwrap();
        buffer[..queued.bytes.len()].copy_from_slice(&queued.bytes);
        Ok(RxFrame {
            bytes: &buffer[..queued.bytes.len()],
            timestamp: queued.timestamp,
            metrics: queued.metrics,
            status: SysStatus::EMPTY,
        })
    }

    fn read_timestamps(&mut self) -> Result<Timestamps, Error<MockError, MockError>> {
        Ok(self.next_timestamps.pop_front().unwrap())
    }

    fn schedule_delayed(
        &mut self,
        delay: DwTime,
    ) -> Result<DelayedTime, Error<MockError, MockError>> {
        let time = self.delayed_times.pop_front().unwrap_or(delay);
        Ok(DelayedTime::new(time, time))
    }
}

fn identity(short: u16, eui: [u8; 8]) -> DeviceIdentity {
    DeviceIdentity::new(
        PanId::new(0xDECA),
        ShortAddress::new(short),
        Eui64::new(eui),
    )
}

fn metrics() -> SignalMetrics {
    SignalMetrics {
        receive_power_dbm: -82.0,
        first_path_power_dbm: -84.0,
        quality: 1.5,
    }
}

#[test]
fn single_tag_anchor_exchange_completes() {
    let tag_identity = identity(0x1234, [0x7D, 0x00, 0x22, 0xEA, 0x82, 0x60, 0x3B, 0x9C]);
    let anchor_identity = identity(0x4321, [0x82, 0x17, 0x5B, 0xD5, 0xA9, 0x9A, 0xE2, 0x9C]);
    let mut tag = RangingNode::<4>::new(Role::Tag, RangingConfig::new(tag_identity));
    let mut anchor = RangingNode::<4>::new(Role::Anchor, RangingConfig::new(anchor_identity));
    let mut tag_radio = MockRadio::default();
    let mut anchor_radio = MockRadio::default();

    tag.start(&mut tag_radio, 0).unwrap();
    anchor.start(&mut anchor_radio, 0).unwrap();

    tag.tick(&mut tag_radio, 80).unwrap();
    anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(10), metrics());
    let blink_event = anchor
        .on_rx(&mut anchor_radio, 81, &mut [0u8; 127])
        .unwrap();
    assert!(matches!(blink_event, Some(RangingEvent::BlinkReceived(_))));

    tag_radio.push_rx(anchor_radio.last_tx(), DwTime::from_ticks(20), metrics());
    let init_event = tag.on_rx(&mut tag_radio, 82, &mut [0u8; 127]).unwrap();
    assert_eq!(
        init_event,
        Some(RangingEvent::RangingInitReceived(ShortAddress::new(0x4321)))
    );

    tag.tick(&mut tag_radio, 160).unwrap();
    tag.tick(&mut tag_radio, 240).unwrap();
    tag_radio.next_timestamps.push_back(Timestamps {
        tx: DwTime::from_ticks(100),
        rx: DwTime::zero(),
        system: DwTime::zero(),
    });
    tag.on_tx_done(&mut tag_radio).unwrap();

    anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(120), metrics());
    anchor
        .on_rx(&mut anchor_radio, 241, &mut [0u8; 127])
        .unwrap();
    anchor_radio.next_timestamps.push_back(Timestamps {
        tx: DwTime::from_ticks(180),
        rx: DwTime::zero(),
        system: DwTime::zero(),
    });
    anchor.on_tx_done(&mut anchor_radio).unwrap();

    tag_radio.push_rx(anchor_radio.last_tx(), DwTime::from_ticks(200), metrics());
    tag_radio.delayed_times.push_back(DwTime::from_ticks(260));
    tag.on_rx(&mut tag_radio, 242, &mut [0u8; 127]).unwrap();
    tag_radio.next_timestamps.push_back(Timestamps {
        tx: DwTime::from_ticks(260),
        rx: DwTime::zero(),
        system: DwTime::zero(),
    });
    tag.on_tx_done(&mut tag_radio).unwrap();

    anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(280), metrics());
    let anchor_event = anchor
        .on_rx(&mut anchor_radio, 243, &mut [0u8; 127])
        .unwrap();
    let Some(RangingEvent::RangeUpdated(anchor_snapshot)) = anchor_event else {
        panic!("expected anchor range update");
    };

    tag_radio.push_rx(anchor_radio.last_tx(), DwTime::from_ticks(300), metrics());
    let tag_event = tag.on_rx(&mut tag_radio, 244, &mut [0u8; 127]).unwrap();
    let Some(RangingEvent::RangeUpdated(tag_snapshot)) = tag_event else {
        panic!("expected tag range update");
    };

    let expected_range = DwTime::from_ticks(20).as_meters();
    assert!((anchor_snapshot.range_m - expected_range).abs() < 1e-6);
    assert!((tag_snapshot.range_m - expected_range).abs() < 1e-6);
}

#[test]
fn tag_waits_for_every_range_report_before_starting_the_next_poll() {
    let tag_identity = identity(0x1234, [1, 2, 3, 4, 5, 6, 7, 8]);
    let anchor_a = identity(0x4321, [9, 10, 11, 12, 13, 14, 15, 16]);
    let anchor_b = identity(0x4322, [17, 18, 19, 20, 21, 22, 23, 24]);
    let mut tag = RangingNode::<4>::new(Role::Tag, RangingConfig::new(tag_identity));
    let mut radio = MockRadio::default();

    tag.start(&mut radio, 0).unwrap();
    tag.tick(&mut radio, 80).unwrap();

    for anchor in [anchor_a, anchor_b] {
        let mut init = [0u8; 127];
        let init_len =
            encode_ranging_init(0, anchor.short_address, tag_identity.eui, &mut init).unwrap();
        radio.push_rx(&init[..init_len], DwTime::from_ticks(20), metrics());
        tag.on_rx(&mut radio, 81, &mut [0u8; 127]).unwrap();
    }

    tag.tick(&mut radio, 160).unwrap();
    radio.next_timestamps.push_back(Timestamps {
        tx: DwTime::from_ticks(100),
        rx: DwTime::zero(),
        system: DwTime::zero(),
    });
    tag.on_tx_done(&mut radio).unwrap();

    for anchor in [anchor_a, anchor_b] {
        let mut poll_ack = [0u8; 127];
        let poll_ack_len = encode_poll_ack(
            1,
            anchor.short_address,
            tag_identity.short_address,
            &mut poll_ack,
        )
        .unwrap();
        radio.push_rx(
            &poll_ack[..poll_ack_len],
            DwTime::from_ticks(200),
            metrics(),
        );
        tag.on_rx(&mut radio, 161, &mut [0u8; 127]).unwrap();
    }
    radio.next_timestamps.push_back(Timestamps {
        tx: DwTime::from_ticks(260),
        rx: DwTime::zero(),
        system: DwTime::zero(),
    });
    tag.on_tx_done(&mut radio).unwrap();

    let tx_count_before_reports = radio.transmitted.len();
    tag.tick(&mut radio, 240).unwrap();
    assert_eq!(radio.transmitted.len(), tx_count_before_reports);

    let report = RangeReportPayload {
        poll_received: DwTime::from_ticks(120),
        poll_ack_sent: DwTime::from_ticks(180),
        range_received: DwTime::from_ticks(280),
        receive_power_dbm: -82.0,
    };
    for (index, anchor) in [anchor_a, anchor_b].into_iter().enumerate() {
        let mut range_report = [0u8; 127];
        let range_report_len = encode_range_report(
            2,
            anchor.short_address,
            tag_identity.short_address,
            report,
            &mut range_report,
        )
        .unwrap();
        radio.push_rx(
            &range_report[..range_report_len],
            DwTime::from_ticks(300),
            metrics(),
        );
        tag.on_rx(&mut radio, 241 + index as u32, &mut [0u8; 127])
            .unwrap();
    }

    tag.tick(&mut radio, 320).unwrap();
    assert_eq!(radio.transmitted.len(), tx_count_before_reports + 1);
    assert_eq!(detect_frame_kind(radio.last_tx()).unwrap(), FrameKind::Poll);
}

#[test]
fn tag_resets_after_exchange_timeout_and_polls_again() {
    let tag_identity = identity(0x1234, [1, 2, 3, 4, 5, 6, 7, 8]);
    let anchor_identity = identity(0x4321, [9, 10, 11, 12, 13, 14, 15, 16]);
    let mut config = RangingConfig::new(tag_identity);
    config.reset_period_ms = 2_000;
    config.exchange_timeout_ms = 200;
    let mut tag = RangingNode::<4>::new(Role::Tag, config);
    let mut radio = MockRadio::default();

    tag.start(&mut radio, 0).unwrap();
    tag.tick(&mut radio, 80).unwrap();

    let mut init = [0u8; 127];
    let init_len = encode_ranging_init(
        0,
        anchor_identity.short_address,
        tag_identity.eui,
        &mut init,
    )
    .unwrap();
    radio.push_rx(&init[..init_len], DwTime::from_ticks(20), metrics());
    tag.on_rx(&mut radio, 81, &mut [0u8; 127]).unwrap();

    tag.tick(&mut radio, 160).unwrap();
    // The timeout itself must not transmit (a blink here could land in
    // another tag's slot); the tag re-polls on its next scheduled attempt.
    let tx_count = radio.transmitted.len();
    assert_eq!(
        tag.tick(&mut radio, 400).unwrap(),
        Some(RangingEvent::ExchangeTimedOut)
    );
    assert_eq!(radio.transmitted.len(), tx_count);

    tag.tick(&mut radio, 480).unwrap();
    assert_eq!(radio.transmitted.len(), tx_count + 1);
    assert_eq!(detect_frame_kind(radio.last_tx()).unwrap(), FrameKind::Poll);
}

#[test]
fn coordinator_broadcasts_schedule_sync_without_changing_anchor_peers() {
    let coordinator_identity = identity(0x4321, [1, 2, 3, 4, 5, 6, 7, 8]);
    let secondary_identity = identity(0x4322, [9, 10, 11, 12, 13, 14, 15, 16]);
    let mut coordinator_config = RangingConfig::new(coordinator_identity);
    coordinator_config.anchor_is_coordinator = true;
    let mut coordinator = RangingNode::<4>::new(Role::Anchor, coordinator_config);
    let mut secondary = RangingNode::<4>::new(Role::Anchor, RangingConfig::new(secondary_identity));
    let mut coordinator_radio = MockRadio::default();
    let mut secondary_radio = MockRadio::default();

    coordinator.start(&mut coordinator_radio, 0).unwrap();
    secondary.start(&mut secondary_radio, 0).unwrap();
    coordinator.tick(&mut coordinator_radio, 80).unwrap();

    assert_eq!(
        detect_frame_kind(coordinator_radio.last_tx()).unwrap(),
        FrameKind::ScheduleSync
    );
    secondary_radio.push_rx(
        coordinator_radio.last_tx(),
        DwTime::from_ticks(20),
        metrics(),
    );
    assert_eq!(
        secondary
            .on_rx(&mut secondary_radio, 81, &mut [0u8; 127])
            .unwrap(),
        Some(RangingEvent::ScheduleSyncReceived(
            coordinator_identity.short_address
        ))
    );
    assert_eq!(secondary.peers().count(), 0);
}

#[test]
fn tag_ranges_with_acknowledged_anchors_when_another_anchor_misses_its_reply() {
    let tag_identity = identity(0x1234, [1, 2, 3, 4, 5, 6, 7, 8]);
    let anchor_a = identity(0x4321, [9, 10, 11, 12, 13, 14, 15, 16]);
    let anchor_b = identity(0x4322, [17, 18, 19, 20, 21, 22, 23, 24]);
    let mut config = RangingConfig::new(tag_identity);
    config.reset_period_ms = 1_000;
    config.schedule.session_timeout_ms = 100;
    let mut tag = RangingNode::<4>::new(Role::Tag, config);
    let mut radio = MockRadio::default();

    tag.start(&mut radio, 0).unwrap();
    tag.tick(&mut radio, 80).unwrap();
    for anchor in [anchor_a, anchor_b] {
        let mut init = [0u8; 127];
        let init_len =
            encode_ranging_init(0, anchor.short_address, tag_identity.eui, &mut init).unwrap();
        radio.push_rx(&init[..init_len], DwTime::from_ticks(20), metrics());
        tag.on_rx(&mut radio, 81, &mut [0u8; 127]).unwrap();
    }

    tag.tick(&mut radio, 160).unwrap();
    let mut poll_ack = [0u8; 127];
    let poll_ack_len = encode_poll_ack(
        1,
        anchor_a.short_address,
        tag_identity.short_address,
        &mut poll_ack,
    )
    .unwrap();
    radio.push_rx(
        &poll_ack[..poll_ack_len],
        DwTime::from_ticks(200),
        metrics(),
    );
    tag.on_rx(&mut radio, 180, &mut [0u8; 127]).unwrap();

    radio.delayed_times.push_back(DwTime::from_ticks(260));
    tag.tick(&mut radio, 260).unwrap();
    let Frame::Range { payload, .. } = parse_frame(radio.last_tx()).unwrap() else {
        panic!("expected range frame");
    };
    let mut timings = [RangeTiming {
        short_address: ShortAddress::new(0),
        poll_sent: DwTime::zero(),
        poll_ack_received: DwTime::zero(),
        range_sent: DwTime::zero(),
    }; 2];
    assert_eq!(decode_range_timings(payload, &mut timings).unwrap(), 1);
    assert_eq!(timings[0].short_address, anchor_a.short_address);

    let mut report = [0u8; 127];
    let report_len = encode_range_report(
        2,
        anchor_a.short_address,
        tag_identity.short_address,
        RangeReportPayload {
            poll_received: DwTime::from_ticks(120),
            poll_ack_sent: DwTime::from_ticks(180),
            range_received: DwTime::from_ticks(280),
            receive_power_dbm: -82.0,
        },
        &mut report,
    )
    .unwrap();
    radio.push_rx(&report[..report_len], DwTime::from_ticks(300), metrics());
    assert!(matches!(
        tag.on_rx(&mut radio, 270, &mut [0u8; 127]).unwrap(),
        Some(RangingEvent::RangeUpdated(_))
    ));

    tag.tick(&mut radio, 340).unwrap();
    assert_eq!(detect_frame_kind(radio.last_tx()).unwrap(), FrameKind::Poll);
}

#[test]
fn tick_prunes_inactive_peers() {
    let tag_identity = identity(0x1234, [0, 1, 2, 3, 4, 5, 6, 7]);
    let anchor_identity = identity(0x4321, [8, 9, 10, 11, 12, 13, 14, 15]);
    let mut tag = RangingNode::<4>::new(Role::Tag, RangingConfig::new(tag_identity));
    let mut anchor = RangingNode::<4>::new(Role::Anchor, RangingConfig::new(anchor_identity));
    let mut tag_radio = MockRadio::default();
    let mut anchor_radio = MockRadio::default();

    tag.start(&mut tag_radio, 0).unwrap();
    anchor.start(&mut anchor_radio, 0).unwrap();
    tag.tick(&mut tag_radio, 80).unwrap();
    anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(10), metrics());
    anchor
        .on_rx(&mut anchor_radio, 81, &mut [0u8; 127])
        .unwrap();
    tag_radio.push_rx(anchor_radio.last_tx(), DwTime::from_ticks(20), metrics());
    tag.on_rx(&mut tag_radio, 82, &mut [0u8; 127]).unwrap();

    let event = tag.tick(&mut tag_radio, 400).unwrap();
    assert_eq!(
        event,
        Some(RangingEvent::PeerInactive(ShortAddress::new(0x4321)))
    );
}

#[test]
fn anchors_stagger_ranging_init_after_blink() {
    let tag_identity = identity(0x1234, [1, 2, 3, 4, 5, 6, 7, 8]);
    let mut blink = [0u8; 127];
    let blink_len =
        encode_discovery_blink(0, tag_identity.eui, tag_identity.short_address, &mut blink)
            .unwrap();

    for (anchor_identity, reply_delay_us) in [
        (identity(0x4321, [9, 10, 11, 12, 13, 14, 15, 16]), 7_000),
        (identity(0x4322, [17, 18, 19, 20, 21, 22, 23, 24]), 21_000),
        (identity(0x4323, [25, 26, 27, 28, 29, 30, 31, 32]), 35_000),
        (identity(0x4324, [33, 34, 35, 36, 37, 38, 39, 40]), 49_000),
    ] {
        let mut config = RangingConfig::new(anchor_identity);
        config.discovery_reply_delay_us = reply_delay_us;
        let mut anchor = RangingNode::<4>::new(Role::Anchor, config);
        let mut radio = MockRadio::default();

        anchor.start(&mut radio, 0).unwrap();
        radio.push_rx(&blink[..blink_len], DwTime::from_ticks(10), metrics());
        assert!(matches!(
            anchor.on_rx(&mut radio, 1, &mut [0u8; 127]).unwrap(),
            Some(RangingEvent::BlinkReceived(_))
        ));

        let scheduled = DwTime::from_micros(reply_delay_us as f32);
        assert_eq!(
            radio.transmitted[0].1.delayed_time,
            Some(DelayedTime::new(scheduled, scheduled))
        );
    }
}

#[test]
fn anchor_recovers_after_inactivity_and_accepts_rediscovery() {
    let tag_identity = identity(0x1234, [0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]);
    let anchor_identity = identity(0x4321, [0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27]);
    let mut tag_config = RangingConfig::new(tag_identity);
    let mut anchor_config = RangingConfig::new(anchor_identity);
    tag_config.reset_period_ms = 100;
    anchor_config.reset_period_ms = 100;

    let mut tag = RangingNode::<4>::new(Role::Tag, tag_config);
    let mut anchor = RangingNode::<4>::new(Role::Anchor, anchor_config);
    let mut tag_radio = MockRadio::default();
    let mut anchor_radio = MockRadio::default();

    tag.start(&mut tag_radio, 0).unwrap();
    anchor.start(&mut anchor_radio, 0).unwrap();

    tag.tick(&mut tag_radio, 80).unwrap();
    anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(10), metrics());
    anchor
        .on_rx(&mut anchor_radio, 81, &mut [0u8; 127])
        .unwrap();
    tag_radio.push_rx(anchor_radio.last_tx(), DwTime::from_ticks(20), metrics());
    tag.on_rx(&mut tag_radio, 82, &mut [0u8; 127]).unwrap();

    tag.tick(&mut tag_radio, 160).unwrap();
    anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(30), metrics());
    anchor
        .on_rx(&mut anchor_radio, 161, &mut [0u8; 127])
        .unwrap();

    let anchor_timeout = anchor.tick(&mut anchor_radio, 300).unwrap();
    assert_eq!(
        anchor_timeout,
        Some(RangingEvent::PeerInactive(ShortAddress::new(0x1234)))
    );
    let tag_timeout = tag.tick(&mut tag_radio, 301).unwrap();
    assert_eq!(
        tag_timeout,
        Some(RangingEvent::PeerInactive(ShortAddress::new(0x4321)))
    );

    tag.tick(&mut tag_radio, 380).unwrap();
    anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(40), metrics());
    let blink_event = anchor
        .on_rx(&mut anchor_radio, 381, &mut [0u8; 127])
        .unwrap();
    assert!(matches!(blink_event, Some(RangingEvent::BlinkReceived(_))));

    tag_radio.push_rx(anchor_radio.last_tx(), DwTime::from_ticks(50), metrics());
    let init_event = tag.on_rx(&mut tag_radio, 382, &mut [0u8; 127]).unwrap();
    assert_eq!(
        init_event,
        Some(RangingEvent::RangingInitReceived(ShortAddress::new(0x4321)))
    );

    tag.tick(&mut tag_radio, 460).unwrap();
    anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(60), metrics());
    assert!(anchor
        .on_rx(&mut anchor_radio, 461, &mut [0u8; 127])
        .is_ok());
}

#[test]
fn recover_link_preserves_peer_and_next_poll_is_accepted_without_rediscovery() {
    let tag_identity = identity(0x1234, [0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37]);
    let anchor_identity = identity(0x4321, [0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47]);
    let mut tag = RangingNode::<4>::new(Role::Tag, RangingConfig::new(tag_identity));
    let mut anchor = RangingNode::<4>::new(Role::Anchor, RangingConfig::new(anchor_identity));
    let mut tag_radio = MockRadio::default();
    let mut anchor_radio = MockRadio::default();

    tag.start(&mut tag_radio, 0).unwrap();
    anchor.start(&mut anchor_radio, 0).unwrap();

    tag.tick(&mut tag_radio, 80).unwrap();
    anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(10), metrics());
    anchor
        .on_rx(&mut anchor_radio, 81, &mut [0u8; 127])
        .unwrap();
    tag_radio.push_rx(anchor_radio.last_tx(), DwTime::from_ticks(20), metrics());
    let init_event = tag.on_rx(&mut tag_radio, 82, &mut [0u8; 127]).unwrap();
    assert_eq!(
        init_event,
        Some(RangingEvent::RangingInitReceived(ShortAddress::new(0x4321)))
    );

    let receive_calls_before = anchor_radio.receive_modes.len();
    assert_eq!(anchor.peers().count(), 1);
    anchor.recover_link(&mut anchor_radio, 120).unwrap();
    assert_eq!(anchor.peers().count(), 1);
    assert_eq!(anchor_radio.receive_modes.len(), receive_calls_before + 1);
    assert_eq!(
        anchor_radio.receive_modes.last().copied(),
        Some(RxOptions {
            delayed_time: None,
            permanent: true,
        })
    );

    tag.tick(&mut tag_radio, 160).unwrap();
    anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(30), metrics());
    let tx_count_before = anchor_radio.transmitted.len();
    let event = anchor
        .on_rx(&mut anchor_radio, 161, &mut [0u8; 127])
        .unwrap();
    assert_eq!(event, None);
    assert_eq!(anchor_radio.transmitted.len(), tx_count_before + 1);
    assert_eq!(
        detect_frame_kind(anchor_radio.last_tx()).unwrap(),
        FrameKind::PollAck
    );
}

#[test]
fn tag_ignores_poll_ack_for_other_destination() {
    let tag_identity = identity(0x1234, [1, 2, 3, 4, 5, 6, 7, 8]);
    let anchor_identity = identity(0x4321, [9, 10, 11, 12, 13, 14, 15, 16]);
    let mut tag = RangingNode::<4>::new(Role::Tag, RangingConfig::new(tag_identity));
    let mut radio = MockRadio::default();

    tag.start(&mut radio, 0).unwrap();
    tag.tick(&mut radio, 80).unwrap();

    let mut init = [0u8; 127];
    let init_len = encode_ranging_init(
        0,
        anchor_identity.short_address,
        tag_identity.eui,
        &mut init,
    )
    .unwrap();
    radio.push_rx(&init[..init_len], DwTime::from_ticks(20), metrics());
    let init_event = tag.on_rx(&mut radio, 81, &mut [0u8; 127]).unwrap();
    assert_eq!(
        init_event,
        Some(RangingEvent::RangingInitReceived(
            anchor_identity.short_address
        ))
    );

    tag.tick(&mut radio, 160).unwrap();
    radio.next_timestamps.push_back(Timestamps {
        tx: DwTime::from_ticks(100),
        rx: DwTime::zero(),
        system: DwTime::zero(),
    });
    tag.on_tx_done(&mut radio).unwrap();

    let tx_count_before = radio.transmitted.len();
    let mut wrong_poll_ack = [0u8; 127];
    let wrong_poll_ack_len = encode_poll_ack(
        1,
        anchor_identity.short_address,
        ShortAddress::new(0x7777),
        &mut wrong_poll_ack,
    )
    .unwrap();
    radio.push_rx(
        &wrong_poll_ack[..wrong_poll_ack_len],
        DwTime::from_ticks(200),
        metrics(),
    );
    assert_eq!(tag.on_rx(&mut radio, 161, &mut [0u8; 127]).unwrap(), None);
    assert_eq!(radio.transmitted.len(), tx_count_before);

    radio.delayed_times.push_back(DwTime::from_ticks(260));
    let mut poll_ack = [0u8; 127];
    let poll_ack_len = encode_poll_ack(
        1,
        anchor_identity.short_address,
        tag_identity.short_address,
        &mut poll_ack,
    )
    .unwrap();
    radio.push_rx(
        &poll_ack[..poll_ack_len],
        DwTime::from_ticks(200),
        metrics(),
    );
    assert_eq!(tag.on_rx(&mut radio, 162, &mut [0u8; 127]).unwrap(), None);
    assert_eq!(
        detect_frame_kind(radio.last_tx()).unwrap(),
        FrameKind::Range
    );
}

#[test]
fn stale_poll_ack_is_ignored_while_waiting_for_range_report() {
    let tag_identity = identity(0x1234, [17, 18, 19, 20, 21, 22, 23, 24]);
    let anchor_identity = identity(0x4321, [25, 26, 27, 28, 29, 30, 31, 32]);
    let mut tag = RangingNode::<4>::new(Role::Tag, RangingConfig::new(tag_identity));
    let mut radio = MockRadio::default();

    tag.start(&mut radio, 0).unwrap();
    tag.tick(&mut radio, 80).unwrap();

    let mut init = [0u8; 127];
    let init_len = encode_ranging_init(
        0,
        anchor_identity.short_address,
        tag_identity.eui,
        &mut init,
    )
    .unwrap();
    radio.push_rx(&init[..init_len], DwTime::from_ticks(20), metrics());
    tag.on_rx(&mut radio, 81, &mut [0u8; 127]).unwrap();

    tag.tick(&mut radio, 160).unwrap();
    radio.next_timestamps.push_back(Timestamps {
        tx: DwTime::from_ticks(100),
        rx: DwTime::zero(),
        system: DwTime::zero(),
    });
    tag.on_tx_done(&mut radio).unwrap();

    radio.delayed_times.push_back(DwTime::from_ticks(260));
    let mut poll_ack = [0u8; 127];
    let poll_ack_len = encode_poll_ack(
        1,
        anchor_identity.short_address,
        tag_identity.short_address,
        &mut poll_ack,
    )
    .unwrap();
    radio.push_rx(
        &poll_ack[..poll_ack_len],
        DwTime::from_ticks(200),
        metrics(),
    );
    tag.on_rx(&mut radio, 161, &mut [0u8; 127]).unwrap();
    radio.next_timestamps.push_back(Timestamps {
        tx: DwTime::from_ticks(260),
        rx: DwTime::zero(),
        system: DwTime::zero(),
    });
    tag.on_tx_done(&mut radio).unwrap();

    let tx_count_before = radio.transmitted.len();
    radio.push_rx(
        &poll_ack[..poll_ack_len],
        DwTime::from_ticks(200),
        metrics(),
    );
    assert_eq!(tag.on_rx(&mut radio, 162, &mut [0u8; 127]).unwrap(), None);
    assert_eq!(radio.transmitted.len(), tx_count_before);

    let mut range_report = [0u8; 127];
    let range_report_len = encode_range_report(
        2,
        anchor_identity.short_address,
        tag_identity.short_address,
        RangeReportPayload {
            poll_received: DwTime::from_ticks(120),
            poll_ack_sent: DwTime::from_ticks(180),
            range_received: DwTime::from_ticks(280),
            receive_power_dbm: -82.0,
        },
        &mut range_report,
    )
    .unwrap();
    radio.push_rx(
        &range_report[..range_report_len],
        DwTime::from_ticks(300),
        metrics(),
    );
    assert!(matches!(
        tag.on_rx(&mut radio, 163, &mut [0u8; 127]).unwrap(),
        Some(RangingEvent::RangeUpdated(_))
    ));
}

#[test]
fn anchor_accepts_a_new_poll_after_missing_the_range_frame() {
    let tag_identity = identity(0x1234, [57, 58, 59, 60, 61, 62, 63, 64]);
    let anchor_identity = identity(0x4321, [65, 66, 67, 68, 69, 70, 71, 72]);
    let mut anchor = RangingNode::<4>::new(Role::Anchor, RangingConfig::new(anchor_identity));
    let mut radio = MockRadio::default();

    anchor.start(&mut radio, 0).unwrap();
    let mut blink = [0u8; 127];
    let blink_len =
        encode_discovery_blink(0, tag_identity.eui, tag_identity.short_address, &mut blink)
            .unwrap();
    radio.push_rx(&blink[..blink_len], DwTime::from_ticks(10), metrics());
    anchor.on_rx(&mut radio, 1, &mut [0u8; 127]).unwrap();

    let targets = [PollTarget {
        short_address: anchor_identity.short_address,
        reply_delay_us: 7_000,
    }];
    let mut poll = [0u8; 127];
    let poll_len = encode_poll(
        1,
        tag_identity.short_address,
        ShortAddress::BROADCAST,
        &targets,
        &mut poll,
    )
    .unwrap();
    radio.push_rx(&poll[..poll_len], DwTime::from_ticks(100), metrics());
    anchor.on_rx(&mut radio, 2, &mut [0u8; 127]).unwrap();
    assert_eq!(
        detect_frame_kind(radio.last_tx()).unwrap(),
        FrameKind::PollAck
    );

    // The range frame is lost; the next poll opens a fresh exchange and must
    // be acknowledged immediately instead of being dropped.
    let tx_count = radio.transmitted.len();
    let mut poll = [0u8; 127];
    let poll_len = encode_poll(
        2,
        tag_identity.short_address,
        ShortAddress::BROADCAST,
        &targets,
        &mut poll,
    )
    .unwrap();
    radio.push_rx(&poll[..poll_len], DwTime::from_ticks(200), metrics());
    anchor.on_rx(&mut radio, 3, &mut [0u8; 127]).unwrap();
    assert_eq!(radio.transmitted.len(), tx_count + 1);
    assert_eq!(
        detect_frame_kind(radio.last_tx()).unwrap(),
        FrameKind::PollAck
    );
}

#[test]
fn restarted_anchor_rejoins_from_a_poll_without_rediscovery() {
    let tag_identity = identity(0x1234, [73, 74, 75, 76, 77, 78, 79, 80]);
    let anchor_identity = identity(0x4321, [81, 82, 83, 84, 85, 86, 87, 88]);
    let mut anchor = RangingNode::<4>::new(Role::Anchor, RangingConfig::new(anchor_identity));
    let mut radio = MockRadio::default();

    anchor.start(&mut radio, 0).unwrap();
    assert_eq!(anchor.peers().count(), 0);

    // The anchor restarted after discovery: the tag still polls it, and the
    // poll itself must be enough to rejoin the session.
    let targets = [PollTarget {
        short_address: anchor_identity.short_address,
        reply_delay_us: 7_000,
    }];
    let mut poll = [0u8; 127];
    let poll_len = encode_poll(
        5,
        tag_identity.short_address,
        ShortAddress::BROADCAST,
        &targets,
        &mut poll,
    )
    .unwrap();
    radio.push_rx(&poll[..poll_len], DwTime::from_ticks(100), metrics());
    anchor.on_rx(&mut radio, 1, &mut [0u8; 127]).unwrap();

    assert_eq!(anchor.peers().count(), 1);
    assert_eq!(
        detect_frame_kind(radio.last_tx()).unwrap(),
        FrameKind::PollAck
    );
}

#[test]
fn anchor_rearms_for_polls_when_the_range_frame_skips_it() {
    let tag_identity = identity(0x1234, [89, 90, 91, 92, 93, 94, 95, 96]);
    let anchor_identity = identity(0x4321, [97, 98, 99, 100, 101, 102, 103, 104]);
    let other_anchor = identity(0x4322, [105, 106, 107, 108, 109, 110, 111, 112]);
    let mut anchor = RangingNode::<4>::new(Role::Anchor, RangingConfig::new(anchor_identity));
    let mut radio = MockRadio::default();

    anchor.start(&mut radio, 0).unwrap();
    let targets = [PollTarget {
        short_address: anchor_identity.short_address,
        reply_delay_us: 7_000,
    }];
    let mut poll = [0u8; 127];
    let poll_len = encode_poll(
        0,
        tag_identity.short_address,
        ShortAddress::BROADCAST,
        &targets,
        &mut poll,
    )
    .unwrap();
    radio.push_rx(&poll[..poll_len], DwTime::from_ticks(100), metrics());
    anchor.on_rx(&mut radio, 1, &mut [0u8; 127]).unwrap();

    // The tag missed this anchor's poll-ack: the broadcast range frame only
    // carries the other anchor's timings and must produce no report.
    let timings = [RangeTiming {
        short_address: other_anchor.short_address,
        poll_sent: DwTime::from_ticks(100),
        poll_ack_received: DwTime::from_ticks(200),
        range_sent: DwTime::from_ticks(260),
    }];
    let mut range = [0u8; 127];
    let range_len = encode_range(
        1,
        tag_identity.short_address,
        ShortAddress::BROADCAST,
        &timings,
        &mut range,
    )
    .unwrap();
    let tx_count = radio.transmitted.len();
    radio.push_rx(&range[..range_len], DwTime::from_ticks(300), metrics());
    assert_eq!(anchor.on_rx(&mut radio, 2, &mut [0u8; 127]).unwrap(), None);
    assert_eq!(radio.transmitted.len(), tx_count);

    // The next poll must be acknowledged right away.
    let mut poll = [0u8; 127];
    let poll_len = encode_poll(
        2,
        tag_identity.short_address,
        ShortAddress::BROADCAST,
        &targets,
        &mut poll,
    )
    .unwrap();
    radio.push_rx(&poll[..poll_len], DwTime::from_ticks(400), metrics());
    anchor.on_rx(&mut radio, 3, &mut [0u8; 127]).unwrap();
    assert_eq!(radio.transmitted.len(), tx_count + 1);
    assert_eq!(
        detect_frame_kind(radio.last_tx()).unwrap(),
        FrameKind::PollAck
    );
}

#[test]
fn coordinator_defers_schedule_sync_while_an_exchange_is_active() {
    let tag_identity = identity(0x1234, [113, 114, 115, 116, 117, 118, 119, 120]);
    let coordinator_identity = identity(0x4321, [121, 122, 123, 124, 125, 126, 127, 128]);
    let mut config = RangingConfig::new(coordinator_identity);
    config.anchor_is_coordinator = true;
    config.reset_period_ms = 10_000;
    let mut coordinator = RangingNode::<4>::new(Role::Anchor, config);
    let mut radio = MockRadio::default();

    coordinator.start(&mut radio, 0).unwrap();
    let targets = [PollTarget {
        short_address: coordinator_identity.short_address,
        reply_delay_us: 7_000,
    }];
    let mut poll = [0u8; 127];
    let poll_len = encode_poll(
        0,
        tag_identity.short_address,
        ShortAddress::BROADCAST,
        &targets,
        &mut poll,
    )
    .unwrap();
    radio.push_rx(&poll[..poll_len], DwTime::from_ticks(100), metrics());
    coordinator.on_rx(&mut radio, 100, &mut [0u8; 127]).unwrap();
    assert_eq!(
        detect_frame_kind(radio.last_tx()).unwrap(),
        FrameKind::PollAck
    );

    // The sync interval has elapsed, but the exchange is still open: the
    // sync broadcast must not collide with the pending range frame.
    let tx_count = radio.transmitted.len();
    coordinator.tick(&mut radio, 160).unwrap();
    assert_eq!(radio.transmitted.len(), tx_count);

    let timings = [RangeTiming {
        short_address: coordinator_identity.short_address,
        poll_sent: DwTime::from_ticks(100),
        poll_ack_received: DwTime::from_ticks(200),
        range_sent: DwTime::from_ticks(260),
    }];
    let mut range = [0u8; 127];
    let range_len = encode_range(
        1,
        tag_identity.short_address,
        ShortAddress::BROADCAST,
        &timings,
        &mut range,
    )
    .unwrap();
    radio.push_rx(&range[..range_len], DwTime::from_ticks(300), metrics());
    coordinator.on_rx(&mut radio, 170, &mut [0u8; 127]).unwrap();

    // Exchange completed and the channel has been quiet: the sync goes out.
    coordinator.tick(&mut radio, 400).unwrap();
    assert_eq!(
        detect_frame_kind(radio.last_tx()).unwrap(),
        FrameKind::ScheduleSync
    );
}

#[test]
fn coordinator_resumes_schedule_sync_after_a_stalled_exchange() {
    let tag_identity = identity(0x1234, [17, 27, 37, 47, 57, 67, 77, 87]);
    let coordinator_identity = identity(0x4321, [18, 28, 38, 48, 58, 68, 78, 88]);
    let mut config = RangingConfig::new(coordinator_identity);
    config.anchor_is_coordinator = true;
    config.reset_period_ms = 10_000;
    config.schedule.range_period_ms = 100;
    config.schedule.session_timeout_ms = 50;
    let mut coordinator = RangingNode::<4>::new(Role::Anchor, config);
    let mut radio = MockRadio::default();

    coordinator.start(&mut radio, 0).unwrap();
    let targets = [PollTarget {
        short_address: coordinator_identity.short_address,
        reply_delay_us: 7_000,
    }];
    let mut poll = [0u8; 127];
    let poll_len = encode_poll(
        0,
        tag_identity.short_address,
        ShortAddress::BROADCAST,
        &targets,
        &mut poll,
    )
    .unwrap();
    radio.push_rx(&poll[..poll_len], DwTime::from_ticks(100), metrics());
    coordinator.on_rx(&mut radio, 100, &mut [0u8; 127]).unwrap();

    // The range frame is lost. While the exchange could still complete, the
    // sync stays deferred...
    let tx_count = radio.transmitted.len();
    coordinator.tick(&mut radio, 180).unwrap();
    assert_eq!(radio.transmitted.len(), tx_count);

    // ...but once the exchange freshness bound expires, the stuck flag must
    // not starve the schedule broadcast (tags stay silent without it).
    coordinator.tick(&mut radio, 260).unwrap();
    assert_eq!(radio.transmitted.len(), tx_count + 1);
    assert_eq!(
        detect_frame_kind(radio.last_tx()).unwrap(),
        FrameKind::ScheduleSync
    );
}

#[test]
fn coordinator_sync_is_never_starved_by_constant_exchanges() {
    let tag_identity = identity(0x1234, [19, 29, 39, 49, 59, 69, 79, 89]);
    let coordinator_identity = identity(0x4321, [20, 30, 40, 50, 60, 70, 80, 90]);
    let mut config = RangingConfig::new(coordinator_identity);
    config.anchor_is_coordinator = true;
    config.reset_period_ms = 10_000;
    config.schedule.range_period_ms = 100;
    let mut coordinator = RangingNode::<4>::new(Role::Anchor, config);
    let mut radio = MockRadio::default();

    coordinator.start(&mut radio, 0).unwrap();
    let targets = [PollTarget {
        short_address: coordinator_identity.short_address,
        reply_delay_us: 7_000,
    }];

    // A poll every 50 ms keeps an exchange pending and the channel busy.
    for (sequence, now) in (100..=450).step_by(50).enumerate() {
        let mut poll = [0u8; 127];
        let poll_len = encode_poll(
            sequence as u8,
            tag_identity.short_address,
            ShortAddress::BROADCAST,
            &targets,
            &mut poll,
        )
        .unwrap();
        radio.push_rx(&poll[..poll_len], DwTime::from_ticks(100), metrics());
        coordinator.on_rx(&mut radio, now, &mut [0u8; 127]).unwrap();
    }

    // Past the hard deadline (4x the sync period) the schedule goes out
    // even though the soft gates never open.
    coordinator.tick(&mut radio, 460).unwrap();
    assert_eq!(
        detect_frame_kind(radio.last_tx()).unwrap(),
        FrameKind::ScheduleSync
    );
}

#[test]
fn tag_uses_schedule_sync_without_creating_a_peer() {
    let tag_identity = identity(0x1234, [1, 3, 5, 7, 9, 11, 13, 15]);
    let coordinator_short = ShortAddress::new(0x4321);
    let mut tag = RangingNode::<4>::new(Role::Tag, RangingConfig::new(tag_identity));
    let mut radio = MockRadio::default();

    tag.start(&mut radio, 0).unwrap();
    let mut sync = [0u8; 127];
    let sync_len = encode_schedule_sync(
        0,
        coordinator_short,
        ShortAddress::BROADCAST,
        1_000,
        2,
        100,
        &mut sync,
    )
    .unwrap();
    radio.push_rx(&sync[..sync_len], DwTime::from_ticks(10), metrics());
    let event = tag.on_rx(&mut radio, 50, &mut [0u8; 127]).unwrap();
    assert_eq!(
        event,
        Some(RangingEvent::ScheduleSyncReceived(coordinator_short))
    );
    // A peer created here would be polled before the coordinator has learned
    // this tag's address, wasting a full exchange.
    assert_eq!(tag.peers().count(), 0);
}

#[test]
fn anchor_serves_two_tags_with_interleaved_exchanges() {
    let tag_a = identity(0x1111, [11, 21, 31, 41, 51, 61, 71, 81]);
    let tag_b = identity(0x2222, [12, 22, 32, 42, 52, 62, 72, 82]);
    let anchor_identity = identity(0x4321, [13, 23, 33, 43, 53, 63, 73, 83]);
    let mut anchor = RangingNode::<4>::new(Role::Anchor, RangingConfig::new(anchor_identity));
    let mut radio = MockRadio::default();

    anchor.start(&mut radio, 0).unwrap();
    let targets = [PollTarget {
        short_address: anchor_identity.short_address,
        reply_delay_us: 7_000,
    }];

    // Tag A polls, then tag B polls before tag A's range frame arrives: the
    // exchange state is per peer, so both must be acknowledged.
    let mut poll = [0u8; 127];
    let poll_len = encode_poll(
        0,
        tag_a.short_address,
        ShortAddress::BROADCAST,
        &targets,
        &mut poll,
    )
    .unwrap();
    radio.push_rx(&poll[..poll_len], DwTime::from_ticks(100), metrics());
    anchor.on_rx(&mut radio, 100, &mut [0u8; 127]).unwrap();
    assert_eq!(
        detect_frame_kind(radio.last_tx()).unwrap(),
        FrameKind::PollAck
    );

    let mut poll = [0u8; 127];
    let poll_len = encode_poll(
        0,
        tag_b.short_address,
        ShortAddress::BROADCAST,
        &targets,
        &mut poll,
    )
    .unwrap();
    radio.push_rx(&poll[..poll_len], DwTime::from_ticks(200), metrics());
    anchor.on_rx(&mut radio, 110, &mut [0u8; 127]).unwrap();
    assert_eq!(
        detect_frame_kind(radio.last_tx()).unwrap(),
        FrameKind::PollAck
    );

    // Both range frames must produce a range report, in either order.
    for (tag, now) in [(tag_a, 120u32), (tag_b, 130u32)] {
        let timings = [RangeTiming {
            short_address: anchor_identity.short_address,
            poll_sent: DwTime::from_ticks(90),
            poll_ack_received: DwTime::from_ticks(180),
            range_sent: DwTime::from_ticks(260),
        }];
        let mut range = [0u8; 127];
        let range_len = encode_range(
            1,
            tag.short_address,
            ShortAddress::BROADCAST,
            &timings,
            &mut range,
        )
        .unwrap();
        radio.push_rx(&range[..range_len], DwTime::from_ticks(300), metrics());
        let event = anchor.on_rx(&mut radio, now, &mut [0u8; 127]).unwrap();
        assert!(matches!(event, Some(RangingEvent::RangeUpdated(_))));
        assert_eq!(
            detect_frame_kind(radio.last_tx()).unwrap(),
            FrameKind::RangeReport
        );
    }
    assert_eq!(anchor.peers().count(), 2);
}

#[test]
fn anchor_ignores_a_range_that_arrives_after_the_exchange_expired() {
    let tag_identity = identity(0x1234, [14, 24, 34, 44, 54, 64, 74, 84]);
    let anchor_identity = identity(0x4321, [15, 25, 35, 45, 55, 65, 75, 85]);
    let mut config = RangingConfig::new(anchor_identity);
    config.reset_period_ms = 10_000;
    let mut anchor = RangingNode::<4>::new(Role::Anchor, config);
    let mut radio = MockRadio::default();

    anchor.start(&mut radio, 0).unwrap();
    let targets = [PollTarget {
        short_address: anchor_identity.short_address,
        reply_delay_us: 7_000,
    }];
    let mut poll = [0u8; 127];
    let poll_len = encode_poll(
        0,
        tag_identity.short_address,
        ShortAddress::BROADCAST,
        &targets,
        &mut poll,
    )
    .unwrap();
    radio.push_rx(&poll[..poll_len], DwTime::from_ticks(100), metrics());
    anchor.on_rx(&mut radio, 100, &mut [0u8; 127]).unwrap();

    // A range arriving long after the poll would mix timestamps from two
    // different exchanges and must be dropped without a report.
    let timings = [RangeTiming {
        short_address: anchor_identity.short_address,
        poll_sent: DwTime::from_ticks(90),
        poll_ack_received: DwTime::from_ticks(180),
        range_sent: DwTime::from_ticks(260),
    }];
    let mut range = [0u8; 127];
    let range_len = encode_range(
        1,
        tag_identity.short_address,
        ShortAddress::BROADCAST,
        &timings,
        &mut range,
    )
    .unwrap();
    let tx_count = radio.transmitted.len();
    radio.push_rx(&range[..range_len], DwTime::from_ticks(300), metrics());
    assert_eq!(
        anchor.on_rx(&mut radio, 500, &mut [0u8; 127]).unwrap(),
        None
    );
    assert_eq!(radio.transmitted.len(), tx_count);

    // The next poll starts a clean exchange.
    let mut poll = [0u8; 127];
    let poll_len = encode_poll(
        2,
        tag_identity.short_address,
        ShortAddress::BROADCAST,
        &targets,
        &mut poll,
    )
    .unwrap();
    radio.push_rx(&poll[..poll_len], DwTime::from_ticks(600), metrics());
    anchor.on_rx(&mut radio, 510, &mut [0u8; 127]).unwrap();
    assert_eq!(radio.transmitted.len(), tx_count + 1);
    assert_eq!(
        detect_frame_kind(radio.last_tx()).unwrap(),
        FrameKind::PollAck
    );
}

#[test]
fn tag_shares_the_frame_only_in_its_slot_after_schedule_sync() {
    let tag_identity = identity(0x1234, [16, 26, 36, 46, 56, 66, 76, 86]);
    let coordinator_short = ShortAddress::new(0x4321);
    let mut config = RangingConfig::new(tag_identity);
    config.schedule.tag_slot = 0;
    config.schedule.tag_slot_count = 2;
    config.schedule.tag_slot_ms = 100;
    config.schedule.session_timeout_ms = 30;
    let mut tag = RangingNode::<4>::new(Role::Tag, config);
    let mut radio = MockRadio::default();

    tag.start(&mut radio, 0).unwrap();

    // Before the coordinator schedule is known, the tag must stay silent.
    tag.tick(&mut radio, 80).unwrap();
    tag.tick(&mut radio, 160).unwrap();
    assert!(radio.transmitted.is_empty());

    let mut sync = [0u8; 127];
    let sync_len = encode_schedule_sync(
        0,
        coordinator_short,
        ShortAddress::BROADCAST,
        0,
        2,
        100,
        &mut sync,
    )
    .unwrap();
    radio.push_rx(&sync[..sync_len], DwTime::from_ticks(10), metrics());
    tag.on_rx(&mut radio, 200, &mut [0u8; 127]).unwrap();

    // 80 ms into the 100 ms slot there is no room left for an exchange.
    tag.tick(&mut radio, 280).unwrap();
    assert!(radio.transmitted.is_empty());

    // At the start of its slot the tag transmits (discovery blink).
    tag.tick(&mut radio, 400).unwrap();
    assert_eq!(radio.transmitted.len(), 1);
    assert_eq!(
        detect_frame_kind(radio.last_tx()).unwrap(),
        FrameKind::Blink
    );

    // The other tag's slot stays untouched.
    tag.tick(&mut radio, 500).unwrap();
    assert_eq!(radio.transmitted.len(), 1);
}

#[test]
fn broadcast_poll_reports_reply_delay_overflow() {
    let tag_identity = identity(0x1234, [33, 34, 35, 36, 37, 38, 39, 40]);
    let anchor_a = identity(0x4001, [41, 42, 43, 44, 45, 46, 47, 48]);
    let anchor_b = identity(0x4002, [49, 50, 51, 52, 53, 54, 55, 56]);
    let mut config = RangingConfig::new(tag_identity);
    config.schedule.session_timeout_ms = 20;
    let mut tag = RangingNode::<4>::new(Role::Tag, config);
    let mut radio = MockRadio::default();

    tag.start(&mut radio, 0).unwrap();
    tag.tick(&mut radio, 80).unwrap();

    for anchor in [anchor_a, anchor_b] {
        let mut init = [0u8; 127];
        let init_len =
            encode_ranging_init(0, anchor.short_address, tag_identity.eui, &mut init).unwrap();
        radio.push_rx(&init[..init_len], DwTime::from_ticks(20), metrics());
        tag.on_rx(&mut radio, 81, &mut [0u8; 127]).unwrap();
    }

    tag.tick(&mut radio, 160).unwrap();
    radio.next_timestamps.push_back(Timestamps {
        tx: DwTime::from_ticks(100),
        rx: DwTime::zero(),
        system: DwTime::zero(),
    });
    tag.on_tx_done(&mut radio).unwrap();

    let mut poll_ack = [0u8; 127];
    let poll_ack_len = encode_poll_ack(
        1,
        anchor_a.short_address,
        tag_identity.short_address,
        &mut poll_ack,
    )
    .unwrap();
    radio.push_rx(
        &poll_ack[..poll_ack_len],
        DwTime::from_ticks(200),
        metrics(),
    );
    tag.on_rx(&mut radio, 161, &mut [0u8; 127]).unwrap();

    radio.delayed_times.push_back(DwTime::from_ticks(260));
    tag.tick(&mut radio, 181).unwrap();
    let Frame::Range { payload, .. } = parse_frame(radio.last_tx()).unwrap() else {
        panic!("expected range frame");
    };
    let mut timings = [RangeTiming {
        short_address: ShortAddress::new(0),
        poll_sent: DwTime::zero(),
        poll_ack_received: DwTime::zero(),
        range_sent: DwTime::zero(),
    }; 4];
    let count = decode_range_timings(payload, &mut timings).unwrap();
    assert_eq!(count, 1);
    assert_eq!(timings[0].short_address, anchor_a.short_address);
}

#[test]
fn two_tags_use_distinct_tdma_slots() {
    let mut slot0_config = RangingConfig::new(identity(0x3400, [1, 1, 1, 1, 1, 1, 1, 1]));
    slot0_config.schedule.tag_slot = 0;
    slot0_config.schedule.tag_slot_count = 2;
    slot0_config.schedule.tag_slot_ms = 100;

    let mut slot1_config = RangingConfig::new(identity(0x3401, [2, 2, 2, 2, 2, 2, 2, 2]));
    slot1_config.schedule.tag_slot = 1;
    slot1_config.schedule.tag_slot_count = 2;
    slot1_config.schedule.tag_slot_ms = 100;

    let mut slot0 = RangingNode::<4>::new(Role::Tag, slot0_config);
    let mut slot1 = RangingNode::<4>::new(Role::Tag, slot1_config);
    let mut slot0_radio = MockRadio::default();
    let mut slot1_radio = MockRadio::default();

    slot0.start(&mut slot0_radio, 0).unwrap();
    slot1.start(&mut slot1_radio, 0).unwrap();

    let coordinator = identity(0x1111, [9, 9, 9, 9, 9, 9, 9, 9]);
    let mut sync = [0u8; 127];
    let sync_len = encode_schedule_sync(
        0,
        coordinator.short_address,
        ShortAddress::BROADCAST,
        0,
        2,
        100,
        &mut sync,
    )
    .unwrap();

    slot0_radio.push_rx(&sync[..sync_len], DwTime::from_ticks(10), metrics());
    slot0.on_rx(&mut slot0_radio, 0, &mut [0u8; 127]).unwrap();

    slot1_radio.push_rx(&sync[..sync_len], DwTime::from_ticks(10), metrics());
    slot1.on_rx(&mut slot1_radio, 0, &mut [0u8; 127]).unwrap();

    slot0.tick(&mut slot0_radio, 80).unwrap();
    slot1.tick(&mut slot1_radio, 80).unwrap();
    assert_eq!(slot0_radio.transmitted.len(), 1);
    assert_eq!(slot1_radio.transmitted.len(), 0);

    slot0.tick(&mut slot0_radio, 160).unwrap();
    slot1.tick(&mut slot1_radio, 160).unwrap();
    assert_eq!(slot0_radio.transmitted.len(), 1);
    assert_eq!(slot1_radio.transmitted.len(), 1);
}

#[test]
fn broadcast_poll_schedules_four_anchors_without_overflow() {
    let tag_identity = identity(0x1234, [33, 34, 35, 36, 37, 38, 39, 40]);
    let anchors = [
        identity(0x4001, [41, 42, 43, 44, 45, 46, 47, 48]),
        identity(0x4002, [49, 50, 51, 52, 53, 54, 55, 56]),
        identity(0x4003, [57, 58, 59, 60, 61, 62, 63, 64]),
        identity(0x4004, [65, 66, 67, 68, 69, 70, 71, 72]),
    ];
    let mut config = RangingConfig::new(tag_identity);
    config.reply_delay_us = 12_000;

    let mut tag = RangingNode::<4>::new(Role::Tag, config);
    let mut radio = MockRadio::default();

    tag.start(&mut radio, 0).unwrap();
    tag.tick(&mut radio, 80).unwrap();

    for (index, anchor) in anchors.iter().enumerate() {
        let mut init = [0u8; 127];
        let init_len =
            encode_ranging_init(0, anchor.short_address, tag_identity.eui, &mut init).unwrap();
        radio.push_rx(
            &init[..init_len],
            DwTime::from_ticks(20 + index as i64),
            metrics(),
        );
        tag.on_rx(&mut radio, 81 + index as u32, &mut [0u8; 127])
            .unwrap();
    }

    tag.tick(&mut radio, 160).unwrap();
    let Frame::Poll { payload, .. } = parse_frame(radio.last_tx()).unwrap() else {
        panic!("expected poll frame");
    };
    let mut targets = [PollTarget {
        short_address: ShortAddress::new(0),
        reply_delay_us: 0,
    }; 4];
    let count = decode_poll_targets(payload, &mut targets).unwrap();
    assert_eq!(count, 4);
    assert_eq!(
        targets.map(|target| target.reply_delay_us),
        [12_000, 36_000, 60_000, 84_000]
    );
}
