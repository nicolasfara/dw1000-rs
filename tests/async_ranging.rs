#![allow(missing_docs)]

extern crate alloc;

use alloc::collections::VecDeque;

use dw1000_rs::protocol::{
    detect_frame_kind, encode_poll_ack, encode_range_report, encode_ranging_init, FrameKind,
    RangeReportPayload,
};
use dw1000_rs::ranging::AsyncRangingRadio;
use dw1000_rs::{
    DeviceIdentity, DwTime, Error, Eui64, PanId, ProtocolError, RangingConfig, RangingEvent,
    RangingNode, Role, RxFrame, RxOptions, ShortAddress, SignalMetrics, SysStatus, Timestamps,
    TxOptions,
};
use futures::executor::block_on;

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

impl AsyncRangingRadio<MockError, MockError> for MockRadio {
    async fn start_receive(
        &mut self,
        options: RxOptions,
    ) -> Result<(), Error<MockError, MockError>> {
        self.receive_modes.push(options);
        Ok(())
    }

    async fn transmit(
        &mut self,
        frame: &[u8],
        options: TxOptions,
    ) -> Result<(), Error<MockError, MockError>> {
        self.transmitted.push((frame.to_vec(), options));
        Ok(())
    }

    async fn read_frame<'a>(
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

    async fn read_timestamps(&mut self) -> Result<Timestamps, Error<MockError, MockError>> {
        Ok(self.next_timestamps.pop_front().unwrap())
    }

    async fn compute_delayed_time(
        &mut self,
        _delay: DwTime,
    ) -> Result<DwTime, Error<MockError, MockError>> {
        Ok(self.delayed_times.pop_front().unwrap())
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
    block_on(async {
        let tag_identity = identity(0x1234, [0x7D, 0x00, 0x22, 0xEA, 0x82, 0x60, 0x3B, 0x9C]);
        let anchor_identity = identity(0x4321, [0x82, 0x17, 0x5B, 0xD5, 0xA9, 0x9A, 0xE2, 0x9C]);
        let mut tag = RangingNode::<4>::new(Role::Tag, RangingConfig::new(tag_identity));
        let mut anchor = RangingNode::<4>::new(Role::Anchor, RangingConfig::new(anchor_identity));
        let mut tag_radio = MockRadio::default();
        let mut anchor_radio = MockRadio::default();

        tag.start_async(&mut tag_radio, 0).await.unwrap();
        anchor.start_async(&mut anchor_radio, 0).await.unwrap();

        tag.tick_async(&mut tag_radio, 80).await.unwrap();
        anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(10), metrics());
        let blink_event = anchor
            .on_rx_async(&mut anchor_radio, 81, &mut [0u8; 127])
            .await
            .unwrap();
        assert!(matches!(blink_event, Some(RangingEvent::BlinkReceived(_))));

        tag_radio.push_rx(anchor_radio.last_tx(), DwTime::from_ticks(20), metrics());
        let init_event = tag
            .on_rx_async(&mut tag_radio, 82, &mut [0u8; 127])
            .await
            .unwrap();
        assert_eq!(
            init_event,
            Some(RangingEvent::RangingInitReceived(ShortAddress::new(0x4321)))
        );

        tag.tick_async(&mut tag_radio, 160).await.unwrap();
        tag.tick_async(&mut tag_radio, 240).await.unwrap();
        tag_radio.next_timestamps.push_back(Timestamps {
            tx: DwTime::from_ticks(100),
            rx: DwTime::zero(),
            system: DwTime::zero(),
        });
        tag.on_tx_done_async(&mut tag_radio).await.unwrap();

        anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(120), metrics());
        anchor
            .on_rx_async(&mut anchor_radio, 241, &mut [0u8; 127])
            .await
            .unwrap();
        anchor_radio.next_timestamps.push_back(Timestamps {
            tx: DwTime::from_ticks(180),
            rx: DwTime::zero(),
            system: DwTime::zero(),
        });
        anchor.on_tx_done_async(&mut anchor_radio).await.unwrap();

        tag_radio.push_rx(anchor_radio.last_tx(), DwTime::from_ticks(200), metrics());
        tag_radio.delayed_times.push_back(DwTime::from_ticks(260));
        tag.on_rx_async(&mut tag_radio, 242, &mut [0u8; 127])
            .await
            .unwrap();
        tag_radio.next_timestamps.push_back(Timestamps {
            tx: DwTime::from_ticks(260),
            rx: DwTime::zero(),
            system: DwTime::zero(),
        });
        tag.on_tx_done_async(&mut tag_radio).await.unwrap();

        anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(280), metrics());
        let anchor_event = anchor
            .on_rx_async(&mut anchor_radio, 243, &mut [0u8; 127])
            .await
            .unwrap();
        let Some(RangingEvent::RangeUpdated(anchor_snapshot)) = anchor_event else {
            panic!("expected anchor range update");
        };

        tag_radio.push_rx(anchor_radio.last_tx(), DwTime::from_ticks(300), metrics());
        let tag_event = tag
            .on_rx_async(&mut tag_radio, 244, &mut [0u8; 127])
            .await
            .unwrap();
        let Some(RangingEvent::RangeUpdated(tag_snapshot)) = tag_event else {
            panic!("expected tag range update");
        };

        let expected_range = DwTime::from_ticks(20).as_meters();
        assert!((anchor_snapshot.range_m - expected_range).abs() < 1e-6);
        assert!((tag_snapshot.range_m - expected_range).abs() < 1e-6);
    });
}

#[test]
fn tick_prunes_inactive_peers() {
    block_on(async {
        let tag_identity = identity(0x1234, [0, 1, 2, 3, 4, 5, 6, 7]);
        let anchor_identity = identity(0x4321, [8, 9, 10, 11, 12, 13, 14, 15]);
        let mut tag = RangingNode::<4>::new(Role::Tag, RangingConfig::new(tag_identity));
        let mut anchor = RangingNode::<4>::new(Role::Anchor, RangingConfig::new(anchor_identity));
        let mut tag_radio = MockRadio::default();
        let mut anchor_radio = MockRadio::default();

        tag.start_async(&mut tag_radio, 0).await.unwrap();
        anchor.start_async(&mut anchor_radio, 0).await.unwrap();
        tag.tick_async(&mut tag_radio, 80).await.unwrap();
        anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(10), metrics());
        anchor
            .on_rx_async(&mut anchor_radio, 81, &mut [0u8; 127])
            .await
            .unwrap();
        tag_radio.push_rx(anchor_radio.last_tx(), DwTime::from_ticks(20), metrics());
        tag.on_rx_async(&mut tag_radio, 82, &mut [0u8; 127])
            .await
            .unwrap();

        let event = tag.tick_async(&mut tag_radio, 400).await.unwrap();
        assert_eq!(
            event,
            Some(RangingEvent::PeerInactive(ShortAddress::new(0x4321)))
        );
    });
}

#[test]
fn anchor_recovers_after_inactivity_and_accepts_rediscovery() {
    block_on(async {
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

        tag.start_async(&mut tag_radio, 0).await.unwrap();
        anchor.start_async(&mut anchor_radio, 0).await.unwrap();

        tag.tick_async(&mut tag_radio, 80).await.unwrap();
        anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(10), metrics());
        anchor
            .on_rx_async(&mut anchor_radio, 81, &mut [0u8; 127])
            .await
            .unwrap();
        tag_radio.push_rx(anchor_radio.last_tx(), DwTime::from_ticks(20), metrics());
        tag.on_rx_async(&mut tag_radio, 82, &mut [0u8; 127])
            .await
            .unwrap();

        tag.tick_async(&mut tag_radio, 160).await.unwrap();
        anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(30), metrics());
        anchor
            .on_rx_async(&mut anchor_radio, 161, &mut [0u8; 127])
            .await
            .unwrap();

        let anchor_timeout = anchor.tick_async(&mut anchor_radio, 300).await.unwrap();
        assert_eq!(
            anchor_timeout,
            Some(RangingEvent::PeerInactive(ShortAddress::new(0x1234)))
        );
        let tag_timeout = tag.tick_async(&mut tag_radio, 301).await.unwrap();
        assert_eq!(
            tag_timeout,
            Some(RangingEvent::PeerInactive(ShortAddress::new(0x4321)))
        );

        tag.tick_async(&mut tag_radio, 380).await.unwrap();
        anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(40), metrics());
        let blink_event = anchor
            .on_rx_async(&mut anchor_radio, 381, &mut [0u8; 127])
            .await
            .unwrap();
        assert!(matches!(blink_event, Some(RangingEvent::BlinkReceived(_))));

        tag_radio.push_rx(anchor_radio.last_tx(), DwTime::from_ticks(50), metrics());
        let init_event = tag
            .on_rx_async(&mut tag_radio, 382, &mut [0u8; 127])
            .await
            .unwrap();
        assert_eq!(
            init_event,
            Some(RangingEvent::RangingInitReceived(ShortAddress::new(0x4321)))
        );

        tag.tick_async(&mut tag_radio, 460).await.unwrap();
        anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(60), metrics());
        assert!(anchor
            .on_rx_async(&mut anchor_radio, 461, &mut [0u8; 127])
            .await
            .is_ok());
    });
}

#[test]
fn recover_link_preserves_peer_and_next_poll_is_accepted_without_rediscovery() {
    block_on(async {
        let tag_identity = identity(0x1234, [0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37]);
        let anchor_identity = identity(0x4321, [0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47]);
        let mut tag = RangingNode::<4>::new(Role::Tag, RangingConfig::new(tag_identity));
        let mut anchor = RangingNode::<4>::new(Role::Anchor, RangingConfig::new(anchor_identity));
        let mut tag_radio = MockRadio::default();
        let mut anchor_radio = MockRadio::default();

        tag.start_async(&mut tag_radio, 0).await.unwrap();
        anchor.start_async(&mut anchor_radio, 0).await.unwrap();

        tag.tick_async(&mut tag_radio, 80).await.unwrap();
        anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(10), metrics());
        anchor
            .on_rx_async(&mut anchor_radio, 81, &mut [0u8; 127])
            .await
            .unwrap();
        tag_radio.push_rx(anchor_radio.last_tx(), DwTime::from_ticks(20), metrics());
        let init_event = tag
            .on_rx_async(&mut tag_radio, 82, &mut [0u8; 127])
            .await
            .unwrap();
        assert_eq!(
            init_event,
            Some(RangingEvent::RangingInitReceived(ShortAddress::new(0x4321)))
        );

        let receive_calls_before = anchor_radio.receive_modes.len();
        assert_eq!(anchor.peers().count(), 1);
        anchor
            .recover_link_async(&mut anchor_radio, 120)
            .await
            .unwrap();
        assert_eq!(anchor.peers().count(), 1);
        assert_eq!(anchor_radio.receive_modes.len(), receive_calls_before + 1);
        assert_eq!(
            anchor_radio.receive_modes.last().copied(),
            Some(RxOptions {
                delayed_time: None,
                permanent: true,
            })
        );

        tag.tick_async(&mut tag_radio, 160).await.unwrap();
        anchor_radio.push_rx(tag_radio.last_tx(), DwTime::from_ticks(30), metrics());
        let tx_count_before = anchor_radio.transmitted.len();
        let event = anchor
            .on_rx_async(&mut anchor_radio, 161, &mut [0u8; 127])
            .await
            .unwrap();
        assert_eq!(event, None);
        assert_eq!(anchor_radio.transmitted.len(), tx_count_before + 1);
        assert_eq!(
            detect_frame_kind(anchor_radio.last_tx()).unwrap(),
            FrameKind::PollAck
        );
    });
}

#[test]
fn tag_ignores_poll_ack_for_other_destination() {
    block_on(async {
        let tag_identity = identity(0x1234, [1, 2, 3, 4, 5, 6, 7, 8]);
        let anchor_identity = identity(0x4321, [9, 10, 11, 12, 13, 14, 15, 16]);
        let mut tag = RangingNode::<4>::new(Role::Tag, RangingConfig::new(tag_identity));
        let mut radio = MockRadio::default();

        tag.start_async(&mut radio, 0).await.unwrap();
        tag.tick_async(&mut radio, 80).await.unwrap();

        let mut init = [0u8; 127];
        let init_len = encode_ranging_init(
            0,
            anchor_identity.short_address,
            tag_identity.eui,
            &mut init,
        )
        .unwrap();
        radio.push_rx(&init[..init_len], DwTime::from_ticks(20), metrics());
        let init_event = tag
            .on_rx_async(&mut radio, 81, &mut [0u8; 127])
            .await
            .unwrap();
        assert_eq!(
            init_event,
            Some(RangingEvent::RangingInitReceived(
                anchor_identity.short_address
            ))
        );

        tag.tick_async(&mut radio, 160).await.unwrap();
        radio.next_timestamps.push_back(Timestamps {
            tx: DwTime::from_ticks(100),
            rx: DwTime::zero(),
            system: DwTime::zero(),
        });
        tag.on_tx_done_async(&mut radio).await.unwrap();

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
        assert_eq!(
            tag.on_rx_async(&mut radio, 161, &mut [0u8; 127])
                .await
                .unwrap(),
            None
        );
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
        assert_eq!(
            tag.on_rx_async(&mut radio, 162, &mut [0u8; 127])
                .await
                .unwrap(),
            None
        );
        assert_eq!(
            detect_frame_kind(radio.last_tx()).unwrap(),
            FrameKind::Range
        );
    });
}

#[test]
fn stale_poll_ack_is_ignored_while_waiting_for_range_report() {
    block_on(async {
        let tag_identity = identity(0x1234, [17, 18, 19, 20, 21, 22, 23, 24]);
        let anchor_identity = identity(0x4321, [25, 26, 27, 28, 29, 30, 31, 32]);
        let mut tag = RangingNode::<4>::new(Role::Tag, RangingConfig::new(tag_identity));
        let mut radio = MockRadio::default();

        tag.start_async(&mut radio, 0).await.unwrap();
        tag.tick_async(&mut radio, 80).await.unwrap();

        let mut init = [0u8; 127];
        let init_len = encode_ranging_init(
            0,
            anchor_identity.short_address,
            tag_identity.eui,
            &mut init,
        )
        .unwrap();
        radio.push_rx(&init[..init_len], DwTime::from_ticks(20), metrics());
        tag.on_rx_async(&mut radio, 81, &mut [0u8; 127])
            .await
            .unwrap();

        tag.tick_async(&mut radio, 160).await.unwrap();
        radio.next_timestamps.push_back(Timestamps {
            tx: DwTime::from_ticks(100),
            rx: DwTime::zero(),
            system: DwTime::zero(),
        });
        tag.on_tx_done_async(&mut radio).await.unwrap();

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
        tag.on_rx_async(&mut radio, 161, &mut [0u8; 127])
            .await
            .unwrap();
        radio.next_timestamps.push_back(Timestamps {
            tx: DwTime::from_ticks(260),
            rx: DwTime::zero(),
            system: DwTime::zero(),
        });
        tag.on_tx_done_async(&mut radio).await.unwrap();

        let tx_count_before = radio.transmitted.len();
        radio.push_rx(
            &poll_ack[..poll_ack_len],
            DwTime::from_ticks(200),
            metrics(),
        );
        assert_eq!(
            tag.on_rx_async(&mut radio, 162, &mut [0u8; 127])
                .await
                .unwrap(),
            None
        );
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
            tag.on_rx_async(&mut radio, 163, &mut [0u8; 127])
                .await
                .unwrap(),
            Some(RangingEvent::RangeUpdated(_))
        ));
    });
}

#[test]
fn broadcast_poll_reports_reply_delay_overflow() {
    block_on(async {
        let tag_identity = identity(0x1234, [33, 34, 35, 36, 37, 38, 39, 40]);
        let anchor_a = identity(0x4001, [41, 42, 43, 44, 45, 46, 47, 48]);
        let anchor_b = identity(0x4002, [49, 50, 51, 52, 53, 54, 55, 56]);
        let mut config = RangingConfig::new(tag_identity);
        config.reply_delay_us = 40_000;

        let mut tag = RangingNode::<2>::new(Role::Tag, config);
        let mut radio = MockRadio::default();

        tag.start_async(&mut radio, 0).await.unwrap();
        tag.tick_async(&mut radio, 80).await.unwrap();

        let mut init = [0u8; 127];
        let init_len =
            encode_ranging_init(0, anchor_a.short_address, tag_identity.eui, &mut init).unwrap();
        radio.push_rx(&init[..init_len], DwTime::from_ticks(20), metrics());
        tag.on_rx_async(&mut radio, 81, &mut [0u8; 127])
            .await
            .unwrap();

        let mut init = [0u8; 127];
        let init_len =
            encode_ranging_init(0, anchor_b.short_address, tag_identity.eui, &mut init).unwrap();
        radio.push_rx(&init[..init_len], DwTime::from_ticks(21), metrics());
        tag.on_rx_async(&mut radio, 82, &mut [0u8; 127])
            .await
            .unwrap();

        let error = tag.tick_async(&mut radio, 160).await.unwrap_err();
        assert_eq!(error, Error::Protocol(ProtocolError::ReplyDelayOverflow));
    });
}
