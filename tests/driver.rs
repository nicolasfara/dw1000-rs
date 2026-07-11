#![allow(missing_docs)]

extern crate alloc;

use alloc::collections::VecDeque;
use alloc::sync::Arc;
use core::convert::Infallible;
use std::sync::Mutex;

use dw1000_rs::{
    AddressConfig, AntennaDelay, Channel, DataRate, DeviceIdentity, Dw1000, DwTime, Eui64, PanId,
    PreambleLength, PulseFrequency, RadioConfig, RxError, RxOptions, ShortAddress, SysStatus,
    TxOptions,
};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{ErrorType as DigitalErrorType, InputPin, OutputPin};
use embedded_hal::spi::{
    Error as SpiErrorTrait, ErrorKind as SpiErrorKind, ErrorType, Operation, SpiDevice,
};

struct NoopDelay;

impl DelayNs for NoopDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct MockSpiError;

impl SpiErrorTrait for MockSpiError {
    fn kind(&self) -> SpiErrorKind {
        SpiErrorKind::Other
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct LoggedTransaction {
    writes: Vec<Vec<u8>>,
    reads: Vec<Vec<u8>>,
}

#[derive(Debug, Clone)]
struct RecordingSpi {
    log: Arc<Mutex<Vec<LoggedTransaction>>>,
    reads: VecDeque<Vec<u8>>,
}

impl RecordingSpi {
    fn new(log: Arc<Mutex<Vec<LoggedTransaction>>>, reads: VecDeque<Vec<u8>>) -> Self {
        Self { log, reads }
    }
}

impl ErrorType for RecordingSpi {
    type Error = MockSpiError;
}

impl SpiDevice<u8> for RecordingSpi {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        let mut transaction = LoggedTransaction {
            writes: Vec::new(),
            reads: Vec::new(),
        };
        for operation in operations {
            match operation {
                Operation::Write(data) => transaction.writes.push(data.to_vec()),
                Operation::Read(buffer) => {
                    let response = self
                        .reads
                        .pop_front()
                        .unwrap_or_else(|| vec![0; buffer.len()]);
                    assert_eq!(response.len(), buffer.len());
                    buffer.copy_from_slice(&response);
                    transaction.reads.push(response);
                }
                Operation::Transfer(read, write) => {
                    let response = self
                        .reads
                        .pop_front()
                        .unwrap_or_else(|| vec![0; read.len()]);
                    assert_eq!(response.len(), read.len());
                    read.copy_from_slice(&response);
                    transaction.writes.push(write.to_vec());
                    transaction.reads.push(response);
                }
                Operation::TransferInPlace(buffer) => {
                    let response = self
                        .reads
                        .pop_front()
                        .unwrap_or_else(|| vec![0; buffer.len()]);
                    assert_eq!(response.len(), buffer.len());
                    buffer.copy_from_slice(&response);
                    transaction.reads.push(response);
                }
                Operation::DelayNs(_) => {}
            }
        }
        self.log.lock().unwrap().push(transaction);
        Ok(())
    }
}

#[derive(Debug, Default, Clone)]
struct MockInputPin;

impl DigitalErrorType for MockInputPin {
    type Error = Infallible;
}

impl InputPin for MockInputPin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(false)
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(true)
    }
}

#[derive(Debug, Default, Clone)]
struct MockOutputPin;

impl DigitalErrorType for MockOutputPin {
    type Error = Infallible;
}

impl OutputPin for MockOutputPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

fn identity() -> DeviceIdentity {
    DeviceIdentity::new(
        PanId::new(0xDECA),
        ShortAddress::new(0x1234),
        Eui64::new([0x82, 0x17, 0x5B, 0xD5, 0xA9, 0x9A, 0xE2, 0x9C]),
    )
}

fn config() -> RadioConfig {
    RadioConfig {
        address: AddressConfig {
            identity: identity(),
        },
        phy: dw1000_rs::PhyConfig {
            data_rate: DataRate::Kbps110,
            pulse_frequency: PulseFrequency::Mhz16,
            preamble_length: PreambleLength::Symbols2048,
            channel: Channel::Channel5,
            preamble_code: None,
            smart_power: false,
        },
        antenna_delay: AntennaDelay::LEGACY_DEFAULT,
        receiver_auto_reenable: true,
        interrupt_polarity_high: true,
        frame_check: true,
    }
}

#[test]
fn reconfigure_programs_identity_registers() {
    let log = Arc::new(Mutex::new(Vec::new()));
    let spi = RecordingSpi::new(log.clone(), VecDeque::from(vec![vec![0; 4]]));
    let mut driver = Dw1000::new(spi, MockInputPin, MockOutputPin);
    driver.reconfigure(&config()).unwrap();

    let transactions = log.lock().unwrap();
    assert!(transactions.iter().any(|transaction| {
        transaction.writes
            == vec![
                vec![0x81],
                vec![0x9C, 0xE2, 0x9A, 0xA9, 0xD5, 0x5B, 0x17, 0x82],
            ]
    }));
    assert!(transactions.iter().any(|transaction| {
        transaction.writes == vec![vec![0x83], vec![0x34, 0x12, 0xCA, 0xDE]]
    }));
}

#[test]
fn transmit_writes_buffer_control_and_start_bits() {
    let log = Arc::new(Mutex::new(Vec::new()));
    let spi = RecordingSpi::new(log.clone(), VecDeque::new());
    let mut driver = Dw1000::new(spi, MockInputPin, MockOutputPin);

    driver
        .transmit(&[0xAA, 0xBB, 0xCC], TxOptions::default())
        .unwrap();

    let transactions = log.lock().unwrap();
    assert!(transactions
        .iter()
        .any(|transaction| transaction.writes == vec![vec![0x89], vec![0xAA, 0xBB, 0xCC]]));
    assert!(transactions
        .iter()
        .any(|transaction| transaction.writes == vec![vec![0x88], vec![5, 0, 0, 0, 0]]));
    assert!(transactions
        .iter()
        .any(|transaction| transaction.writes == vec![vec![0x8D], vec![0x02, 0, 0, 0]]));
}

#[test]
fn read_sys_status_decodes_bits() {
    let log = Arc::new(Mutex::new(Vec::new()));
    let spi = RecordingSpi::new(log, VecDeque::from(vec![vec![0x80, 0x40, 0, 0, 0]]));
    let mut driver = Dw1000::new(spi, MockInputPin, MockOutputPin);
    let status = driver.read_sys_status().unwrap();
    assert_eq!(status, SysStatus((1 << 7) | (1 << 14)));
}

#[test]
fn read_frame_rejects_non_rx_status() {
    let log = Arc::new(Mutex::new(Vec::new()));
    let spi = RecordingSpi::new(
        log,
        VecDeque::from(vec![
            vec![0; 4],
            dw1000_rs::registers::sys_status_to_bytes(dw1000_rs::registers::status::TX_FRAME_SENT)
                .to_vec(),
        ]),
    );
    let mut driver = Dw1000::new(spi, MockInputPin, MockOutputPin);
    driver.reconfigure(&config()).unwrap();

    let error = driver.read_frame(&mut [0u8; 16]).unwrap_err();
    assert_eq!(error, dw1000_rs::Error::Receive(RxError::FrameNotReady));
}

#[test]
fn read_signal_metrics_before_config_returns_not_configured() {
    let log = Arc::new(Mutex::new(Vec::new()));
    let spi = RecordingSpi::new(log, VecDeque::new());
    let mut driver = Dw1000::new(spi, MockInputPin, MockOutputPin);

    let error = driver.read_signal_metrics().unwrap_err();
    assert_eq!(error, dw1000_rs::Error::NotConfigured);
}

#[test]
fn clear_tx_sent_restarts_receive_when_permanent_mode_is_enabled() {
    let log = Arc::new(Mutex::new(Vec::new()));
    // The live SYS_STATUS check must still report the completed transmission.
    let reads = VecDeque::from(vec![vec![
        (dw1000_rs::registers::status::TX_FRAME_SENT.0 & 0xFF) as u8,
    ]]);
    let spi = RecordingSpi::new(log.clone(), reads);
    let mut driver = Dw1000::new(spi, MockInputPin, MockOutputPin);

    driver
        .start_receive(RxOptions {
            delayed_time: None,
            permanent: true,
        })
        .unwrap();
    driver
        .transmit(&[0xAA, 0xBB, 0xCC], TxOptions::default())
        .unwrap();
    driver
        .clear_events(dw1000_rs::registers::status::TX_FRAME_SENT)
        .unwrap();

    let transactions = log.lock().unwrap();
    let restart_count = transactions
        .iter()
        .filter(|transaction| transaction.writes == vec![vec![0x8D], vec![0x00, 0x01, 0x00, 0x00]])
        .count();
    assert_eq!(restart_count, 2);
}

#[test]
fn clear_good_receive_restarts_single_buffered_permanent_receive() {
    let log = Arc::new(Mutex::new(Vec::new()));
    let spi = RecordingSpi::new(log.clone(), VecDeque::new());
    let mut driver = Dw1000::new(spi, MockInputPin, MockOutputPin);

    driver
        .start_receive(RxOptions {
            delayed_time: None,
            permanent: true,
        })
        .unwrap();
    driver
        .clear_events(
            dw1000_rs::registers::status::RX_FRAME_READY
                | dw1000_rs::registers::status::RX_FRAME_GOOD,
        )
        .unwrap();

    let transactions = log.lock().unwrap();
    let restart_count = transactions
        .iter()
        .filter(|transaction| transaction.writes == vec![vec![0x8D], vec![0x00, 0x01, 0x00, 0x00]])
        .count();
    assert_eq!(restart_count, 2);
}

#[test]
fn clear_stale_tx_sent_does_not_cancel_a_pending_delayed_transmission() {
    let log = Arc::new(Mutex::new(Vec::new()));
    // TXFRS reads back clear: the mask carries a stale bit while a new
    // delayed transmission is still pending, so the receiver must not be
    // restarted (TRXOFF would cancel the pending send).
    let spi = RecordingSpi::new(log.clone(), VecDeque::new());
    let mut driver = Dw1000::new(spi, MockInputPin, MockOutputPin);

    driver
        .start_receive(RxOptions {
            delayed_time: None,
            permanent: true,
        })
        .unwrap();
    driver
        .transmit(&[0xAA, 0xBB, 0xCC], TxOptions::default())
        .unwrap();
    driver
        .clear_events(dw1000_rs::registers::status::TX_FRAME_SENT)
        .unwrap();

    let transactions = log.lock().unwrap();
    let restart_count = transactions
        .iter()
        .filter(|transaction| transaction.writes == vec![vec![0x8D], vec![0x00, 0x01, 0x00, 0x00]])
        .count();
    assert_eq!(restart_count, 1);
}

#[test]
fn clearing_rx_event_does_not_cancel_pending_delayed_tx() {
    let log = Arc::new(Mutex::new(Vec::new()));
    let spi = RecordingSpi::new(log.clone(), VecDeque::new());
    let mut driver = Dw1000::new(spi, MockInputPin, MockOutputPin);

    driver
        .start_receive(RxOptions {
            delayed_time: None,
            permanent: true,
        })
        .unwrap();
    let scheduled = driver
        .schedule_delayed(DwTime::from_micros(7000.0))
        .unwrap();
    driver
        .transmit(
            &[0xAA, 0xBB, 0xCC],
            TxOptions {
                delayed_time: Some(scheduled),
                wait_for_response: false,
            },
        )
        .unwrap();
    driver
        .clear_events(
            dw1000_rs::registers::status::RX_FRAME_READY
                | dw1000_rs::registers::status::RX_FRAME_GOOD,
        )
        .unwrap();

    let transactions = log.lock().unwrap();
    let restart_count = transactions
        .iter()
        .filter(|transaction| transaction.writes == vec![vec![0x8D], vec![0x00, 0x01, 0x00, 0x00]])
        .count();
    assert_eq!(restart_count, 1);
}

#[test]
fn start_receive_clears_stale_rx_events() {
    let log = Arc::new(Mutex::new(Vec::new()));
    let spi = RecordingSpi::new(log.clone(), VecDeque::new());
    let mut driver = Dw1000::new(spi, MockInputPin, MockOutputPin);

    driver
        .start_receive(RxOptions {
            delayed_time: None,
            permanent: true,
        })
        .unwrap();

    let expected =
        dw1000_rs::registers::sys_status_to_bytes(dw1000_rs::registers::status::ALL_RX_EVENTS);
    let transactions = log.lock().unwrap();
    assert!(transactions
        .iter()
        .any(|transaction| { transaction.writes == vec![vec![0x8F], expected.to_vec()] }));
}

#[test]
fn delayed_transmit_programs_dx_time_without_antenna_delay() {
    let log = Arc::new(Mutex::new(Vec::new()));
    // SYS_TIME read consumed by schedule_delayed.
    let spi = RecordingSpi::new(
        log.clone(),
        VecDeque::from(vec![vec![0x34, 0x12, 0x00, 0x00, 0x00]]),
    );
    let mut driver = Dw1000::new(spi, MockInputPin, MockOutputPin);

    let delay = DwTime::from_micros(7000.0);
    let scheduled = driver.schedule_delayed(delay).unwrap();

    // DX_TIME must be (SYS_TIME + delay) with the low 9 bits cleared and no
    // antenna delay; the prediction adds the (default) antenna delay on top.
    let mut expected = DwTime::from_bytes(&[0x34, 0x12, 0x00, 0x00, 0x00]) + delay;
    let mut bytes = expected.to_bytes();
    bytes[0] = 0;
    bytes[1] &= 0xFE;
    expected = DwTime::from_bytes(&bytes);
    assert_eq!(scheduled.dx_time(), expected);
    assert_eq!(scheduled.dx_time().ticks() & 0x1FF, 0);
    assert_eq!(
        scheduled.predicted_tx_timestamp(),
        expected + DwTime::from_ticks(16_456)
    );

    driver
        .transmit(
            &[0xAA, 0xBB, 0xCC],
            TxOptions {
                delayed_time: Some(scheduled),
                wait_for_response: false,
            },
        )
        .unwrap();

    let transactions = log.lock().unwrap();
    assert!(transactions
        .iter()
        .any(|transaction| transaction.writes
            == vec![vec![0x8A], scheduled.dx_time().to_bytes().to_vec()]));
}

#[test]
fn reconfigure_programs_lde_cfg1_and_sfd_timeout() {
    let log = Arc::new(Mutex::new(Vec::new()));
    let spi = RecordingSpi::new(log.clone(), VecDeque::from(vec![vec![0; 4]]));
    let mut driver = Dw1000::new(spi, MockInputPin, MockOutputPin);
    driver.reconfigure(&config()).unwrap();

    let transactions = log.lock().unwrap();
    // LDE_CFG1 (0x2E:0x0806) must carry PEAK_MULTPLIER | N_STD_FACTOR = 0x6D.
    assert!(transactions
        .iter()
        .any(|transaction| transaction.writes == vec![vec![0xEE, 0x86, 0x10], vec![0x6D]]));
    // DRX_SFDTOC (0x27:0x20): 2048-symbol preamble + 1 + 64 SFD - 64 PAC.
    assert!(transactions
        .iter()
        .any(|transaction| transaction.writes == vec![vec![0xE7, 0x20], vec![0x01, 0x08]]));
}

#[test]
fn init_rejects_wrong_device_id() {
    let log = Arc::new(Mutex::new(Vec::new()));
    let spi = RecordingSpi::new(
        log,
        VecDeque::from(vec![vec![0x11, 0x22, 0x33, 0x44]]),
    );
    let mut driver = Dw1000::new(spi, MockInputPin, MockOutputPin);

    let error = driver.init(&mut NoopDelay, &config()).unwrap_err();
    assert_eq!(error, dw1000_rs::Error::InvalidDeviceId(0x4433_2211));
}

#[test]
fn read_frame_error_resets_receiver() {
    let log = Arc::new(Mutex::new(Vec::new()));
    let reads = VecDeque::from(vec![
        vec![0; 4],
        dw1000_rs::registers::sys_status_to_bytes(
            dw1000_rs::registers::status::RX_FRAME_CHECK_ERROR,
        )
        .to_vec(),
    ]);
    let spi = RecordingSpi::new(log.clone(), reads);
    let mut driver = Dw1000::new(spi, MockInputPin, MockOutputPin);
    driver.reconfigure(&config()).unwrap();

    let error = driver.read_frame(&mut [0u8; 16]).unwrap_err();
    assert_eq!(error, dw1000_rs::Error::Receive(RxError::FrameCheck));

    // Errata recovery: PMSC SOFTRESET must pulse the RX reset (0xE0 then 0xF0).
    let transactions = log.lock().unwrap();
    assert!(transactions
        .iter()
        .any(|transaction| transaction.writes == vec![vec![0xF6, 0x03], vec![0xE0]]));
    assert!(transactions
        .iter()
        .any(|transaction| transaction.writes == vec![vec![0xF6, 0x03], vec![0xF0]]));
}
