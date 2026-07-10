#![no_std]

mod app;
mod board;

use app::RangingApp;
use board::Board;
use dw1000_rs::{AntennaDelay, DeviceIdentity, RangingSchedule, Role};

const PEER_CAPACITY: usize = 4;
const RX_BUFFER_LEN: usize = 127;
const DW1000_SPI_BAUD_HZ: u32 = 3_000_000;
const STM6600_BOOT_TIMEOUT_MS: u64 = 1_000;
const DW1000_LINK_RECOVERY_TIMEOUT_MS: u32 = 1_000;
const RANGING_REPLY_DELAY_US: u32 = 12_000;
const PEER_INACTIVITY_TIMEOUT_MS: u32 = 3_000;
const FAULT_LED_ON_MS: u64 = 120;
const FAULT_LED_OFF_MS: u64 = 120;
const FAULT_LED_PAUSE_MS: u64 = 700;
const FAULT_BLINK_COUNT: usize = 3;

pub async fn run(
    role: Role,
    identity: DeviceIdentity,
    antenna_delay: AntennaDelay,
    schedule: RangingSchedule,
    anchor_is_coordinator: bool,
) -> ! {
    let board = match Board::init().await {
        Ok(board) => board,
        Err(error) => error.fault_loop().await,
    };

    let app = match RangingApp::<PEER_CAPACITY>::new(
        board,
        role,
        identity,
        antenna_delay,
        schedule,
        anchor_is_coordinator,
    )
    .await
    {
        Ok(app) => app,
        Err(error) => error.fault_loop().await,
    };

    app.run_forever().await
}
