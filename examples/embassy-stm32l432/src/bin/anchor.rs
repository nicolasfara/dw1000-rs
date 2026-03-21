#![no_main]
#![no_std]

use dw1000_embassy_stm32l432::run;
use dw1000_rs::{AntennaDelay, DeviceIdentity, Eui64, PanId, Role, ShortAddress};
use {defmt_rtt as _, panic_probe as _};

const ANCHOR_ANTENNA_DELAY: AntennaDelay = AntennaDelay::new(16_456);

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    run(
        Role::Anchor,
        DeviceIdentity::new(
            PanId::new(0x0D57),
            ShortAddress::new(3344),
            Eui64::new([0xB1, 0x4A, 0x7C, 0x00, 0x11, 0x22, 0x33, 0x44]),
        ),
        ANCHOR_ANTENNA_DELAY,
    )
    .await
}
