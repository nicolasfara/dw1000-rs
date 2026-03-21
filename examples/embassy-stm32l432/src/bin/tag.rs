#![no_main]
#![no_std]

use dw1000_embassy_stm32l432::run;
use dw1000_rs::{AntennaDelay, DeviceIdentity, Eui64, PanId, Role, ShortAddress};
use {defmt_rtt as _, panic_probe as _};

const TAG_ANTENNA_DELAY: AntennaDelay = AntennaDelay::new(16_456);

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    run(
        Role::Tag,
        DeviceIdentity::new(
            PanId::new(0x0D57),
            ShortAddress::new(3345),
            Eui64::new([0x82, 0x17, 0x5B, 0xD5, 0xA9, 0x9A, 0xE2, 0x9C]),
        ),
        TAG_ANTENNA_DELAY,
    )
    .await
}
