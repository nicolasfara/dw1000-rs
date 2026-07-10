#![no_main]
#![no_std]

use dw1000_embassy_stm32l432::run;
use dw1000_rs::{AntennaDelay, DeviceIdentity, Eui64, PanId, RangingSchedule, Role, ShortAddress};
use {defmt_rtt as _, panic_probe as _};

include!(concat!(env!("OUT_DIR"), "/anchor_identity.rs"));

const ANCHOR_ANTENNA_DELAY: AntennaDelay = AntennaDelay::new(16_456);

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    run(
        Role::Anchor,
        DeviceIdentity::new(
            PanId::new(ANCHOR_PAN_ID),
            ShortAddress::new(ANCHOR_SHORT_ADDRESS),
            Eui64::new(ANCHOR_EUI),
        ),
        ANCHOR_ANTENNA_DELAY,
        RangingSchedule {
            anchor_slot: ANCHOR_SLOT,
            discovery_slot_spacing_us: DISCOVERY_SLOT_SPACING_US,
            tag_slot: 0,
            tag_slot_count: TAG_SLOT_COUNT,
            tag_slot_ms: TAG_SLOT_MS,
            session_timeout_ms: SESSION_TIMEOUT_MS,
            range_period_ms: RANGE_PERIOD_MS,
        },
        ANCHOR_COORDINATOR,
    )
    .await
}
