#![no_main]
#![no_std]

use dw1000_embassy_stm32l432::run;
use dw1000_rs::{AntennaDelay, DeviceIdentity, Eui64, PanId, RangingSchedule, Role, ShortAddress};
use {defmt_rtt as _, panic_probe as _};

include!(concat!(env!("OUT_DIR"), "/tag_identity.rs"));

const TAG_ANTENNA_DELAY: AntennaDelay = AntennaDelay::new(16_456);

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    run(
        Role::Tag,
        DeviceIdentity::new(
            PanId::new(TAG_PAN_ID),
            ShortAddress::new(TAG_SHORT_ADDRESS),
            Eui64::new(TAG_EUI),
        ),
        TAG_ANTENNA_DELAY,
        RangingSchedule {
            anchor_slot: 0,
            discovery_slot_spacing_us: DISCOVERY_SLOT_SPACING_US,
            tag_slot: TAG_SLOT,
            tag_slot_count: TAG_SLOT_COUNT,
            tag_slot_ms: TAG_SLOT_MS,
            session_timeout_ms: SESSION_TIMEOUT_MS,
            range_period_ms: RANGE_PERIOD_MS,
        },
        false,
    )
    .await
}
