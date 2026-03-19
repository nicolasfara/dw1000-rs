#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt_rtt as _;
use dw1000_rs::{AntennaDelay, DeviceIdentity, Eui64, PanId, Role, ShortAddress};
use dw1000_template::common::run_with_antenna_delay;
use panic_halt as _;

// Tune this value on real hardware against a known tag-to-anchor distance.
const ANCHOR_ANTENNA_DELAY: AntennaDelay = AntennaDelay::new(16_456);

#[entry]
fn main() -> ! {
    run_with_antenna_delay(
        Role::Anchor,
        DeviceIdentity::new(
            PanId::new(0x0D57),
            ShortAddress::new(3344),
            Eui64::new([0xB1, 0x4A, 0x7C, 0x00, 0x11, 0x22, 0x33, 0x44]),
        ),
        ANCHOR_ANTENNA_DELAY,
    )
}
