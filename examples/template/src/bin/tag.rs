#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt_rtt as _;
use dw1000_rs::{AntennaDelay, DeviceIdentity, Eui64, PanId, Role, ShortAddress};
use dw1000_template::common::run_with_antenna_delay;
use panic_halt as _;

// Tune this value on real hardware against a known tag-to-anchor distance.
const TAG_ANTENNA_DELAY: AntennaDelay = AntennaDelay::new(16_456);

#[entry]
fn main() -> ! {
    run_with_antenna_delay(
        Role::Tag,
        DeviceIdentity::new(
            PanId::new(0x0D57),
            ShortAddress::new(3345),
            Eui64::new([0x82, 0x17, 0x5B, 0xD5, 0xA9, 0x9A, 0xE2, 0x9C]),
        ),
        TAG_ANTENNA_DELAY,
    )
}
