#![no_main]
#![no_std]

use dw1000_embassy_stm32l432::{nodes::NodeConfig, run};
use dw1000_rs::{AntennaDelay, DeviceIdentity, Eui64, PanId, Role, ShortAddress};
use {defmt_rtt as _, panic_probe as _};

include!(concat!(env!("OUT_DIR"), "/anchor_config.rs"));

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    run(Role::Anchor, ANCHOR_CONFIG).await
}
