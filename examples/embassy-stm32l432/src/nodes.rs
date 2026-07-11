//! Per-node configuration for the multi-anchor, multi-tag example.
//!
//! Concrete values are generated at build time by `build.rs` from the
//! `DW1000_ANCHOR_*` and `DW1000_TAG_*` environment variables and included
//! into the `anchor` and `tag` binaries.

use dw1000_rs::{AntennaDelay, DeviceIdentity};

/// Per-node values that must be unique or calibrated for each physical board.
#[derive(Clone, Copy)]
pub struct NodeConfig {
    /// IEEE 802.15.4 identity programmed into the DW1000.
    pub identity: DeviceIdentity,
    /// Symmetric TX/RX antenna delay for this board.
    pub antenna_delay: AntennaDelay,
    /// Delayed response slot used only when this node is an anchor.
    pub discovery_reply_delay_us: u16,
    /// Enables periodic TDMA schedule broadcasts for the coordinator anchor.
    pub anchor_is_coordinator: bool,
    /// TDMA slot index used only when this node is a tag.
    pub tag_slot: u8,
}
