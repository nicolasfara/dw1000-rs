#![no_std]
#![deny(unsafe_code)]
#![warn(missing_docs)]

//! Embedded Rust driver for the Decawave DW1000 UWB transceiver
//!
//! This crate provides a no_std driver for the DW1000 Ultra-Wideband (UWB)
//! transceiver chip, suitable for use in embedded systems.

#[cfg(feature = "defmt")]
use defmt;

pub mod config;
pub mod constants;
mod device;
pub mod time;
// pub mod hl;
// pub mod device;
// pub mod mac;
// pub mod ranging;

pub use config::*;
pub use constants::DeviceMode;
pub use time::DW1000Time;
// pub use hl::DW1000;
// pub use device::DW1000Device;
// pub use mac::DW1000Mac;
// pub use ranging::{DW1000Ranging, MessageType, DeviceType, RangingEvent};
