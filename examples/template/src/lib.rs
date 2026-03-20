#![no_std]

// Shared RTT timestamp source fed by the STM32 platform timebase.
defmt::timestamp!("{=u32}", crate::platform::defmt_timestamp_ms());

pub mod common;
pub mod compat;
pub mod platform;
