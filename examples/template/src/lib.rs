#![no_std]

// Placeholder timestamp source for defmt RTT logging. Replace this with a real
// monotonic counter if you want meaningful timestamps in the example output.
defmt::timestamp!("{=u32}", 0);

pub mod common;
pub mod compat;
pub mod platform;
