//! DW1000 timestamp handling module
//!
//! This module provides the `DW1000Time` type for handling timestamps from the
//! DW1000 UWB transceiver. The DW1000 uses 40-bit timestamps where each bit
//! represents approximately 15.65 picoseconds.

use core::ops::{Add, AddAssign, Sub, SubAssign};

#[cfg(feature = "defmt")]
use defmt::Format;

/// Duration of one DW1000 timestamp tick in microseconds (~15.65 ps).
pub const TIME_RES: f32 = 0.000_015_650_041;

/// Ticks per microsecond (inverse of `TIME_RES`).
pub const TIME_RES_INV: f32 = 63897.6;

/// Distance represented by one DW1000 timestamp tick, in meters (c * 15.65 ps).
pub const DISTANCE_PER_TICK_M: f32 = 0.004_691_764;

/// Timestamp byte length - 40 bit -> 5 bytes
pub const LENGTH_TIMESTAMP: usize = 5;

/// Timer/counter overflow (40 bits) -> overflow approx. every 17.2 seconds
pub const TIME_OVERFLOW: i64 = 0x0100_0000_0000;

/// Maximum valid timestamp value (40-bit max)
pub const TIME_MAX: i64 = 0xFF_FFFF_FFFF;

/// Represents a timestamp from the DW1000 UWB transceiver.
///
/// The DW1000 uses 40-bit timestamps where each increment represents
/// approximately 15.65 picoseconds. This structure provides methods
/// to convert between raw timestamps and real-world time units.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct DW1000Time {
    /// Internal timestamp value (40-bit, but stored as i64 for calculations)
    timestamp: i64,
}

impl DW1000Time {
    /// Creates a zero-valued timestamp.
    #[inline]
    pub const fn zero() -> Self {
        Self { timestamp: 0 }
    }

    /// Creates a new `DW1000Time` from raw DW1000 ticks (1 tick ≈ 15.65 ps).
    #[inline]
    pub const fn from_ticks(ticks: i64) -> Self {
        Self { timestamp: ticks }
    }

    /// Creates a new `DW1000Time` from a 40-bit little-endian byte array.
    pub fn from_bytes(data: &[u8; LENGTH_TIMESTAMP]) -> Self {
        let mut timestamp = 0i64;
        for (index, &byte) in data.iter().enumerate() {
            timestamp |= (byte as i64) << (index * 8);
        }
        Self { timestamp }
    }

    /// Creates a new `DW1000Time` from microseconds.
    #[inline]
    pub fn from_micros(time_us: f32) -> Self {
        Self {
            timestamp: (time_us * TIME_RES_INV) as i64,
        }
    }

    /// Returns the raw tick value.
    #[inline]
    pub const fn ticks(&self) -> i64 {
        self.timestamp
    }

    /// Returns the timestamp encoded as the DW1000 40-bit little-endian wire format.
    pub fn to_bytes(&self) -> [u8; LENGTH_TIMESTAMP] {
        let mut data = [0u8; LENGTH_TIMESTAMP];
        for (index, byte) in data.iter_mut().enumerate() {
            *byte = ((self.timestamp >> (index * 8)) & 0xFF) as u8;
        }
        data
    }

    /// Returns the time in microseconds
    #[inline]
    pub fn as_microseconds(&self) -> f32 {
        (self.timestamp % TIME_OVERFLOW) as f32 * TIME_RES
    }

    /// Returns the time as distance in meters (d = c * t)
    ///
    /// This is useful for time-of-flight calculations
    #[inline]
    pub fn as_meters(&self) -> f32 {
        (self.timestamp % TIME_OVERFLOW) as f32 * DISTANCE_PER_TICK_M
    }

    /// Computes asymmetric two-way-ranging time of flight from the six protocol timestamps.
    pub fn asymmetric_tof(
        poll_sent: Self,
        poll_received: Self,
        poll_ack_sent: Self,
        poll_ack_received: Self,
        range_sent: Self,
        range_received: Self,
    ) -> Self {
        let round1 = (poll_ack_received - poll_sent).wrapped().timestamp as i128;
        let reply1 = (poll_ack_sent - poll_received).wrapped().timestamp as i128;
        let round2 = (range_received - poll_ack_sent).wrapped().timestamp as i128;
        let reply2 = (range_sent - poll_ack_received).wrapped().timestamp as i128;
        let denominator = round1 + round2 + reply1 + reply2;
        if denominator == 0 {
            return Self::zero();
        }
        Self::from_ticks(((round1 * round2 - reply1 * reply2) / denominator) as i64)
    }

    /// Wraps negative timestamps due to overflow
    ///
    /// Converts negative values that occur due to overflow of one node to the
    /// correct positive value.
    ///
    /// # Example
    /// Maximum timestamp is 1000. Node N1 sends 999 as timestamp. N2 receives
    /// and sends delayed and increased timestamp back. Delay is 10, so timestamp
    /// would be 1009, but due to overflow 009 is sent back.
    /// Now calculate TOF: 009 - 999 = -990 -> incorrect time, so wrap()
    /// Wrap calculation: -990 + 1000 = 10 -> correct time
    #[inline]
    pub fn wrap(&mut self) -> &mut Self {
        if self.timestamp < 0 {
            self.timestamp += TIME_OVERFLOW;
        }
        self
    }

    /// Returns a new wrapped timestamp
    ///
    /// Same as `wrap()` but returns a new instance instead of modifying self
    #[inline]
    pub fn wrapped(mut self) -> Self {
        self.wrap();
        self
    }

    /// Checks if the timestamp is valid for use with the DW1000 device
    ///
    /// Returns `true` if the timestamp is within the valid range [0, TIME_MAX],
    /// `false` if negative or overflow (maybe after calculation)
    #[inline]
    pub const fn is_valid_timestamp(&self) -> bool {
        self.timestamp >= 0 && self.timestamp <= TIME_MAX
    }
}

/// Short alias used across the driver and ranging layers.
pub type DwTime = DW1000Time;

/// A scheduled delayed TX/RX activation, produced by `schedule_delayed` on the
/// drivers.
///
/// `dx_time` is the value programmed into the `DX_TIME` register (low 9 bits
/// zeroed, the hardware ignores them). For a delayed transmission the DW1000
/// emits the ranging marker at `dx_time` and reports
/// `TX_STAMP = dx_time + TX_ANTD`, so `predicted_tx` is the exact transmit
/// timestamp and can be embedded in ranging payloads before the frame is sent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct DelayedTime {
    dx_time: DwTime,
    predicted_tx: DwTime,
}

impl DelayedTime {
    /// Creates a scheduled time from its parts.
    ///
    /// Normally obtained from the drivers' `schedule_delayed`; this
    /// constructor exists for custom `RangingRadio` implementations and tests.
    pub const fn new(dx_time: DwTime, predicted_tx: DwTime) -> Self {
        Self {
            dx_time,
            predicted_tx,
        }
    }

    /// Value to program into the `DX_TIME` register.
    pub const fn dx_time(&self) -> DwTime {
        self.dx_time
    }

    /// Predicted transmit timestamp (`dx_time` + antenna delay).
    pub const fn predicted_tx_timestamp(&self) -> DwTime {
        self.predicted_tx
    }
}

// Implement Add operations
impl Add for DW1000Time {
    type Output = Self;

    #[inline]
    fn add(self, other: Self) -> Self {
        Self {
            timestamp: self.timestamp + other.timestamp,
        }
    }
}

impl AddAssign for DW1000Time {
    #[inline]
    fn add_assign(&mut self, other: Self) {
        self.timestamp += other.timestamp;
    }
}

// Implement Sub operations
impl Sub for DW1000Time {
    type Output = Self;

    #[inline]
    fn sub(self, other: Self) -> Self {
        Self {
            timestamp: self.timestamp - other.timestamp,
        }
    }
}

impl SubAssign for DW1000Time {
    #[inline]
    fn sub_assign(&mut self, other: Self) {
        self.timestamp -= other.timestamp;
    }
}
