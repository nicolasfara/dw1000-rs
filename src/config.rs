//! DW1000 configuration types
//!
//! This module provides type-safe configuration structures and enums for
//! configuring the DW1000 UWB transceiver.

#[cfg(feature = "defmt")]
use defmt::Format;

use crate::device::{AntennaDelay, DeviceIdentity};
use crate::time::DelayedTime;

/// Data transmission/reception bit rate
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum DataRate {
    /// 110 kbps data rate
    Kbps110 = 0x00,
    /// 850 kbps data rate
    Kbps850 = 0x01,
    /// 6.8 Mbps data rate
    Mbps6800 = 0x02,
}

/// Transmission pulse repetition frequency (PRF)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum PulseFrequency {
    /// 16 MHz PRF (more power efficient)
    Mhz16 = 0x01,
    /// 64 MHz PRF (better performance, more power)
    Mhz64 = 0x02,
}

/// Preamble length configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum PreambleLength {
    /// 64 symbols
    Symbols64 = 0x01,
    /// 128 symbols
    Symbols128 = 0x05,
    /// 256 symbols
    Symbols256 = 0x09,
    /// 512 symbols
    Symbols512 = 0x0D,
    /// 1024 symbols
    Symbols1024 = 0x02,
    /// 1536 symbols
    Symbols1536 = 0x06,
    /// 2048 symbols
    Symbols2048 = 0x0A,
    /// 4096 symbols
    Symbols4096 = 0x03,
}

/// Preamble Acquisition Chunk (PAC) size
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum PacSize {
    /// 8 symbols
    Symbols8 = 8,
    /// 16 symbols
    Symbols16 = 16,
    /// 32 symbols
    Symbols32 = 32,
    /// 64 symbols
    Symbols64 = 64,
}

/// RF channel selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum Channel {
    /// Channel 1 (3.5 GHz)
    Channel1 = 1,
    /// Channel 2 (4.0 GHz)
    Channel2 = 2,
    /// Channel 3 (4.5 GHz)
    Channel3 = 3,
    /// Channel 4 (4.0 GHz)
    Channel4 = 4,
    /// Channel 5 (6.5 GHz)
    Channel5 = 5,
    /// Channel 7 (6.5 GHz)
    Channel7 = 7,
}

/// Preamble codes for different PRF settings
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum PreambleCode {
    /// Preamble code 1 (16 MHz PRF)
    Code1 = 1,
    /// Preamble code 2 (16 MHz PRF)
    Code2 = 2,
    /// Preamble code 3 (16 MHz PRF)
    Code3 = 3,
    /// Preamble code 4 (16 MHz PRF)
    Code4 = 4,
    /// Preamble code 5 (16 MHz PRF)
    Code5 = 5,
    /// Preamble code 6 (16 MHz PRF)
    Code6 = 6,
    /// Preamble code 7 (16 MHz PRF)
    Code7 = 7,
    /// Preamble code 8 (16 MHz PRF)
    Code8 = 8,
    /// Preamble code 9 (64 MHz PRF)
    Code9 = 9,
    /// Preamble code 10 (64 MHz PRF)
    Code10 = 10,
    /// Preamble code 11 (64 MHz PRF)
    Code11 = 11,
    /// Preamble code 12 (64 MHz PRF)
    Code12 = 12,
    /// Preamble code 17 (64 MHz PRF)
    Code17 = 17,
    /// Preamble code 18 (64 MHz PRF)
    Code18 = 18,
    /// Preamble code 19 (64 MHz PRF)
    Code19 = 19,
    /// Preamble code 20 (64 MHz PRF)
    Code20 = 20,
}

impl PreambleCode {
    /// Returns the raw preamble-code value expected by the DW1000.
    pub const fn raw(self) -> u8 {
        self as u8
    }
}

/// Pre-defined operation modes combining data rate, PRF, and preamble length
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum OperatingMode {
    /// Long data range, low power (110 kbps, 16 MHz PRF, 2048 preamble)
    LongDataRangeLowPower,
    /// Short data fast, low power (6.8 Mbps, 16 MHz PRF, 128 preamble)
    ShortDataFastLowPower,
    /// Long data fast, low power (6.8 Mbps, 16 MHz PRF, 1024 preamble)
    LongDataFastLowPower,
    /// Short data fast, accuracy (6.8 Mbps, 64 MHz PRF, 128 preamble)
    ShortDataFastAccuracy,
    /// Long data fast, accuracy (6.8 Mbps, 64 MHz PRF, 1024 preamble)
    LongDataFastAccuracy,
    /// Long data range, accuracy (110 kbps, 64 MHz PRF, 2048 preamble)
    LongDataRangeAccuracy,
}

impl OperatingMode {
    /// Returns the data rate, pulse frequency, and preamble length for this mode
    pub const fn config(&self) -> (DataRate, PulseFrequency, PreambleLength) {
        match self {
            OperatingMode::LongDataRangeLowPower => (
                DataRate::Kbps110,
                PulseFrequency::Mhz16,
                PreambleLength::Symbols2048,
            ),
            OperatingMode::ShortDataFastLowPower => (
                DataRate::Mbps6800,
                PulseFrequency::Mhz16,
                PreambleLength::Symbols128,
            ),
            OperatingMode::LongDataFastLowPower => (
                DataRate::Mbps6800,
                PulseFrequency::Mhz16,
                PreambleLength::Symbols1024,
            ),
            OperatingMode::ShortDataFastAccuracy => (
                DataRate::Mbps6800,
                PulseFrequency::Mhz64,
                PreambleLength::Symbols128,
            ),
            OperatingMode::LongDataFastAccuracy => (
                DataRate::Mbps6800,
                PulseFrequency::Mhz64,
                PreambleLength::Symbols1024,
            ),
            OperatingMode::LongDataRangeAccuracy => (
                DataRate::Kbps110,
                PulseFrequency::Mhz64,
                PreambleLength::Symbols2048,
            ),
        }
    }
}

/// Driver configuration validation errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum ConfigError {
    /// The requested preamble length cannot be used with the selected PHY setup.
    UnsupportedPreambleLength,
    /// The selected preamble code cannot be used with the selected pulse frequency.
    InvalidPreambleCode,
}

/// Addressing configuration written into the DW1000.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct AddressConfig {
    /// Local device identity.
    pub identity: DeviceIdentity,
}

/// User-provided PHY configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct PhyConfig {
    /// Data rate.
    pub data_rate: DataRate,
    /// Pulse repetition frequency.
    pub pulse_frequency: PulseFrequency,
    /// Preamble length.
    pub preamble_length: PreambleLength,
    /// RF channel.
    pub channel: Channel,
    /// Optional explicit preamble code.
    pub preamble_code: Option<PreambleCode>,
    /// Smart power control.
    pub smart_power: bool,
}

/// Driver-ready PHY configuration with all derived fields resolved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct ValidatedPhyConfig {
    /// Data rate.
    pub data_rate: DataRate,
    /// Pulse repetition frequency.
    pub pulse_frequency: PulseFrequency,
    /// Preamble length.
    pub preamble_length: PreambleLength,
    /// RF channel.
    pub channel: Channel,
    /// Resolved preamble code.
    pub preamble_code: PreambleCode,
    /// Resolved PAC size.
    pub pac_size: PacSize,
    /// Smart power control.
    pub smart_power: bool,
}

impl PhyConfig {
    fn validated(self) -> Result<ValidatedPhyConfig, ConfigError> {
        let preamble_code = self
            .preamble_code
            .unwrap_or(default_preamble_code(self.pulse_frequency));
        if !preamble_code_matches_pulse_frequency(preamble_code, self.pulse_frequency) {
            return Err(ConfigError::InvalidPreambleCode);
        }

        Ok(ValidatedPhyConfig {
            data_rate: self.data_rate,
            pulse_frequency: self.pulse_frequency,
            preamble_length: self.preamble_length,
            channel: self.channel,
            preamble_code,
            pac_size: pac_size_for_preamble(self.preamble_length),
            smart_power: self.smart_power,
        })
    }
}

/// Complete radio configuration consumed by the driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct RadioConfig {
    /// Local address configuration.
    pub address: AddressConfig,
    /// PHY configuration.
    pub phy: PhyConfig,
    /// Symmetric antenna delay.
    pub antenna_delay: AntennaDelay,
    /// Re-enable RX automatically after receive completion.
    pub receiver_auto_reenable: bool,
    /// Interrupt polarity (`true` = active high).
    pub interrupt_polarity_high: bool,
    /// Include/check the IEEE 802.15.4 frame check sequence.
    pub frame_check: bool,
}

impl RadioConfig {
    /// Builds a radio configuration from a logical identity and operating mode.
    pub fn from_mode(identity: DeviceIdentity, mode: OperatingMode) -> Self {
        let (data_rate, pulse_frequency, preamble_length) = mode.config();
        Self {
            address: AddressConfig { identity },
            phy: PhyConfig {
                data_rate,
                pulse_frequency,
                preamble_length,
                channel: Channel::Channel5,
                preamble_code: None,
                smart_power: false,
            },
            antenna_delay: AntennaDelay::LEGACY_DEFAULT,
            receiver_auto_reenable: true,
            interrupt_polarity_high: true,
            frame_check: true,
        }
    }

    /// Validates and resolves derived PHY fields.
    pub fn validated_phy(&self) -> Result<ValidatedPhyConfig, ConfigError> {
        self.phy.validated()
    }
}

/// Receive options for a single DW1000 receive session.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct RxOptions {
    /// Optional absolute delayed start time, obtained from `schedule_delayed`.
    pub delayed_time: Option<DelayedTime>,
    /// Keep the receiver permanently armed.
    pub permanent: bool,
}

/// Transmit options for a single DW1000 frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct TxOptions {
    /// Optional absolute delayed transmit time, obtained from `schedule_delayed`.
    pub delayed_time: Option<DelayedTime>,
    /// Set the WAIT4RESP bit after transmit.
    pub wait_for_response: bool,
}

const fn default_preamble_code(pulse_frequency: PulseFrequency) -> PreambleCode {
    match pulse_frequency {
        PulseFrequency::Mhz16 => PreambleCode::Code4,
        PulseFrequency::Mhz64 => PreambleCode::Code10,
    }
}

const fn preamble_code_matches_pulse_frequency(
    preamble_code: PreambleCode,
    pulse_frequency: PulseFrequency,
) -> bool {
    matches!(
        (pulse_frequency, preamble_code),
        (
            PulseFrequency::Mhz16,
            PreambleCode::Code1
                | PreambleCode::Code2
                | PreambleCode::Code3
                | PreambleCode::Code4
                | PreambleCode::Code5
                | PreambleCode::Code6
                | PreambleCode::Code7
                | PreambleCode::Code8
        ) | (
            PulseFrequency::Mhz64,
            PreambleCode::Code9
                | PreambleCode::Code10
                | PreambleCode::Code11
                | PreambleCode::Code12
                | PreambleCode::Code17
                | PreambleCode::Code18
                | PreambleCode::Code19
                | PreambleCode::Code20
        )
    )
}

const fn pac_size_for_preamble(preamble_length: PreambleLength) -> PacSize {
    match preamble_length {
        PreambleLength::Symbols64 | PreambleLength::Symbols128 => PacSize::Symbols8,
        PreambleLength::Symbols256 | PreambleLength::Symbols512 => PacSize::Symbols16,
        PreambleLength::Symbols1024 => PacSize::Symbols32,
        PreambleLength::Symbols1536 | PreambleLength::Symbols2048 | PreambleLength::Symbols4096 => {
            PacSize::Symbols64
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        Channel, ConfigError, DataRate, PhyConfig, PreambleCode, PreambleLength, PulseFrequency,
    };

    fn phy_config(
        pulse_frequency: PulseFrequency,
        preamble_code: Option<PreambleCode>,
    ) -> PhyConfig {
        PhyConfig {
            data_rate: DataRate::Kbps110,
            pulse_frequency,
            preamble_length: PreambleLength::Symbols2048,
            channel: Channel::Channel5,
            preamble_code,
            smart_power: false,
        }
    }

    #[test]
    fn explicit_preamble_code_must_match_pulse_frequency() {
        assert_eq!(
            phy_config(PulseFrequency::Mhz16, Some(PreambleCode::Code4))
                .validated()
                .unwrap()
                .preamble_code,
            PreambleCode::Code4
        );
        assert_eq!(
            phy_config(PulseFrequency::Mhz64, Some(PreambleCode::Code10))
                .validated()
                .unwrap()
                .preamble_code,
            PreambleCode::Code10
        );
        assert_eq!(
            phy_config(PulseFrequency::Mhz16, Some(PreambleCode::Code10)).validated(),
            Err(ConfigError::InvalidPreambleCode)
        );
        assert_eq!(
            phy_config(PulseFrequency::Mhz64, Some(PreambleCode::Code4)).validated(),
            Err(ConfigError::InvalidPreambleCode)
        );
    }

    #[test]
    fn default_preamble_codes_still_resolve_for_each_pulse_frequency() {
        assert_eq!(
            phy_config(PulseFrequency::Mhz16, None)
                .validated()
                .unwrap()
                .preamble_code,
            PreambleCode::Code4
        );
        assert_eq!(
            phy_config(PulseFrequency::Mhz64, None)
                .validated()
                .unwrap()
                .preamble_code,
            PreambleCode::Code10
        );
    }
}
