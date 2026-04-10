//! Shared DW1000 driver internals used by the blocking and async frontends.

use libm::{floorf, log10f};

use crate::config::{
    ConfigError, DataRate, PreambleCode, PulseFrequency, RadioConfig, ValidatedPhyConfig,
};
use crate::device::{AntennaDelay, DeviceIdentity, SysStatus};
use crate::error::RxError;
use crate::registers::status;
use crate::registers::{Register, LEN_RX_FINFO, NO_SUBADDRESS};
use crate::time::{DwTime, DISTANCE_PER_TICK_M};

pub(crate) const LEN_UWB_FRAMES: usize = 127;

const WRITE: u8 = 0x80;
const WRITE_SUB: u8 = 0xC0;
const READ: u8 = 0x00;
const READ_SUB: u8 = 0x40;
const RW_SUB_EXT: u8 = 0x80;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DriverState {
    Idle,
    Rx,
    Tx,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct DriverRuntime {
    pub(crate) frame_check: bool,
    pub(crate) permanent_receive: bool,
    pub(crate) state: DriverState,
    pub(crate) antenna_delay: AntennaDelay,
    pub(crate) phy: Option<ValidatedPhyConfig>,
    pub(crate) identity: Option<DeviceIdentity>,
    pub(crate) rx_after_tx_pending: bool,
}

impl DriverRuntime {
    pub(crate) const fn new() -> Self {
        Self {
            frame_check: true,
            permanent_receive: false,
            state: DriverState::Idle,
            antenna_delay: AntennaDelay::LEGACY_DEFAULT,
            phy: None,
            identity: None,
            rx_after_tx_pending: false,
        }
    }

    pub(crate) fn reconfigure(
        &mut self,
        config: &RadioConfig,
    ) -> Result<ValidatedPhyConfig, ConfigError> {
        let phy = config.validated_phy()?;
        self.identity = Some(config.address.identity);
        self.phy = Some(phy);
        self.antenna_delay = config.antenna_delay;
        self.frame_check = config.frame_check;
        self.permanent_receive = false;
        self.state = DriverState::Idle;
        self.rx_after_tx_pending = false;
        Ok(phy)
    }

    pub(crate) const fn identity(&self) -> Option<DeviceIdentity> {
        self.identity
    }

    pub(crate) fn validate_rx_status(&self, status: SysStatus) -> Result<(), RxError> {
        if status.contains(status::LDE_ERROR) {
            return Err(RxError::LeadingEdgeDetection);
        }
        if status.contains(status::RX_FRAME_CHECK_ERROR) {
            return Err(RxError::FrameCheck);
        }
        if status.contains(status::RX_HEADER_ERROR) {
            return Err(RxError::Header);
        }
        if status.contains(status::RX_REED_SOLOMON_ERROR) {
            return Err(RxError::ReedSolomon);
        }
        if status.contains(status::RX_TIMEOUT) {
            return Err(RxError::Timeout);
        }
        Ok(())
    }

    pub(crate) fn rx_payload_len(&self, rx_finfo: [u8; LEN_RX_FINFO]) -> usize {
        let mut len = (((rx_finfo[1] as usize) << 8) | rx_finfo[0] as usize) & 0x03FF;
        if self.frame_check && len > 2 {
            len -= 2;
        }
        len
    }

    pub(crate) fn compute_delayed_time(&self, system_time: DwTime) -> DwTime {
        let mut future = system_time;
        let mut bytes = future.to_bytes();
        bytes[0] = 0;
        bytes[1] &= 0xFE;
        future = DwTime::from_bytes(&bytes);
        future + DwTime::from_ticks(self.antenna_delay.raw() as i64)
    }

    pub(crate) fn correct_receive_timestamp(
        &self,
        timestamp: DwTime,
        receive_power_dbm: f32,
    ) -> DwTime {
        let Some(phy) = self.phy else {
            return timestamp;
        };
        if !receive_power_dbm.is_finite() {
            return timestamp;
        }
        let rx_power_base = -(receive_power_dbm + 61.0) * 0.5;
        if !rx_power_base.is_finite() {
            return timestamp;
        }
        let mut low = floorf(rx_power_base) as isize;
        let mut high = low + 1;
        if low <= 0 {
            low = 0;
            high = 0;
        } else if high >= 17 {
            low = 17;
            high = 17;
        }
        let (table, zero, scale) = match (phy.channel, phy.pulse_frequency) {
            (
                crate::config::Channel::Channel4 | crate::config::Channel::Channel7,
                PulseFrequency::Mhz16,
            ) => (&BIAS_900_16, 7usize, 2.0),
            (
                crate::config::Channel::Channel4 | crate::config::Channel::Channel7,
                PulseFrequency::Mhz64,
            ) => (&BIAS_900_64, 7usize, 2.0),
            (_, PulseFrequency::Mhz16) => (&BIAS_500_16, 10usize, 1.0),
            (_, PulseFrequency::Mhz64) => (&BIAS_500_64, 8usize, 1.0),
        };
        let low_bias = signed_bias(table[low as usize], low as usize, zero) * scale;
        let high_bias = signed_bias(table[high as usize], high as usize, zero) * scale;
        let bias_mm = low_bias + (rx_power_base - low as f32) * (high_bias - low_bias);
        let adjustment = DwTime::from_ticks((bias_mm * 0.001 / DISTANCE_PER_TICK_M) as i64);
        timestamp - adjustment
    }

    pub(crate) const fn should_restart_receive(&self, event_mask: SysStatus) -> bool {
        self.rx_after_tx_pending
            && matches!(self.state, DriverState::Tx)
            && event_mask.contains(status::TX_FRAME_SENT)
    }
}

pub(crate) fn compute_receive_quality(noise: u16, fp2: u16) -> f32 {
    fp2 as f32 / noise.max(1) as f32
}

pub(crate) fn compute_first_path_power(
    phy: ValidatedPhyConfig,
    fp1: u16,
    fp2: u16,
    fp3: u16,
    preamble_acc_count: u16,
) -> f32 {
    let f1 = fp1 as f32;
    let f2 = fp2 as f32;
    let f3 = fp3 as f32;
    let n = preamble_acc_count.max(1) as f32;
    let (a, corr) = match phy.pulse_frequency {
        PulseFrequency::Mhz16 => (113.77, 2.3334),
        PulseFrequency::Mhz64 => (121.74, 1.1667),
    };
    let mut power = 10.0 * log10f((f1 * f1 + f2 * f2 + f3 * f3) / (n * n)) - a;
    if power > -88.0 {
        power += (power + 88.0) * corr;
    }
    power
}

pub(crate) fn compute_receive_power(
    phy: ValidatedPhyConfig,
    cir_power: u16,
    preamble_acc_count: u16,
) -> f32 {
    let c = cir_power as f32;
    let n = preamble_acc_count.max(1) as f32;
    let (a, corr) = match phy.pulse_frequency {
        PulseFrequency::Mhz16 => (113.77, 2.3334),
        PulseFrequency::Mhz64 => (121.74, 1.1667),
    };
    let mut power = 10.0 * log10f((c * 131_072.0) / (n * n)) - a;
    if power > -88.0 {
        power += (power + 88.0) * corr;
    }
    power
}

pub(crate) fn extract_preamble_acc_count(rx_finfo: [u8; LEN_RX_FINFO]) -> u16 {
    (((rx_finfo[2] as u16) >> 4) & 0xFF) | ((rx_finfo[3] as u16) << 4)
}

pub(crate) fn build_header(register: Register, subaddress: u16, write: bool) -> [u8; 3] {
    let mut header = [0u8; 3];
    if subaddress == NO_SUBADDRESS {
        header[0] = (if write { WRITE } else { READ }) | register as u8;
        return header;
    }
    header[0] = (if write { WRITE_SUB } else { READ_SUB }) | register as u8;
    if subaddress < 128 {
        header[1] = subaddress as u8;
    } else {
        header[1] = RW_SUB_EXT | (subaddress as u8);
        header[2] = (subaddress >> 7) as u8;
    }
    header
}

pub(crate) const fn header_len(subaddress: u16) -> usize {
    if subaddress == NO_SUBADDRESS {
        1
    } else if subaddress < 128 {
        2
    } else {
        3
    }
}

pub(crate) fn set_bit(bytes: &mut [u8], bit: u16, value: bool) {
    let index = (bit / 8) as usize;
    let shift = (bit % 8) as u8;
    debug_assert!(
        index < bytes.len(),
        "bit index {} out of range for {}-byte register",
        bit,
        bytes.len()
    );
    let Some(byte) = bytes.get_mut(index) else {
        return;
    };
    if value {
        *byte |= 1 << shift;
    } else {
        *byte &= !(1 << shift);
    }
}

pub(crate) fn lde_repc_value(code: PreambleCode, rate: DataRate) -> u16 {
    let base = match code {
        PreambleCode::Code1 | PreambleCode::Code2 => 0x5998,
        PreambleCode::Code3 | PreambleCode::Code8 => 0x51EA,
        PreambleCode::Code4 => 0x428E,
        PreambleCode::Code5 => 0x451E,
        PreambleCode::Code6 => 0x2E14,
        PreambleCode::Code7 => 0x8000,
        PreambleCode::Code9 => 0x28F4,
        PreambleCode::Code10 | PreambleCode::Code17 => 0x3332,
        PreambleCode::Code11 => 0x3AE0,
        PreambleCode::Code12 => 0x3D70,
        PreambleCode::Code18 | PreambleCode::Code19 => 0x35C2,
        PreambleCode::Code20 => 0x47AE,
    };
    if rate == DataRate::Kbps110 {
        base >> 3
    } else {
        base
    }
}

pub(crate) fn tx_power_value(
    channel: crate::config::Channel,
    prf: PulseFrequency,
    smart_power: bool,
) -> u32 {
    match (channel, prf, smart_power) {
        (
            crate::config::Channel::Channel1 | crate::config::Channel::Channel2,
            PulseFrequency::Mhz16,
            true,
        ) => 0x1535_5575,
        (
            crate::config::Channel::Channel1 | crate::config::Channel::Channel2,
            PulseFrequency::Mhz16,
            false,
        ) => 0x7575_7575,
        (
            crate::config::Channel::Channel1 | crate::config::Channel::Channel2,
            PulseFrequency::Mhz64,
            true,
        ) => 0x0727_4767,
        (
            crate::config::Channel::Channel1 | crate::config::Channel::Channel2,
            PulseFrequency::Mhz64,
            false,
        ) => 0x6767_6767,
        (crate::config::Channel::Channel3, PulseFrequency::Mhz16, true) => 0x0F2F_4F6F,
        (crate::config::Channel::Channel3, PulseFrequency::Mhz16, false) => 0x6F6F_6F6F,
        (crate::config::Channel::Channel3, PulseFrequency::Mhz64, true) => 0x2B4B_6B8B,
        (crate::config::Channel::Channel3, PulseFrequency::Mhz64, false) => 0x8B8B_8B8B,
        (crate::config::Channel::Channel4, PulseFrequency::Mhz16, true) => 0x1F1F_3F5F,
        (crate::config::Channel::Channel4, PulseFrequency::Mhz16, false) => 0x5F5F_5F5F,
        (crate::config::Channel::Channel4, PulseFrequency::Mhz64, true) => 0x3A5A_7A9A,
        (crate::config::Channel::Channel4, PulseFrequency::Mhz64, false) => 0x9A9A_9A9A,
        (crate::config::Channel::Channel5, PulseFrequency::Mhz16, true) => 0x0E08_2848,
        (crate::config::Channel::Channel5, PulseFrequency::Mhz16, false) => 0x4848_4848,
        (crate::config::Channel::Channel5, PulseFrequency::Mhz64, true) => 0x2545_6585,
        (crate::config::Channel::Channel5, PulseFrequency::Mhz64, false) => 0x8585_8585,
        (crate::config::Channel::Channel7, PulseFrequency::Mhz16, true) => 0x3252_7292,
        (crate::config::Channel::Channel7, PulseFrequency::Mhz16, false) => 0x9292_9292,
        (crate::config::Channel::Channel7, PulseFrequency::Mhz64, true) => 0x5171_B1D1,
        (crate::config::Channel::Channel7, PulseFrequency::Mhz64, false) => 0xD1D1_D1D1,
    }
}

const BIAS_500_16: [u8; 18] = [
    198, 187, 179, 163, 143, 127, 109, 84, 59, 31, 0, 36, 65, 84, 97, 106, 110, 112,
];
const BIAS_500_64: [u8; 18] = [
    110, 105, 100, 93, 82, 69, 51, 27, 0, 21, 35, 42, 49, 62, 71, 76, 81, 86,
];
const BIAS_900_16: [u8; 18] = [
    137, 122, 105, 88, 69, 47, 25, 0, 21, 48, 79, 105, 127, 147, 160, 169, 178, 197,
];
const BIAS_900_64: [u8; 18] = [
    147, 133, 117, 99, 75, 50, 29, 0, 24, 45, 63, 76, 87, 98, 116, 122, 132, 142,
];

fn signed_bias(value: u8, index: usize, zero: usize) -> f32 {
    if index < zero {
        -(value as f32)
    } else {
        value as f32
    }
}

#[cfg(test)]
mod tests {
    use super::DriverRuntime;
    use crate::config::{OperatingMode, RadioConfig};
    use crate::device::{AntennaDelay, DeviceIdentity, Eui64, PanId, ShortAddress};
    use crate::time::DwTime;

    fn identity() -> DeviceIdentity {
        DeviceIdentity::new(
            PanId::new(0x0D57),
            ShortAddress::new(3344),
            Eui64::new([0x82, 0x17, 0x5B, 0xD5, 0xA9, 0x9A, 0xE2, 0x9C]),
        )
    }

    #[test]
    fn non_finite_receive_power_skips_timestamp_bias_correction() {
        let mut runtime = DriverRuntime::new();
        let mut config = RadioConfig::from_mode(identity(), OperatingMode::LongDataRangeAccuracy);
        config.antenna_delay = AntennaDelay::new(16_456);
        runtime.reconfigure(&config).unwrap();

        let timestamp = DwTime::from_ticks(123_456);
        assert_eq!(
            runtime.correct_receive_timestamp(timestamp, f32::NEG_INFINITY),
            timestamp
        );
        assert_eq!(
            runtime.correct_receive_timestamp(timestamp, f32::INFINITY),
            timestamp
        );
        assert_eq!(
            runtime.correct_receive_timestamp(timestamp, f32::NAN),
            timestamp
        );
    }
}
