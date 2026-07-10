//! Shared DW1000 driver internals used by the blocking and async frontends.

use libm::{floorf, log10f};

use crate::config::{
    ConfigError, DataRate, PacSize, PreambleCode, PreambleLength, PulseFrequency, RadioConfig,
    ValidatedPhyConfig,
};
use crate::device::{AntennaDelay, DeviceIdentity, SysStatus};
use crate::error::RxError;
use crate::registers::status;
use crate::registers::{
    Register, BLNKEN_BIT, DIS_DRXB_BIT, DIS_STXP_BIT, DWSFD_BIT, GPDCE_BIT, GPIO_MODE_GPIO,
    GPIO_MODE_LED, HIRQ_POL_BIT, KHZCLKEN_BIT, LEN_CHAN_CTRL, LEN_PANADR, LEN_RX_FINFO,
    LEN_RX_FQUAL, LEN_RX_TIME, LEN_SYS_CFG, LEN_SYS_MASK, LEN_TX_ANTD, LEN_TX_FCTRL, MAFFREJ_BIT,
    MLDEERR_BIT, MRXDFR_BIT, MRXFCE_BIT, MRXFCG_BIT, MRXFSL_BIT, MRXPHE_BIT, MRXPTO_BIT,
    MRXRFTO_BIT, MRXSFDTO_BIT, MSGP0_BIT, MSGP1_BIT, MSGP2_BIT, MSGP3_BIT, MTXFRS_BIT,
    NO_SUBADDRESS, RNSSFD_BIT, RXAUTR_BIT, RXDLYS_BIT, RXENAB_BIT, RXM110K_BIT, SFCST_BIT,
    TNSSFD_BIT, TRXOFF_BIT, TXDLYS_BIT, TXSTRT_BIT, WAIT4RESP_BIT,
};
use crate::registers::{
    AGC_TUNE1_SUB, AGC_TUNE2_SUB, AGC_TUNE3_SUB, DRX_SFDTOC_SUB, DRX_TUNE0B_SUB, DRX_TUNE1A_SUB,
    DRX_TUNE1B_SUB, DRX_TUNE2_SUB, DRX_TUNE4H_SUB, FS_PLLCFG_SUB, FS_PLLTUNE_SUB, FS_XTALT_SUB,
    LDE_CFG1_SUB, LDE_CFG2_SUB, LDE_REPC_SUB, LDE_RXANTD_SUB, RF_RXCTRLH_SUB, RF_TXCTRL_SUB,
    SFD_LENGTH_SUB, TC_PGDELAY_SUB,
};
use crate::device::SignalMetrics;
use crate::time::{DelayedTime, DwTime, DISTANCE_PER_TICK_M};

pub(crate) const LEN_UWB_FRAMES: usize = 127;

/// LDE_CFG1 value from the official driver: PEAK_MULTPLIER (0x60) | N_STD_FACTOR (13).
pub(crate) const LDE_CFG1_VALUE: u8 = 0x6D;
/// Subaddress of the upper SYS_STATUS bytes used for late delayed-TX/RX checks.
pub(crate) const SYS_STATUS_HI_SUB: u16 = 0x03;
/// HPDWARN | TXPUTE in the 16-bit word at SYS_STATUS offset 3 (`SYS_STATUS_TXERR`).
pub(crate) const DELAYED_TX_LATE_MASK: u16 = 0x0408;
/// HPDWARN in the byte at SYS_STATUS offset 3.
pub(crate) const HPDWARN_HI_BIT: u8 = 0x08;
/// PMSC SOFTRESET value that holds the receiver in reset.
pub(crate) const PMSC_SOFTRESET_RX: u8 = 0xE0;
/// PMSC SOFTRESET value that releases all reset lines.
pub(crate) const PMSC_SOFTRESET_CLEAR: u8 = 0xF0;
/// OTP address of the factory LDO tune value.
pub(crate) const OTP_ADDRESS_LDOTUNE: u16 = 0x004;
/// OTP address of the factory crystal trim value.
pub(crate) const OTP_ADDRESS_XTAL_TRIM: u16 = 0x01E;
/// OTP_SF value that kicks the LDO tune load from OTP.
pub(crate) const OTP_SF_LDO_KICK: u8 = 0x02;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ClockMode {
    Auto,
    Xti,
}

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
    /// Raw crystal-trim OTP value captured during init while the XTI clock is
    /// forced (OTP reads are only reliable under XTI).
    pub(crate) xtal_trim: Option<u8>,
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
            xtal_trim: None,
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
        if status.contains(status::RX_SFD_TIMEOUT) {
            return Err(RxError::SfdTimeout);
        }
        if status.contains(status::RX_OVERRUN) {
            return Err(RxError::Overrun);
        }
        if status.contains(status::FRAME_FILTER_REJECTION) {
            return Err(RxError::FrameFiltered);
        }
        if status.contains(status::RX_TIMEOUT) {
            return Err(RxError::Timeout);
        }
        if status.contains(status::RX_PREAMBLE_TIMEOUT) {
            return Err(RxError::PreambleTimeout);
        }
        if !status.contains(status::RX_FRAME_READY) {
            return Err(RxError::FrameNotReady);
        }
        Ok(())
    }

    pub(crate) fn rx_payload_len(&self, rx_finfo: [u8; LEN_RX_FINFO]) -> usize {
        // Standard PHR mode: 7-bit frame length (long frames are never enabled).
        let mut len = (rx_finfo[0] & 0x7F) as usize;
        if self.frame_check && len > 2 {
            len -= 2;
        }
        len
    }

    /// Computes the delayed activation time for TX/RX.
    ///
    /// The DW1000 ignores the low 9 bits of `DX_TIME`, so they are zeroed to
    /// make the programmed time exact. The antenna delay is *not* added here:
    /// the chip reports `TX_STAMP = DX_TIME + TX_ANTD`, so it only belongs to
    /// the predicted transmit timestamp.
    pub(crate) fn schedule_delayed(&self, now: DwTime, delay: DwTime) -> DelayedTime {
        let mut bytes = (now + delay).to_bytes();
        bytes[0] = 0;
        bytes[1] &= 0xFE;
        let dx_time = DwTime::from_bytes(&bytes);
        let predicted_tx = dx_time + DwTime::from_ticks(self.antenna_delay.raw() as i64);
        DelayedTime::new(dx_time, predicted_tx)
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
        if !self.permanent_receive {
            return false;
        }

        match self.state {
            DriverState::Tx => {
                self.rx_after_tx_pending && event_mask.contains(status::TX_FRAME_SENT)
            }
            // RXAUTR should handle this in hardware, but explicitly rearming after
            // the status is cleared keeps continuous receive reliable on all frames.
            DriverState::Rx => (event_mask.0 & RECEIVE_RESTART_EVENTS) != 0,
            DriverState::Idle => false,
        }
    }

    pub(crate) fn checked_frame_len(&self, payload_len: usize) -> Result<usize, (usize, usize)> {
        let frame_len = if self.frame_check {
            payload_len + 2
        } else {
            payload_len
        };
        if frame_len > LEN_UWB_FRAMES {
            return Err((frame_len, LEN_UWB_FRAMES));
        }
        Ok(frame_len)
    }

    pub(crate) fn begin_receive_session(&mut self, permanent: bool) {
        self.state = DriverState::Rx;
        self.permanent_receive = permanent;
        self.rx_after_tx_pending = false;
    }

    pub(crate) fn begin_transmit_session(&mut self) {
        self.state = DriverState::Tx;
        self.rx_after_tx_pending = self.permanent_receive;
    }

    pub(crate) fn complete_transmit_session(&mut self) {
        if !self.permanent_receive {
            self.state = DriverState::Idle;
        }
    }
}

pub(crate) fn compose_receive_sys_ctrl(sys_ctrl: &mut [u8], frame_check: bool, delayed: bool) {
    set_bit(sys_ctrl, SFCST_BIT, !frame_check);
    set_bit(sys_ctrl, RXENAB_BIT, true);
    if delayed {
        set_bit(sys_ctrl, RXDLYS_BIT, true);
    }
}

pub(crate) fn compose_transmit_sys_ctrl(
    sys_ctrl: &mut [u8],
    frame_check: bool,
    wait_for_response: bool,
    delayed: bool,
) {
    set_bit(sys_ctrl, SFCST_BIT, !frame_check);
    set_bit(sys_ctrl, WAIT4RESP_BIT, wait_for_response);
    if delayed {
        set_bit(sys_ctrl, TXDLYS_BIT, true);
    }
    set_bit(sys_ctrl, TXSTRT_BIT, true);
}

pub(crate) fn prepare_idle_state(runtime: &mut DriverRuntime, sys_ctrl: &mut [u8]) {
    set_bit(sys_ctrl, TRXOFF_BIT, true);
    runtime.state = DriverState::Idle;
}

pub(crate) const fn cleared_interrupt_mask() -> [u8; LEN_SYS_MASK] {
    [0; LEN_SYS_MASK]
}

pub(crate) const fn receive_status_clear_mask() -> SysStatus {
    status::ALL_RX_EVENTS
}

pub(crate) const fn transmit_status_clear_mask() -> SysStatus {
    status::ALL_TX
}

pub(crate) fn set_lde_load_preamble(pmsc_ctrl0: &mut [u8], otp_ctrl: &mut [u8]) {
    pmsc_ctrl0[0] = 0x01;
    pmsc_ctrl0[1] = 0x03;
    otp_ctrl[0] = 0x00;
    otp_ctrl[1] = 0x80;
}

pub(crate) fn set_lde_restore_preamble(pmsc_ctrl0: &mut [u8]) {
    pmsc_ctrl0[0] = 0x00;
    pmsc_ctrl0[1] &= 0x02;
}

pub(crate) fn apply_clock_mode(pmsc_ctrl0: &mut [u8], mode: ClockMode) {
    match mode {
        ClockMode::Auto => {
            pmsc_ctrl0[0] = 0x00;
            pmsc_ctrl0[1] &= 0xFE;
        }
        ClockMode::Xti => {
            pmsc_ctrl0[0] &= 0xFC;
            pmsc_ctrl0[0] |= 0x01;
        }
    }
}

/// Register value for FS_XTALT from the raw OTP trim: the top three bits must
/// be 0b011 and an unprogrammed OTP falls back to the mid-range trim (0x10).
pub(crate) const fn fs_xtalt_value(otp_trim: u8) -> u8 {
    let trim = otp_trim & 0x1F;
    if trim == 0 {
        0x70
    } else {
        0x60 | trim
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

/// Decodes the receive timestamp and signal metrics from bulk register reads
/// of `RX_TIME` (14 bytes), `RX_FQUAL` (8 bytes), and `RX_FINFO`.
pub(crate) fn parse_rx_snapshot(
    phy: ValidatedPhyConfig,
    rx_finfo: &[u8; LEN_RX_FINFO],
    rx_time: &[u8; LEN_RX_TIME],
    rx_fqual: &[u8; LEN_RX_FQUAL],
) -> (DwTime, SignalMetrics) {
    let mut stamp = [0u8; 5];
    stamp.copy_from_slice(&rx_time[0..5]);
    let timestamp = DwTime::from_bytes(&stamp);
    // RX_TIME layout: stamp (0..5), first-path index (5..7), FP_AMPL1 (7..9).
    let fp1 = u16::from_le_bytes([rx_time[7], rx_time[8]]);
    // RX_FQUAL layout: STD_NOISE, FP_AMPL2, FP_AMPL3, CIR_PWR (2 bytes each).
    let noise = u16::from_le_bytes([rx_fqual[0], rx_fqual[1]]);
    let fp2 = u16::from_le_bytes([rx_fqual[2], rx_fqual[3]]);
    let fp3 = u16::from_le_bytes([rx_fqual[4], rx_fqual[5]]);
    let cir_power = u16::from_le_bytes([rx_fqual[6], rx_fqual[7]]);
    let preamble_acc_count = extract_preamble_acc_count(*rx_finfo);
    let metrics = SignalMetrics {
        receive_power_dbm: compute_receive_power(phy, cir_power, preamble_acc_count),
        first_path_power_dbm: compute_first_path_power(phy, fp1, fp2, fp3, preamble_acc_count),
        quality: compute_receive_quality(noise, fp2),
    };
    (timestamp, metrics)
}

pub(crate) fn compose_base_register_fields(
    sys_cfg: &mut [u8],
    sys_mask: &mut [u8],
    interrupt_polarity_high: bool,
    receiver_auto_reenable: bool,
    smart_power: bool,
) {
    set_bit(sys_cfg, DIS_DRXB_BIT, true);
    set_bit(sys_cfg, HIRQ_POL_BIT, interrupt_polarity_high);
    set_bit(sys_cfg, RXAUTR_BIT, receiver_auto_reenable);
    set_bit(sys_cfg, DIS_STXP_BIT, !smart_power);

    set_bit(sys_mask, MTXFRS_BIT, true);
    set_bit(sys_mask, MRXPHE_BIT, true);
    set_bit(sys_mask, MRXDFR_BIT, true);
    set_bit(sys_mask, MRXFCG_BIT, true);
    set_bit(sys_mask, MRXFCE_BIT, true);
    set_bit(sys_mask, MRXFSL_BIT, true);
    set_bit(sys_mask, MLDEERR_BIT, true);
    set_bit(sys_mask, MRXRFTO_BIT, true);
    set_bit(sys_mask, MRXPTO_BIT, true);
    set_bit(sys_mask, MRXSFDTO_BIT, true);
    set_bit(sys_mask, MAFFREJ_BIT, true);
}

pub(crate) fn compose_phy_register_fields(
    sys_cfg: &mut [u8],
    tx_fctrl: &mut [u8],
    chan_ctrl: &mut [u8],
    phy: ValidatedPhyConfig,
) -> u8 {
    let sfd_len = compose_data_rate_fields(sys_cfg, tx_fctrl, chan_ctrl, phy.data_rate);
    compose_pulse_frequency_fields(tx_fctrl, chan_ctrl, phy.pulse_frequency);
    compose_preamble_length_fields(tx_fctrl, phy.preamble_length);
    compose_channel_fields(chan_ctrl, phy.channel as u8);
    compose_preamble_code_fields(chan_ctrl, phy.preamble_code.raw());
    sfd_len
}

/// SFD length in symbols for the SFD scheme selected per data rate.
const fn sfd_length(rate: DataRate) -> u8 {
    match rate {
        DataRate::Mbps6800 => 0x08,
        DataRate::Kbps850 => 0x10,
        DataRate::Kbps110 => 0x40,
    }
}

const fn preamble_symbols(length: PreambleLength) -> u16 {
    match length {
        PreambleLength::Symbols64 => 64,
        PreambleLength::Symbols128 => 128,
        PreambleLength::Symbols256 => 256,
        PreambleLength::Symbols512 => 512,
        PreambleLength::Symbols1024 => 1024,
        PreambleLength::Symbols1536 => 1536,
        PreambleLength::Symbols2048 => 2048,
        PreambleLength::Symbols4096 => 4096,
    }
}

fn compose_data_rate_fields(
    sys_cfg: &mut [u8],
    tx_fctrl: &mut [u8],
    chan_ctrl: &mut [u8],
    rate: DataRate,
) -> u8 {
    tx_fctrl[1] &= 0x83;
    tx_fctrl[1] |= (rate as u8) << 5;
    set_bit(sys_cfg, RXM110K_BIT, rate == DataRate::Kbps110);
    let (dwsfd, tnssfd, rnssfd) = match rate {
        DataRate::Mbps6800 => (false, false, false),
        DataRate::Kbps850 => (true, true, true),
        DataRate::Kbps110 => (true, false, false),
    };
    set_bit(chan_ctrl, DWSFD_BIT, dwsfd);
    set_bit(chan_ctrl, TNSSFD_BIT, tnssfd);
    set_bit(chan_ctrl, RNSSFD_BIT, rnssfd);
    sfd_length(rate)
}

fn compose_pulse_frequency_fields(
    tx_fctrl: &mut [u8],
    chan_ctrl: &mut [u8],
    frequency: PulseFrequency,
) {
    tx_fctrl[2] &= 0xFC;
    tx_fctrl[2] |= frequency as u8;
    chan_ctrl[2] &= 0xF3;
    chan_ctrl[2] |= (frequency as u8) << 2;
}

fn compose_preamble_length_fields(tx_fctrl: &mut [u8], length: crate::config::PreambleLength) {
    tx_fctrl[2] &= 0xC3;
    tx_fctrl[2] |= (length as u8) << 2;
}

fn compose_channel_fields(chan_ctrl: &mut [u8], channel: u8) {
    chan_ctrl[0] = channel | (channel << 4);
}

fn compose_preamble_code_fields(chan_ctrl: &mut [u8], code: u8) {
    chan_ctrl[2] &= 0x3F;
    chan_ctrl[2] |= code << 6;
    chan_ctrl[3] = ((code >> 2) & 0x07) | (code << 3);
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct TuningValues {
    pub(crate) agc_tune1: u16,
    pub(crate) drx_tune0b: u16,
    pub(crate) drx_tune1a: u16,
    pub(crate) drx_tune1b: u16,
    pub(crate) drx_tune2: u32,
    pub(crate) drx_tune4h: u16,
    pub(crate) sfd_timeout: u16,
    pub(crate) rf_rxctrlh: u8,
    pub(crate) rf_txctrl: u32,
    pub(crate) tc_pgdelay: u8,
    pub(crate) fspllcfg: u32,
    pub(crate) fsplltune: u8,
    pub(crate) lde_cfg2: u16,
    pub(crate) lde_repc: u16,
    pub(crate) tx_power: u32,
}

pub(crate) fn select_tuning_values(phy: ValidatedPhyConfig) -> Result<TuningValues, ConfigError> {
    let agc_tune1 = match phy.pulse_frequency {
        PulseFrequency::Mhz16 => 0x8870u16,
        PulseFrequency::Mhz64 => 0x889Bu16,
    };

    let drx_tune0b = match phy.data_rate {
        DataRate::Kbps110 => 0x0016u16,
        DataRate::Kbps850 => 0x0006u16,
        DataRate::Mbps6800 => 0x0001u16,
    };

    let drx_tune1a = match phy.pulse_frequency {
        PulseFrequency::Mhz16 => 0x0087u16,
        PulseFrequency::Mhz64 => 0x008Du16,
    };

    let drx_tune1b: u16 = match (phy.preamble_length, phy.data_rate) {
        (
            PreambleLength::Symbols1024
            | PreambleLength::Symbols1536
            | PreambleLength::Symbols2048
            | PreambleLength::Symbols4096,
            DataRate::Kbps110,
        ) => 0x0064,
        (PreambleLength::Symbols64, DataRate::Mbps6800) => 0x0010,
        (_, DataRate::Kbps850 | DataRate::Mbps6800) => 0x0020,
        _ => return Err(ConfigError::UnsupportedPreambleLength),
    };

    // Note: the official driver ships 0x311A003C for PRF16/PAC8; the user
    // manual (and this table) recommends 0x311A002D. Both are functional.
    let drx_tune2: u32 = match (phy.pac_size, phy.pulse_frequency) {
        (PacSize::Symbols8, PulseFrequency::Mhz16) => 0x311A_002D,
        (PacSize::Symbols8, PulseFrequency::Mhz64) => 0x313B_006B,
        (PacSize::Symbols16, PulseFrequency::Mhz16) => 0x331A_0052,
        (PacSize::Symbols16, PulseFrequency::Mhz64) => 0x333B_00BE,
        (PacSize::Symbols32, PulseFrequency::Mhz16) => 0x351A_009A,
        (PacSize::Symbols32, PulseFrequency::Mhz64) => 0x353B_015E,
        (PacSize::Symbols64, PulseFrequency::Mhz16) => 0x371A_011D,
        (PacSize::Symbols64, PulseFrequency::Mhz64) => 0x373B_0296,
    };

    let drx_tune4h = match phy.preamble_length {
        PreambleLength::Symbols64 => 0x0010u16,
        _ => 0x0028u16,
    };

    // SFD detection timeout in preamble symbols: the receiver gives up when
    // no SFD follows a detected preamble. Must never be written as zero.
    let sfd_timeout = preamble_symbols(phy.preamble_length) + 1 + sfd_length(phy.data_rate) as u16
        - phy.pac_size as u16;

    let rf_rxctrlh = match phy.channel {
        crate::config::Channel::Channel4 | crate::config::Channel::Channel7 => 0xBC,
        _ => 0xD8,
    };

    let rf_txctrl: u32 = match phy.channel {
        crate::config::Channel::Channel1 => 0x0000_5C40,
        crate::config::Channel::Channel2 => 0x0004_5CA0,
        crate::config::Channel::Channel3 => 0x0008_6CC0,
        crate::config::Channel::Channel4 => 0x0004_5C80,
        crate::config::Channel::Channel5 => 0x001E_3FE0,
        crate::config::Channel::Channel7 => 0x001E_7DE0,
    };

    let tc_pgdelay = match phy.channel {
        crate::config::Channel::Channel1 => 0xC9,
        crate::config::Channel::Channel2 => 0xC2,
        crate::config::Channel::Channel3 => 0xC5,
        crate::config::Channel::Channel4 => 0x95,
        crate::config::Channel::Channel5 => 0xC0,
        crate::config::Channel::Channel7 => 0x93,
    };

    let (fspllcfg, fsplltune) = match phy.channel {
        crate::config::Channel::Channel1 => (0x0900_0407u32, 0x1E),
        crate::config::Channel::Channel2 | crate::config::Channel::Channel4 => {
            (0x0840_0508u32, 0x26)
        }
        crate::config::Channel::Channel3 => (0x0840_1009u32, 0x56),
        crate::config::Channel::Channel5 | crate::config::Channel::Channel7 => {
            (0x0800_041Du32, 0xBE)
        }
    };

    let lde_cfg2 = match phy.pulse_frequency {
        PulseFrequency::Mhz16 => 0x1607u16,
        PulseFrequency::Mhz64 => 0x0607u16,
    };

    let lde_repc = lde_repc_value(phy.preamble_code, phy.data_rate);
    let tx_power = tx_power_value(phy.channel, phy.pulse_frequency, phy.smart_power);

    Ok(TuningValues {
        agc_tune1,
        drx_tune0b,
        drx_tune1a,
        drx_tune1b,
        drx_tune2,
        drx_tune4h,
        sfd_timeout,
        rf_rxctrlh,
        rf_txctrl,
        tc_pgdelay,
        fspllcfg,
        fsplltune,
        lde_cfg2,
        lde_repc,
        tx_power,
    })
}

/// Maximum payload of a single [`RegWrite`].
const REG_WRITE_MAX: usize = 8;
/// Number of writes produced by [`config_register_writes`].
pub(crate) const CONFIG_WRITE_COUNT: usize = 28;

/// A single register write, used to share configuration sequences between the
/// blocking and async frontends.
#[derive(Debug, Clone, Copy)]
pub(crate) struct RegWrite {
    pub(crate) register: Register,
    pub(crate) subaddress: u16,
    len: usize,
    bytes: [u8; REG_WRITE_MAX],
}

impl RegWrite {
    fn new(register: Register, subaddress: u16, data: &[u8]) -> Self {
        let mut bytes = [0u8; REG_WRITE_MAX];
        bytes[..data.len()].copy_from_slice(data);
        Self {
            register,
            subaddress,
            len: data.len(),
            bytes,
        }
    }

    pub(crate) fn data(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

/// Produces the full register-write sequence applied by `reconfigure`.
#[allow(clippy::too_many_arguments)]
pub(crate) fn config_register_writes(
    identity: &DeviceIdentity,
    sys_cfg: &[u8; LEN_SYS_CFG],
    sys_mask: &[u8; LEN_SYS_MASK],
    chan_ctrl: &[u8; LEN_CHAN_CTRL],
    tx_fctrl: &[u8; LEN_TX_FCTRL],
    sfd_len: u8,
    tuning: &TuningValues,
    antenna_delay: AntennaDelay,
    fs_xtalt: u8,
) -> [RegWrite; CONFIG_WRITE_COUNT] {
    let mut panadr = [0u8; LEN_PANADR];
    panadr[0..2].copy_from_slice(&identity.short_address.to_le_bytes());
    panadr[2..4].copy_from_slice(&identity.pan_id.to_le_bytes());
    let antenna = antenna_delay.raw().to_le_bytes();
    debug_assert_eq!(antenna.len(), LEN_TX_ANTD);

    [
        RegWrite::new(Register::UsrSfd, SFD_LENGTH_SUB, &[sfd_len]),
        RegWrite::new(Register::PanAdr, NO_SUBADDRESS, &panadr),
        RegWrite::new(Register::Eui, NO_SUBADDRESS, &identity.eui.to_register_bytes()),
        RegWrite::new(Register::SysCfg, NO_SUBADDRESS, sys_cfg),
        RegWrite::new(Register::SysMask, NO_SUBADDRESS, sys_mask),
        RegWrite::new(Register::ChanCtrl, NO_SUBADDRESS, chan_ctrl),
        RegWrite::new(Register::TxFctrl, NO_SUBADDRESS, tx_fctrl),
        RegWrite::new(
            Register::AgcTune,
            AGC_TUNE1_SUB,
            &tuning.agc_tune1.to_le_bytes(),
        ),
        RegWrite::new(
            Register::AgcTune,
            AGC_TUNE2_SUB,
            &0x2502_A907u32.to_le_bytes(),
        ),
        RegWrite::new(Register::AgcTune, AGC_TUNE3_SUB, &0x0035u16.to_le_bytes()),
        RegWrite::new(
            Register::DrxTune,
            DRX_TUNE0B_SUB,
            &tuning.drx_tune0b.to_le_bytes(),
        ),
        RegWrite::new(
            Register::DrxTune,
            DRX_TUNE1A_SUB,
            &tuning.drx_tune1a.to_le_bytes(),
        ),
        RegWrite::new(
            Register::DrxTune,
            DRX_TUNE1B_SUB,
            &tuning.drx_tune1b.to_le_bytes(),
        ),
        RegWrite::new(
            Register::DrxTune,
            DRX_TUNE2_SUB,
            &tuning.drx_tune2.to_le_bytes(),
        ),
        RegWrite::new(
            Register::DrxTune,
            DRX_TUNE4H_SUB,
            &tuning.drx_tune4h.to_le_bytes(),
        ),
        RegWrite::new(
            Register::DrxTune,
            DRX_SFDTOC_SUB,
            &tuning.sfd_timeout.to_le_bytes(),
        ),
        RegWrite::new(Register::RfConf, RF_RXCTRLH_SUB, &[tuning.rf_rxctrlh]),
        RegWrite::new(
            Register::RfConf,
            RF_TXCTRL_SUB,
            &tuning.rf_txctrl.to_le_bytes(),
        ),
        RegWrite::new(Register::TxCal, TC_PGDELAY_SUB, &[tuning.tc_pgdelay]),
        RegWrite::new(
            Register::FsCtrl,
            FS_PLLCFG_SUB,
            &tuning.fspllcfg.to_le_bytes(),
        ),
        RegWrite::new(Register::FsCtrl, FS_PLLTUNE_SUB, &[tuning.fsplltune]),
        RegWrite::new(Register::LdeIf, LDE_CFG1_SUB, &[LDE_CFG1_VALUE]),
        RegWrite::new(
            Register::LdeIf,
            LDE_CFG2_SUB,
            &tuning.lde_cfg2.to_le_bytes(),
        ),
        RegWrite::new(
            Register::LdeIf,
            LDE_REPC_SUB,
            &tuning.lde_repc.to_le_bytes(),
        ),
        RegWrite::new(
            Register::TxPower,
            NO_SUBADDRESS,
            &tuning.tx_power.to_le_bytes(),
        ),
        RegWrite::new(Register::TxAntd, NO_SUBADDRESS, &antenna),
        RegWrite::new(Register::LdeIf, LDE_RXANTD_SUB, &antenna),
        RegWrite::new(Register::FsCtrl, FS_XTALT_SUB, &[fs_xtalt]),
    ]
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

pub(crate) fn compose_gpio_led_mode(gpio_mode: &mut [u8]) {
    // Keep RXOKLED/SFDLED disabled and only route RXLED/TXLED to GPIO2/GPIO3.
    set_gpio_message_mode(gpio_mode, MSGP0_BIT, GPIO_MODE_GPIO);
    set_gpio_message_mode(gpio_mode, MSGP1_BIT, GPIO_MODE_GPIO);
    set_gpio_message_mode(gpio_mode, MSGP2_BIT, GPIO_MODE_LED);
    set_gpio_message_mode(gpio_mode, MSGP3_BIT, GPIO_MODE_LED);
}

pub(crate) fn compose_led_clock_enable(pmsc_ctrl0: &mut [u8]) {
    set_bit(pmsc_ctrl0, GPDCE_BIT, true);
    set_bit(pmsc_ctrl0, KHZCLKEN_BIT, true);
}

pub(crate) fn compose_led_blink_enable(pmsc_ledc: &mut [u8], blink_time: u8) {
    pmsc_ledc[0] = blink_time;
    set_bit(pmsc_ledc, BLNKEN_BIT, true);
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

fn set_gpio_message_mode(gpio_mode: &mut [u8], lsb_bit: u16, mode: u8) {
    set_bit(gpio_mode, lsb_bit, (mode & 0x01) != 0);
    set_bit(gpio_mode, lsb_bit + 1, (mode & 0x02) != 0);
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
    fn schedule_delayed_masks_low_bits_and_predicts_tx_stamp() {
        let mut runtime = DriverRuntime::new();
        let mut config = RadioConfig::from_mode(identity(), OperatingMode::LongDataRangeAccuracy);
        config.antenna_delay = AntennaDelay::new(16_456);
        runtime.reconfigure(&config).unwrap();

        let now = DwTime::from_ticks(0x12_3456_789A);
        let delay = DwTime::from_ticks(0x1_0000);
        let scheduled = runtime.schedule_delayed(now, delay);

        // The DX_TIME value must have the low 9 bits zeroed and must not
        // include the antenna delay.
        assert_eq!(scheduled.dx_time().ticks() & 0x1FF, 0);
        assert_eq!(
            scheduled.dx_time().ticks(),
            (now + delay).ticks() & !0x1FF_i64
        );
        // The prediction is the DX time plus the antenna delay, exactly what
        // the chip will report as TX_STAMP.
        assert_eq!(
            scheduled.predicted_tx_timestamp().ticks(),
            scheduled.dx_time().ticks() + 16_456
        );
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

    #[test]
    fn sfd_timeout_matches_official_formula() {
        let config = RadioConfig::from_mode(identity(), OperatingMode::LongDataRangeAccuracy);
        let phy = config.validated_phy().unwrap();
        let tuning = super::select_tuning_values(phy).unwrap();
        // 2048-symbol preamble, 64-symbol DW SFD, PAC 64: 2048 + 1 + 64 - 64.
        assert_eq!(tuning.sfd_timeout, 2049);
    }

    #[test]
    fn compose_led_configuration_sets_required_bits() {
        let mut gpio_mode = [0xFFu8; 4];
        super::compose_gpio_led_mode(&mut gpio_mode);
        assert_eq!(gpio_mode[0] & 0b1100_0000, 0b0000_0000);
        assert_eq!(gpio_mode[1] & 0b0011_1111, 0b0001_0100);

        let mut pmsc_ctrl0 = [0u8; 4];
        super::compose_led_clock_enable(&mut pmsc_ctrl0);
        assert_eq!(pmsc_ctrl0[2] & 0b1000_0100, 0b1000_0100);

        let mut pmsc_ledc = [0u8; 4];
        super::compose_led_blink_enable(&mut pmsc_ledc, 0x20);
        assert_eq!(pmsc_ledc[0], 0x20);
        assert_eq!(pmsc_ledc[1] & 0x01, 0x01);
    }
}
