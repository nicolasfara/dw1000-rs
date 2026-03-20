//! Core DW1000 driver implementation.

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::{Operation, SpiDevice};
use libm::log10f;

use crate::config::{
    ConfigError, DataRate, PacSize, PreambleCode, PreambleLength, PulseFrequency, RadioConfig,
    RxOptions, TxOptions, ValidatedPhyConfig,
};
use crate::device::{AntennaDelay, DeviceIdentity, RxFrame, SignalMetrics, SysStatus, Timestamps};
use crate::error::{Error, RxError};
use crate::registers::status;
use crate::registers::{
    sys_status_from_bytes, sys_status_to_bytes, Register, AGC_TUNE1_SUB, AGC_TUNE2_SUB,
    AGC_TUNE3_SUB, CIR_PWR_SUB, DIS_DRXB_BIT, DIS_STXP_BIT, DRX_TUNE0B_SUB, DRX_TUNE1A_SUB,
    DRX_TUNE1B_SUB, DRX_TUNE2_SUB, DRX_TUNE4H_SUB, DWSFD_BIT, FP_AMPL1_SUB, FP_AMPL2_SUB,
    FP_AMPL3_SUB, FS_PLLCFG_SUB, FS_PLLTUNE_SUB, FS_XTALT_SUB, HIRQ_POL_BIT, LDE_CFG1_SUB,
    LDE_CFG2_SUB, LDE_REPC_SUB, LDE_RXANTD_SUB, LEN_CHAN_CTRL, LEN_CIR_PWR, LEN_FP_AMPL1,
    LEN_FP_AMPL2, LEN_FP_AMPL3, LEN_LDE_RXANTD, LEN_OTP_ADDR, LEN_OTP_CTRL, LEN_OTP_RDAT,
    LEN_PANADR, LEN_PMSC_CTRL0, LEN_RX_FINFO, LEN_RX_STAMP, LEN_STD_NOISE, LEN_SYS_CFG,
    LEN_SYS_CTRL, LEN_SYS_MASK, LEN_SYS_STATUS, LEN_TX_ANTD, LEN_TX_FCTRL, LEN_TX_STAMP,
    NO_SUBADDRESS, OTP_ADDR_SUB, OTP_CTRL_SUB, OTP_RDAT_SUB, PMSC_CTRL0_SUB, RF_RXCTRLH_SUB,
    RF_TXCTRL_SUB, RNSSFD_BIT, RXAUTR_BIT, RXDLYS_BIT, RXENAB_BIT, RXM110K_BIT, RX_STAMP_SUB,
    SFCST_BIT, SFD_LENGTH_SUB, STD_NOISE_SUB, TC_PGDELAY_SUB, TNSSFD_BIT, TRXOFF_BIT, TXDLYS_BIT,
    TXSTRT_BIT, TX_STAMP_SUB, WAIT4RESP_BIT,
};
use crate::time::DwTime;

const LEN_UWB_FRAMES: usize = 127;
const LEN_EXT_UWB_FRAMES: usize = 1023;

const WRITE: u8 = 0x80;
const WRITE_SUB: u8 = 0xC0;
const READ: u8 = 0x00;
const READ_SUB: u8 = 0x40;
const RW_SUB_EXT: u8 = 0x80;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DriverState {
    Idle,
    Rx,
    Tx,
}

/// Blocking DW1000 driver.
#[derive(Debug)]
pub struct Dw1000<SPI, IRQ, RST> {
    spi: SPI,
    irq: IRQ,
    reset: RST,
    sys_cfg: [u8; LEN_SYS_CFG],
    sys_ctrl: [u8; LEN_SYS_CTRL],
    sys_mask: [u8; LEN_SYS_MASK],
    tx_fctrl: [u8; LEN_TX_FCTRL],
    chan_ctrl: [u8; LEN_CHAN_CTRL],
    panadr: [u8; LEN_PANADR],
    frame_check: bool,
    permanent_receive: bool,
    state: DriverState,
    antenna_delay: AntennaDelay,
    phy: Option<ValidatedPhyConfig>,
    identity: Option<DeviceIdentity>,
    rx_after_tx_pending: bool,
}

impl<SPI, IRQ, RST, PinE> Dw1000<SPI, IRQ, RST>
where
    SPI: SpiDevice,
    IRQ: InputPin<Error = PinE>,
    RST: OutputPin<Error = PinE>,
{
    /// Creates a new driver instance.
    pub fn new(spi: SPI, irq: IRQ, reset: RST) -> Self {
        Self {
            spi,
            irq,
            reset,
            sys_cfg: [0; LEN_SYS_CFG],
            sys_ctrl: [0; LEN_SYS_CTRL],
            sys_mask: [0; LEN_SYS_MASK],
            tx_fctrl: [0; LEN_TX_FCTRL],
            chan_ctrl: [0; LEN_CHAN_CTRL],
            panadr: [0xFF; LEN_PANADR],
            frame_check: true,
            permanent_receive: false,
            state: DriverState::Idle,
            antenna_delay: AntennaDelay::LEGACY_DEFAULT,
            phy: None,
            identity: None,
            rx_after_tx_pending: false,
        }
    }

    /// Initializes the DW1000 and applies the supplied configuration.
    pub fn init(
        &mut self,
        delay: &mut impl DelayNs,
        config: &RadioConfig,
    ) -> Result<(), Error<SPI::Error, PinE>> {
        self.hard_reset(delay)?;
        self.enable_clock(ClockMode::Auto)?;
        delay.delay_ms(5);
        self.clear_interrupts()?;
        self.enable_clock(ClockMode::Xti)?;
        delay.delay_ms(5);
        self.manage_lde(delay)?;
        self.enable_clock(ClockMode::Auto)?;
        delay.delay_ms(5);
        self.reconfigure(config)
    }

    /// Applies a new radio configuration without performing a reset.
    pub fn reconfigure(&mut self, config: &RadioConfig) -> Result<(), Error<SPI::Error, PinE>> {
        let phy = config.validated_phy()?;
        self.identity = Some(config.address.identity);
        self.phy = Some(phy);
        self.antenna_delay = config.antenna_delay;
        self.frame_check = config.frame_check;
        self.permanent_receive = false;
        self.rx_after_tx_pending = false;

        self.idle()?;
        self.panadr = [0xFF; LEN_PANADR];
        self.panadr[0..2].copy_from_slice(&config.address.identity.short_address.to_le_bytes());
        self.panadr[2..4].copy_from_slice(&config.address.identity.pan_id.to_le_bytes());
        self.sys_cfg = [0; LEN_SYS_CFG];
        self.sys_mask = [0; LEN_SYS_MASK];
        self.tx_fctrl = [0; LEN_TX_FCTRL];
        self.chan_ctrl = [0; LEN_CHAN_CTRL];

        set_bit(&mut self.sys_cfg, DIS_DRXB_BIT, true);
        set_bit(
            &mut self.sys_cfg,
            HIRQ_POL_BIT,
            config.interrupt_polarity_high,
        );
        set_bit(&mut self.sys_cfg, RXAUTR_BIT, config.receiver_auto_reenable);
        set_bit(&mut self.sys_cfg, DIS_STXP_BIT, !phy.smart_power);

        set_bit(&mut self.sys_mask, 7, true);
        set_bit(&mut self.sys_mask, 13, true);
        set_bit(&mut self.sys_mask, 14, true);
        set_bit(&mut self.sys_mask, 18, true);
        set_bit(&mut self.sys_mask, 15, true);
        set_bit(&mut self.sys_mask, 12, true);
        set_bit(&mut self.sys_mask, 16, true);
        set_bit(&mut self.sys_mask, 3, true);

        self.apply_phy_config(phy)?;
        let panadr = self.panadr;
        self.write_register(Register::PanAdr, NO_SUBADDRESS, &panadr)?;
        self.write_register(
            Register::Eui,
            NO_SUBADDRESS,
            &config.address.identity.eui.to_register_bytes(),
        )?;
        let sys_cfg = self.sys_cfg;
        let sys_mask = self.sys_mask;
        let chan_ctrl = self.chan_ctrl;
        let tx_fctrl = self.tx_fctrl;
        self.write_register(Register::SysCfg, NO_SUBADDRESS, &sys_cfg)?;
        self.write_register(Register::SysMask, NO_SUBADDRESS, &sys_mask)?;
        self.write_register(Register::ChanCtrl, NO_SUBADDRESS, &chan_ctrl)?;
        self.write_register(Register::TxFctrl, NO_SUBADDRESS, &tx_fctrl)?;
        self.apply_tuning(phy)?;
        let antenna = self.antenna_delay.to_time_bytes();
        self.write_register(Register::TxAntd, NO_SUBADDRESS, &antenna[..LEN_TX_ANTD])?;
        self.write_register(Register::LdeIf, LDE_RXANTD_SUB, &antenna[..LEN_LDE_RXANTD])?;
        Ok(())
    }

    /// Starts a receive session.
    pub fn start_receive(&mut self, options: RxOptions) -> Result<(), Error<SPI::Error, PinE>> {
        self.idle()?;
        self.clear_receive_status()?;
        self.sys_ctrl = [0; LEN_SYS_CTRL];
        self.state = DriverState::Rx;
        self.permanent_receive = options.permanent;
        self.rx_after_tx_pending = false;
        if let Some(delay) = options.delayed_time {
            let future = self.compute_delayed_time(delay)?;
            self.write_register(Register::DxTime, NO_SUBADDRESS, &future.to_bytes())?;
            set_bit(&mut self.sys_ctrl, RXDLYS_BIT, true);
        }
        set_bit(&mut self.sys_ctrl, SFCST_BIT, !self.frame_check);
        set_bit(&mut self.sys_ctrl, RXENAB_BIT, true);
        let sys_ctrl = self.sys_ctrl;
        self.write_register(Register::SysCtrl, NO_SUBADDRESS, &sys_ctrl)?;
        Ok(())
    }

    /// Transmits a frame.
    pub fn transmit(
        &mut self,
        frame: &[u8],
        options: TxOptions,
    ) -> Result<(), Error<SPI::Error, PinE>> {
        let frame_len = if self.frame_check {
            frame.len() + 2
        } else {
            frame.len()
        };
        if frame_len > LEN_EXT_UWB_FRAMES {
            return Err(Error::FrameTooLong {
                len: frame_len,
                max: LEN_EXT_UWB_FRAMES,
            });
        }
        if frame_len > LEN_UWB_FRAMES {
            return Err(Error::FrameTooLong {
                len: frame_len,
                max: LEN_UWB_FRAMES,
            });
        }

        self.idle()?;
        self.clear_transmit_status()?;
        self.write_register(Register::TxBuffer, NO_SUBADDRESS, frame)?;
        self.tx_fctrl[0] = (frame_len & 0xFF) as u8;
        self.tx_fctrl[1] &= 0xFC;
        self.tx_fctrl[1] |= ((frame_len >> 8) & 0x03) as u8;
        let tx_fctrl = self.tx_fctrl;
        self.write_register(Register::TxFctrl, NO_SUBADDRESS, &tx_fctrl)?;

        self.sys_ctrl = [0; LEN_SYS_CTRL];
        self.state = DriverState::Tx;
        self.rx_after_tx_pending = self.permanent_receive;
        set_bit(&mut self.sys_ctrl, SFCST_BIT, !self.frame_check);
        set_bit(&mut self.sys_ctrl, WAIT4RESP_BIT, options.wait_for_response);
        if let Some(delay) = options.delayed_time {
            let future = self.compute_delayed_time(delay)?;
            self.write_register(Register::DxTime, NO_SUBADDRESS, &future.to_bytes())?;
            set_bit(&mut self.sys_ctrl, TXDLYS_BIT, true);
        }
        set_bit(&mut self.sys_ctrl, TXSTRT_BIT, true);
        let sys_ctrl = self.sys_ctrl;
        self.write_register(Register::SysCtrl, NO_SUBADDRESS, &sys_ctrl)?;
        if !self.permanent_receive {
            self.state = DriverState::Idle;
        }
        Ok(())
    }

    /// Computes the delayed absolute transmit or receive timestamp used by the DW1000.
    pub fn compute_delayed_time(
        &mut self,
        delay: DwTime,
    ) -> Result<DwTime, Error<SPI::Error, PinE>> {
        let now = self.read_system_timestamp()?;
        let mut future = now + delay;
        let mut bytes = future.to_bytes();
        bytes[0] = 0;
        bytes[1] &= 0xFE;
        future = DwTime::from_bytes(&bytes);
        future += DwTime::from_ticks(self.antenna_delay.raw() as i64);
        Ok(future)
    }

    /// Reads a received frame into `buffer`.
    pub fn read_frame<'a>(
        &mut self,
        buffer: &'a mut [u8],
    ) -> Result<RxFrame<'a>, Error<SPI::Error, PinE>> {
        let status = self.read_sys_status()?;
        if status.contains(status::LDE_ERROR) {
            return Err(Error::Receive(RxError::LeadingEdgeDetection));
        }
        if status.contains(status::RX_FRAME_CHECK_ERROR) {
            return Err(Error::Receive(RxError::FrameCheck));
        }
        if status.contains(status::RX_HEADER_ERROR) {
            return Err(Error::Receive(RxError::Header));
        }
        if status.contains(status::RX_REED_SOLOMON_ERROR) {
            return Err(Error::Receive(RxError::ReedSolomon));
        }
        if status.contains(status::RX_TIMEOUT) {
            return Err(Error::Receive(RxError::Timeout));
        }

        let needed = self.read_received_length()?;
        if buffer.len() < needed {
            return Err(Error::BufferTooSmall {
                len: buffer.len(),
                needed,
            });
        }
        self.read_register(Register::RxBuffer, NO_SUBADDRESS, &mut buffer[..needed])?;
        let timestamp = self.read_receive_timestamp()?;
        let metrics = self.read_signal_metrics()?;
        Ok(RxFrame {
            bytes: &buffer[..needed],
            timestamp: self.correct_receive_timestamp(timestamp, metrics.receive_power_dbm),
            metrics,
            status,
        })
    }

    /// Reads the latest timestamps.
    pub fn read_timestamps(&mut self) -> Result<Timestamps, Error<SPI::Error, PinE>> {
        Ok(Timestamps {
            tx: self.read_transmit_timestamp()?,
            rx: self.read_receive_timestamp()?,
            system: self.read_system_timestamp()?,
        })
    }

    /// Reads the current receive metrics.
    pub fn read_signal_metrics(&mut self) -> Result<SignalMetrics, Error<SPI::Error, PinE>> {
        Ok(SignalMetrics {
            receive_power_dbm: self.read_receive_power()?,
            first_path_power_dbm: self.read_first_path_power()?,
            quality: self.read_receive_quality()?,
        })
    }

    /// Reads `SYS_STATUS`.
    pub fn read_sys_status(&mut self) -> Result<SysStatus, Error<SPI::Error, PinE>> {
        let mut bytes = [0u8; LEN_SYS_STATUS];
        self.read_register(Register::SysStatus, NO_SUBADDRESS, &mut bytes)?;
        Ok(sys_status_from_bytes(&bytes))
    }

    /// Clears the selected `SYS_STATUS` bits.
    pub fn clear_events(&mut self, event_mask: SysStatus) -> Result<(), Error<SPI::Error, PinE>> {
        self.write_register(
            Register::SysStatus,
            NO_SUBADDRESS,
            &sys_status_to_bytes(event_mask),
        )?;
        let should_restart_receive = self.rx_after_tx_pending
            && matches!(self.state, DriverState::Tx)
            && event_mask.contains(status::TX_FRAME_SENT);
        if should_restart_receive {
            self.start_receive(RxOptions {
                delayed_time: None,
                permanent: true,
            })?;
        }
        Ok(())
    }

    /// Returns the currently configured identity, if the radio has been configured.
    pub const fn identity(&self) -> Option<DeviceIdentity> {
        self.identity
    }

    /// Returns `true` if the IRQ pin is asserted.
    pub fn irq_asserted(&mut self) -> Result<bool, Error<SPI::Error, PinE>> {
        self.irq.is_high().map_err(Error::Pin)
    }

    fn hard_reset(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error, PinE>> {
        self.reset.set_low().map_err(Error::Pin)?;
        delay.delay_ms(2);
        self.reset.set_high().map_err(Error::Pin)?;
        delay.delay_ms(10);
        Ok(())
    }

    fn apply_phy_config(&mut self, phy: ValidatedPhyConfig) -> Result<(), Error<SPI::Error, PinE>> {
        self.set_data_rate(phy.data_rate)?;
        self.set_pulse_frequency(phy.pulse_frequency);
        self.set_preamble_length(phy.preamble_length);
        self.set_channel(phy.channel);
        self.set_preamble_code(phy.preamble_code);
        Ok(())
    }

    fn set_data_rate(&mut self, rate: DataRate) -> Result<(), Error<SPI::Error, PinE>> {
        self.tx_fctrl[1] &= 0x83;
        self.tx_fctrl[1] |= ((rate as u8) << 5) & 0xFF;
        set_bit(&mut self.sys_cfg, RXM110K_BIT, rate == DataRate::Kbps110);
        let (dwsfd, tnssfd, rnssfd, sfd_len) = match rate {
            DataRate::Mbps6800 => (false, false, false, 0x08),
            DataRate::Kbps850 => (true, true, true, 0x10),
            DataRate::Kbps110 => (true, false, false, 0x40),
        };
        set_bit(&mut self.chan_ctrl, DWSFD_BIT, dwsfd);
        set_bit(&mut self.chan_ctrl, TNSSFD_BIT, tnssfd);
        set_bit(&mut self.chan_ctrl, RNSSFD_BIT, rnssfd);
        self.write_register(Register::UsrSfd, SFD_LENGTH_SUB, &[sfd_len])?;
        Ok(())
    }

    fn set_pulse_frequency(&mut self, frequency: PulseFrequency) {
        self.tx_fctrl[2] &= 0xFC;
        self.tx_fctrl[2] |= frequency as u8;
        self.chan_ctrl[2] &= 0xF3;
        self.chan_ctrl[2] |= (frequency as u8) << 2;
    }

    fn set_preamble_length(&mut self, length: PreambleLength) {
        self.tx_fctrl[2] &= 0xC3;
        self.tx_fctrl[2] |= (length as u8) << 2;
    }

    fn set_channel(&mut self, channel: crate::config::Channel) {
        let channel = channel as u8;
        self.chan_ctrl[0] = channel | (channel << 4);
    }

    fn set_preamble_code(&mut self, code: PreambleCode) {
        let code = code.raw();
        self.chan_ctrl[2] &= 0x3F;
        self.chan_ctrl[2] |= code << 6;
        self.chan_ctrl[3] = ((code >> 2) & 0x07) | (code << 3);
    }

    fn apply_tuning(&mut self, phy: ValidatedPhyConfig) -> Result<(), Error<SPI::Error, PinE>> {
        let agc_tune1 = match phy.pulse_frequency {
            PulseFrequency::Mhz16 => 0x8870u16.to_le_bytes(),
            PulseFrequency::Mhz64 => 0x889Bu16.to_le_bytes(),
        };
        self.write_register(Register::AgcTune, AGC_TUNE1_SUB, &agc_tune1)?;
        self.write_register(
            Register::AgcTune,
            AGC_TUNE2_SUB,
            &0x2502_A907u32.to_le_bytes(),
        )?;
        self.write_register(Register::AgcTune, AGC_TUNE3_SUB, &0x0035u16.to_le_bytes())?;

        let drx_tune0b = match phy.data_rate {
            DataRate::Kbps110 => 0x0016u16,
            DataRate::Kbps850 => 0x0006u16,
            DataRate::Mbps6800 => 0x0001u16,
        };
        self.write_register(Register::DrxTune, DRX_TUNE0B_SUB, &drx_tune0b.to_le_bytes())?;

        let drx_tune1a = match phy.pulse_frequency {
            PulseFrequency::Mhz16 => 0x0087u16,
            PulseFrequency::Mhz64 => 0x008Du16,
        };
        self.write_register(Register::DrxTune, DRX_TUNE1A_SUB, &drx_tune1a.to_le_bytes())?;

        let drx_tune1b: u16 = match (phy.preamble_length, phy.data_rate) {
            (
                PreambleLength::Symbols1536
                | PreambleLength::Symbols2048
                | PreambleLength::Symbols4096,
                DataRate::Kbps110,
            ) => 0x0064,
            (PreambleLength::Symbols64, DataRate::Mbps6800) => 0x0010,
            (_, DataRate::Kbps850 | DataRate::Mbps6800) => 0x0020,
            _ => return Err(Error::InvalidConfig(ConfigError::UnsupportedPreambleLength)),
        };
        self.write_register(Register::DrxTune, DRX_TUNE1B_SUB, &drx_tune1b.to_le_bytes())?;

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
        self.write_register(Register::DrxTune, DRX_TUNE2_SUB, &drx_tune2.to_le_bytes())?;

        let drx_tune4h = match phy.preamble_length {
            PreambleLength::Symbols64 => 0x0010u16,
            _ => 0x0028u16,
        };
        self.write_register(Register::DrxTune, DRX_TUNE4H_SUB, &drx_tune4h.to_le_bytes())?;

        let rf_rxctrlh = match phy.channel {
            crate::config::Channel::Channel4 | crate::config::Channel::Channel7 => [0xBC],
            _ => [0xD8],
        };
        self.write_register(Register::RfConf, RF_RXCTRLH_SUB, &rf_rxctrlh)?;

        let rf_txctrl: u32 = match phy.channel {
            crate::config::Channel::Channel1 => 0x0000_5C40,
            crate::config::Channel::Channel2 => 0x0004_5CA0,
            crate::config::Channel::Channel3 => 0x0008_6CC0,
            crate::config::Channel::Channel4 => 0x0004_5C80,
            crate::config::Channel::Channel5 => 0x001E_3FE0,
            crate::config::Channel::Channel7 => 0x001E_7DE0,
        };
        self.write_register(Register::RfConf, RF_TXCTRL_SUB, &rf_txctrl.to_le_bytes())?;

        let tc_pgdelay = match phy.channel {
            crate::config::Channel::Channel1 => [0xC9],
            crate::config::Channel::Channel2 => [0xC2],
            crate::config::Channel::Channel3 => [0xC5],
            crate::config::Channel::Channel4 => [0x95],
            crate::config::Channel::Channel5 => [0xC0],
            crate::config::Channel::Channel7 => [0x93],
        };
        self.write_register(Register::TxCal, TC_PGDELAY_SUB, &tc_pgdelay)?;

        let (fspllcfg, fsplltune) = match phy.channel {
            crate::config::Channel::Channel1 => (0x0900_0407u32, [0x1E]),
            crate::config::Channel::Channel2 | crate::config::Channel::Channel4 => {
                (0x0840_0508u32, [0x26])
            }
            crate::config::Channel::Channel3 => (0x0840_1009u32, [0x56]),
            crate::config::Channel::Channel5 | crate::config::Channel::Channel7 => {
                (0x0800_041Du32, [0xBE])
            }
        };
        self.write_register(Register::FsCtrl, FS_PLLCFG_SUB, &fspllcfg.to_le_bytes())?;
        self.write_register(Register::FsCtrl, FS_PLLTUNE_SUB, &fsplltune)?;

        self.write_register(Register::LdeIf, LDE_CFG1_SUB, &[0x0D])?;
        let lde_cfg2 = match phy.pulse_frequency {
            PulseFrequency::Mhz16 => 0x1607u16,
            PulseFrequency::Mhz64 => 0x0607u16,
        };
        self.write_register(Register::LdeIf, LDE_CFG2_SUB, &lde_cfg2.to_le_bytes())?;
        let lde_repc = lde_repc_value(phy.preamble_code, phy.data_rate);
        self.write_register(Register::LdeIf, LDE_REPC_SUB, &lde_repc.to_le_bytes())?;

        let tx_power = tx_power_value(phy.channel, phy.pulse_frequency, phy.smart_power);
        self.write_register(Register::TxPower, NO_SUBADDRESS, &tx_power.to_le_bytes())?;

        let xtal_trim = self.read_otp(0x01E)?[0];
        let fs_xtalt = if xtal_trim == 0 {
            0x70
        } else {
            (xtal_trim & 0x1F) | 0x60
        };
        self.write_register(Register::FsCtrl, FS_XTALT_SUB, &[fs_xtalt])?;
        Ok(())
    }

    fn manage_lde(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error, PinE>> {
        let mut pmsc_ctrl0 = [0u8; LEN_PMSC_CTRL0];
        self.read_register(Register::Pmsc, PMSC_CTRL0_SUB, &mut pmsc_ctrl0)?;
        let mut otp_ctrl = [0u8; LEN_OTP_CTRL];
        self.read_register(Register::OtpIf, OTP_CTRL_SUB, &mut otp_ctrl)?;
        pmsc_ctrl0[0] = 0x01;
        pmsc_ctrl0[1] = 0x03;
        otp_ctrl[0] = 0x00;
        otp_ctrl[1] = 0x80;
        self.write_register(Register::Pmsc, PMSC_CTRL0_SUB, &pmsc_ctrl0[..2])?;
        self.write_register(Register::OtpIf, OTP_CTRL_SUB, &otp_ctrl)?;
        delay.delay_ms(5);
        pmsc_ctrl0[0] = 0x00;
        pmsc_ctrl0[1] &= 0x02;
        self.write_register(Register::Pmsc, PMSC_CTRL0_SUB, &pmsc_ctrl0[..2])?;
        Ok(())
    }

    fn enable_clock(&mut self, mode: ClockMode) -> Result<(), Error<SPI::Error, PinE>> {
        let mut pmsc_ctrl0 = [0u8; LEN_PMSC_CTRL0];
        self.read_register(Register::Pmsc, PMSC_CTRL0_SUB, &mut pmsc_ctrl0)?;
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
        self.write_register(Register::Pmsc, PMSC_CTRL0_SUB, &pmsc_ctrl0[..2])
    }

    fn read_otp(&mut self, address: u16) -> Result<[u8; LEN_OTP_RDAT], Error<SPI::Error, PinE>> {
        let mut address_bytes = [0u8; LEN_OTP_ADDR];
        address_bytes.copy_from_slice(&address.to_le_bytes());
        self.write_register(Register::OtpIf, OTP_ADDR_SUB, &address_bytes)?;
        self.write_register(Register::OtpIf, OTP_CTRL_SUB, &[0x03])?;
        self.write_register(Register::OtpIf, OTP_CTRL_SUB, &[0x01])?;
        let mut data = [0u8; LEN_OTP_RDAT];
        self.read_register(Register::OtpIf, OTP_RDAT_SUB, &mut data)?;
        self.write_register(Register::OtpIf, OTP_CTRL_SUB, &[0x00])?;
        Ok(data)
    }

    fn idle(&mut self) -> Result<(), Error<SPI::Error, PinE>> {
        self.sys_ctrl = [0; LEN_SYS_CTRL];
        set_bit(&mut self.sys_ctrl, TRXOFF_BIT, true);
        self.state = DriverState::Idle;
        let sys_ctrl = self.sys_ctrl;
        self.write_register(Register::SysCtrl, NO_SUBADDRESS, &sys_ctrl)
    }

    fn clear_interrupts(&mut self) -> Result<(), Error<SPI::Error, PinE>> {
        self.sys_mask = [0; LEN_SYS_MASK];
        let sys_mask = self.sys_mask;
        self.write_register(Register::SysMask, NO_SUBADDRESS, &sys_mask)
    }

    fn clear_receive_status(&mut self) -> Result<(), Error<SPI::Error, PinE>> {
        let mask = status::RX_FRAME_READY.0
            | status::LDE_DONE.0
            | status::LDE_ERROR.0
            | status::RX_HEADER_ERROR.0
            | status::RX_FRAME_CHECK_ERROR.0
            | status::RX_FRAME_GOOD.0
            | status::RX_REED_SOLOMON_ERROR.0
            | status::RX_TIMEOUT.0;
        self.clear_events(SysStatus(mask))
    }

    fn clear_transmit_status(&mut self) -> Result<(), Error<SPI::Error, PinE>> {
        let mask = status::TX_FRAME_BEGIN.0
            | status::TX_PREAMBLE_SENT.0
            | status::TX_HEADER_SENT.0
            | status::TX_FRAME_SENT.0;
        self.clear_events(SysStatus(mask))
    }

    fn read_received_length(&mut self) -> Result<usize, Error<SPI::Error, PinE>> {
        let mut info = [0u8; LEN_RX_FINFO];
        self.read_register(Register::RxFinfo, NO_SUBADDRESS, &mut info)?;
        let mut len = (((info[1] as usize) << 8) | info[0] as usize) & 0x03FF;
        if self.frame_check && len > 2 {
            len -= 2;
        }
        Ok(len)
    }

    fn read_transmit_timestamp(&mut self) -> Result<DwTime, Error<SPI::Error, PinE>> {
        let mut bytes = [0u8; LEN_TX_STAMP];
        self.read_register(Register::TxTime, TX_STAMP_SUB, &mut bytes)?;
        Ok(DwTime::from_bytes(&bytes))
    }

    fn read_receive_timestamp(&mut self) -> Result<DwTime, Error<SPI::Error, PinE>> {
        let mut bytes = [0u8; LEN_RX_STAMP];
        self.read_register(Register::RxTime, RX_STAMP_SUB, &mut bytes)?;
        Ok(DwTime::from_bytes(&bytes))
    }

    fn read_system_timestamp(&mut self) -> Result<DwTime, Error<SPI::Error, PinE>> {
        let mut bytes = [0u8; LEN_RX_STAMP];
        self.read_register(Register::SysTime, NO_SUBADDRESS, &mut bytes)?;
        Ok(DwTime::from_bytes(&bytes))
    }

    fn read_receive_quality(&mut self) -> Result<f32, Error<SPI::Error, PinE>> {
        let mut noise = [0u8; LEN_STD_NOISE];
        let mut fp2 = [0u8; LEN_FP_AMPL2];
        self.read_register(Register::RxFqual, STD_NOISE_SUB, &mut noise)?;
        self.read_register(Register::RxFqual, FP_AMPL2_SUB, &mut fp2)?;
        let noise = u16::from_le_bytes(noise);
        let fp2 = u16::from_le_bytes(fp2);
        Ok(fp2 as f32 / noise.max(1) as f32)
    }

    fn read_first_path_power(&mut self) -> Result<f32, Error<SPI::Error, PinE>> {
        let phy = self.phy.expect("phy configuration must be available");
        let mut fp1 = [0u8; LEN_FP_AMPL1];
        let mut fp2 = [0u8; LEN_FP_AMPL2];
        let mut fp3 = [0u8; LEN_FP_AMPL3];
        let mut info = [0u8; LEN_RX_FINFO];
        self.read_register(Register::RxTime, FP_AMPL1_SUB, &mut fp1)?;
        self.read_register(Register::RxFqual, FP_AMPL2_SUB, &mut fp2)?;
        self.read_register(Register::RxFqual, FP_AMPL3_SUB, &mut fp3)?;
        self.read_register(Register::RxFinfo, NO_SUBADDRESS, &mut info)?;
        let f1 = u16::from_le_bytes(fp1) as f32;
        let f2 = u16::from_le_bytes(fp2) as f32;
        let f3 = u16::from_le_bytes(fp3) as f32;
        let n = ((((info[2] as u16) >> 4) & 0xFF) | ((info[3] as u16) << 4)).max(1) as f32;
        let (a, corr) = match phy.pulse_frequency {
            PulseFrequency::Mhz16 => (113.77, 2.3334),
            PulseFrequency::Mhz64 => (121.74, 1.1667),
        };
        let mut power = 10.0 * log10f((f1 * f1 + f2 * f2 + f3 * f3) / (n * n)) - a;
        if power > -88.0 {
            power += (power + 88.0) * corr;
        }
        Ok(power)
    }

    fn read_receive_power(&mut self) -> Result<f32, Error<SPI::Error, PinE>> {
        let phy = self.phy.expect("phy configuration must be available");
        let mut cir = [0u8; LEN_CIR_PWR];
        let mut info = [0u8; LEN_RX_FINFO];
        self.read_register(Register::RxFqual, CIR_PWR_SUB, &mut cir)?;
        self.read_register(Register::RxFinfo, NO_SUBADDRESS, &mut info)?;
        let c = u16::from_le_bytes(cir) as f32;
        let n = ((((info[2] as u16) >> 4) & 0xFF) | ((info[3] as u16) << 4)).max(1) as f32;
        let (a, corr) = match phy.pulse_frequency {
            PulseFrequency::Mhz16 => (113.77, 2.3334),
            PulseFrequency::Mhz64 => (121.74, 1.1667),
        };
        let mut power = 10.0 * log10f((c * 131_072.0) / (n * n)) - a;
        if power > -88.0 {
            power += (power + 88.0) * corr;
        }
        Ok(power)
    }

    fn correct_receive_timestamp(&self, timestamp: DwTime, receive_power_dbm: f32) -> DwTime {
        let Some(phy) = self.phy else {
            return timestamp;
        };
        let rx_power_base = -(receive_power_dbm + 61.0) * 0.5;
        let mut low = libm::floorf(rx_power_base) as isize;
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
        let adjustment =
            DwTime::from_ticks((bias_mm * 0.001 / crate::time::DISTANCE_PER_TICK_M) as i64);
        timestamp - adjustment
    }

    fn read_register(
        &mut self,
        register: Register,
        subaddress: u16,
        buffer: &mut [u8],
    ) -> Result<(), Error<SPI::Error, PinE>> {
        let header = build_header(register, subaddress, false);
        let header_len = header_len(subaddress);
        let mut operations = [
            Operation::Write(&header[..header_len]),
            Operation::Read(buffer),
        ];
        self.spi.transaction(&mut operations).map_err(Error::Spi)
    }

    fn write_register(
        &mut self,
        register: Register,
        subaddress: u16,
        data: &[u8],
    ) -> Result<(), Error<SPI::Error, PinE>> {
        let header = build_header(register, subaddress, true);
        let header_len = header_len(subaddress);
        let mut operations = [
            Operation::Write(&header[..header_len]),
            Operation::Write(data),
        ];
        self.spi.transaction(&mut operations).map_err(Error::Spi)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ClockMode {
    Auto,
    Xti,
}

fn build_header(register: Register, subaddress: u16, write: bool) -> [u8; 3] {
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

fn header_len(subaddress: u16) -> usize {
    if subaddress == NO_SUBADDRESS {
        1
    } else if subaddress < 128 {
        2
    } else {
        3
    }
}

fn set_bit(bytes: &mut [u8], bit: u16, value: bool) {
    let index = (bit / 8) as usize;
    let shift = (bit % 8) as u8;
    if value {
        bytes[index] |= 1 << shift;
    } else {
        bytes[index] &= !(1 << shift);
    }
}

fn lde_repc_value(code: PreambleCode, rate: DataRate) -> u16 {
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

fn tx_power_value(channel: crate::config::Channel, prf: PulseFrequency, smart_power: bool) -> u32 {
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
