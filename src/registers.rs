//! Typed DW1000 register addresses, subaddresses, and bit positions.

use crate::device::SysStatus;

/// Registers accessed by the driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Register {
    /// Device identifier register.
    DevId = 0x00,
    /// Extended identifier register.
    Eui = 0x01,
    /// PAN ID and short address register.
    PanAdr = 0x03,
    /// System configuration register.
    SysCfg = 0x04,
    /// System time counter.
    SysTime = 0x06,
    /// TX frame control.
    TxFctrl = 0x08,
    /// TX data buffer.
    TxBuffer = 0x09,
    /// Delayed TX/RX time register.
    DxTime = 0x0A,
    /// System control register.
    SysCtrl = 0x0D,
    /// Interrupt mask register.
    SysMask = 0x0E,
    /// System status register.
    SysStatus = 0x0F,
    /// RX frame info.
    RxFinfo = 0x10,
    /// RX data buffer.
    RxBuffer = 0x11,
    /// RX frame quality.
    RxFqual = 0x12,
    /// RX timestamp register.
    RxTime = 0x15,
    /// TX timestamp register.
    TxTime = 0x17,
    /// TX antenna delay.
    TxAntd = 0x18,
    /// Channel control.
    ChanCtrl = 0x1F,
    /// User-defined SFD register.
    UsrSfd = 0x21,
    /// AGC tuning.
    AgcTune = 0x23,
    /// DRX tuning.
    DrxTune = 0x27,
    /// RF configuration.
    RfConf = 0x28,
    /// TX power register.
    TxPower = 0x1E,
    /// TX calibration.
    TxCal = 0x2A,
    /// Frequency synthesizer control.
    FsCtrl = 0x2B,
    /// OTP interface.
    OtpIf = 0x2D,
    /// LDE interface.
    LdeIf = 0x2E,
    /// Power management and system clocking.
    Pmsc = 0x36,
    /// GPIO control.
    GpioCtrl = 0x26,
}

/// Special value meaning "no subaddress".
pub const NO_SUBADDRESS: u16 = 0xFFFF;

/// Expected `DEV_ID` value for a DW1000.
pub const EXPECTED_DEVICE_ID: u32 = 0xDECA_0130;

/// DEV_ID length.
pub const LEN_DEV_ID: usize = 4;
/// EUI length.
pub const LEN_EUI: usize = 8;
/// PAN/short-address register length.
pub const LEN_PANADR: usize = 4;
/// SYS_CFG length.
pub const LEN_SYS_CFG: usize = 4;
/// SYS_CTRL length.
pub const LEN_SYS_CTRL: usize = 4;
/// SYS_MASK length.
pub const LEN_SYS_MASK: usize = 4;
/// SYS_STATUS length.
pub const LEN_SYS_STATUS: usize = 5;
/// TX_FCTRL length.
pub const LEN_TX_FCTRL: usize = 5;
/// CHAN_CTRL length.
pub const LEN_CHAN_CTRL: usize = 4;
/// RX_FINFO length.
pub const LEN_RX_FINFO: usize = 4;
/// Full RX_TIME register length (stamp + first-path index/amplitude + raw stamp).
pub const LEN_RX_TIME: usize = 14;
/// RX_TIME length.
pub const LEN_RX_STAMP: usize = 5;
/// TX_TIME length.
pub const LEN_TX_STAMP: usize = 5;
/// RX_FQUAL register length.
pub const LEN_RX_FQUAL: usize = 8;
/// PMSC control field length.
pub const LEN_PMSC_CTRL0: usize = 4;
/// PMSC LED control field length.
pub const LEN_PMSC_LEDC: usize = 4;
/// OTP address field length.
pub const LEN_OTP_ADDR: usize = 2;
/// OTP control field length.
pub const LEN_OTP_CTRL: usize = 2;
/// OTP data length.
pub const LEN_OTP_RDAT: usize = 4;
/// TX antenna delay field length.
pub const LEN_TX_ANTD: usize = 2;
/// LDE RX antenna delay length.
pub const LEN_LDE_RXANTD: usize = 2;

/// RX timestamp subaddress.
pub const RX_STAMP_SUB: u16 = 0x00;
/// TX timestamp subaddress.
pub const TX_STAMP_SUB: u16 = 0x00;
/// PMSC control 0 subaddress.
pub const PMSC_CTRL0_SUB: u16 = 0x00;
/// PMSC SOFTRESET field subaddress (byte 3 of PMSC_CTRL0).
pub const PMSC_SOFTRESET_SUB: u16 = 0x03;
/// PMSC LED control subaddress.
pub const PMSC_LEDC_SUB: u16 = 0x28;
/// OTP address subaddress.
pub const OTP_ADDR_SUB: u16 = 0x04;
/// OTP control subaddress.
pub const OTP_CTRL_SUB: u16 = 0x06;
/// OTP special-function subaddress (LDO tune kick).
pub const OTP_SF_SUB: u16 = 0x12;
/// OTP data subaddress.
pub const OTP_RDAT_SUB: u16 = 0x0A;
/// User SFD length subaddress.
pub const SFD_LENGTH_SUB: u16 = 0x00;
/// AGC tuning subaddresses.
pub const AGC_TUNE1_SUB: u16 = 0x04;
/// AGC tuning subaddresses.
pub const AGC_TUNE2_SUB: u16 = 0x0C;
/// AGC tuning subaddresses.
pub const AGC_TUNE3_SUB: u16 = 0x12;
/// DRX tune 0b subaddress.
pub const DRX_TUNE0B_SUB: u16 = 0x02;
/// DRX tune 1a subaddress.
pub const DRX_TUNE1A_SUB: u16 = 0x04;
/// DRX tune 1b subaddress.
pub const DRX_TUNE1B_SUB: u16 = 0x06;
/// DRX tune 2 subaddress.
pub const DRX_TUNE2_SUB: u16 = 0x08;
/// DRX tune 4H subaddress.
pub const DRX_TUNE4H_SUB: u16 = 0x26;
/// DRX SFD detection timeout subaddress.
pub const DRX_SFDTOC_SUB: u16 = 0x20;
/// LDE config 1 subaddress.
pub const LDE_CFG1_SUB: u16 = 0x0806;
/// LDE config 2 subaddress.
pub const LDE_CFG2_SUB: u16 = 0x1806;
/// LDE repetition counter subaddress.
pub const LDE_REPC_SUB: u16 = 0x2804;
/// LDE RX antenna delay subaddress.
pub const LDE_RXANTD_SUB: u16 = 0x1804;
/// RF RX control high subaddress.
pub const RF_RXCTRLH_SUB: u16 = 0x0B;
/// RF TX control subaddress.
pub const RF_TXCTRL_SUB: u16 = 0x0C;
/// TX calibration PG delay subaddress.
pub const TC_PGDELAY_SUB: u16 = 0x0B;
/// FS PLL configuration subaddress.
pub const FS_PLLCFG_SUB: u16 = 0x07;
/// FS PLL tune subaddress.
pub const FS_PLLTUNE_SUB: u16 = 0x0B;
/// FS XTAL trim subaddress.
pub const FS_XTALT_SUB: u16 = 0x0E;

/// SYS_CFG bits.
pub const HIRQ_POL_BIT: u16 = 9;
/// SYS_CFG bits.
pub const DIS_DRXB_BIT: u16 = 12;
/// SYS_CFG bits.
pub const DIS_STXP_BIT: u16 = 18;
/// SYS_CFG bits.
pub const RXM110K_BIT: u16 = 22;
/// SYS_CFG bits.
pub const RXAUTR_BIT: u16 = 29;
/// SYS_CTRL bits.
pub const SFCST_BIT: u16 = 0;
/// SYS_CTRL bits.
pub const TXSTRT_BIT: u16 = 1;
/// SYS_CTRL bits.
pub const TXDLYS_BIT: u16 = 2;
/// SYS_CTRL bits.
pub const TRXOFF_BIT: u16 = 6;
/// SYS_CTRL bits.
pub const WAIT4RESP_BIT: u16 = 7;
/// SYS_CTRL bits.
pub const RXENAB_BIT: u16 = 8;
/// SYS_CTRL bits.
pub const RXDLYS_BIT: u16 = 9;
/// SYS_MASK bit: TX frame sent interrupt.
pub const MTXFRS_BIT: u16 = 7;
/// SYS_MASK bit: RX frame ready interrupt.
pub const MRXDFR_BIT: u16 = 13;
/// SYS_MASK bit: RX frame good (FCS OK) interrupt.
pub const MRXFCG_BIT: u16 = 14;
/// SYS_MASK bit: RX frame check error interrupt.
pub const MRXFCE_BIT: u16 = 15;
/// SYS_MASK bit: RX Reed-Solomon error interrupt.
pub const MRXFSL_BIT: u16 = 16;
/// SYS_MASK bit: RX PHY header error interrupt.
pub const MRXPHE_BIT: u16 = 12;
/// SYS_MASK bit: leading-edge detection error interrupt.
pub const MLDEERR_BIT: u16 = 18;
/// SYS_MASK bit: receive frame wait timeout interrupt.
pub const MRXRFTO_BIT: u16 = 17;
/// SYS_MASK bit: preamble detection timeout interrupt.
pub const MRXPTO_BIT: u16 = 21;
/// SYS_MASK bit: receive SFD timeout interrupt.
pub const MRXSFDTO_BIT: u16 = 26;
/// SYS_MASK bit: automatic frame filtering rejection interrupt.
pub const MAFFREJ_BIT: u16 = 29;
/// CHAN_CTRL bits.
pub const DWSFD_BIT: u16 = 17;
/// CHAN_CTRL bits.
pub const TNSSFD_BIT: u16 = 20;
/// CHAN_CTRL bits.
pub const RNSSFD_BIT: u16 = 21;

/// GPIO mode subaddress within GPIO_CTRL.
pub const GPIO_MODE_SUB: u16 = 0x00;
/// GPIO mode field length.
pub const LEN_GPIO_MODE: usize = 4;
/// GPIO_MODE bit position: mode selection for GPIO0/RXOKLED.
pub const MSGP0_BIT: u16 = 6;
/// GPIO_MODE bit position: mode selection for GPIO1/SFDLED.
pub const MSGP1_BIT: u16 = 8;
/// GPIO_MODE bit position: mode selection for GPIO2/RXLED.
pub const MSGP2_BIT: u16 = 10;
/// GPIO_MODE bit position: mode selection for GPIO3/TXLED.
pub const MSGP3_BIT: u16 = 12;
/// GPIO_MODE field value selecting plain GPIO operation.
pub const GPIO_MODE_GPIO: u8 = 0;
/// GPIO_MODE field value selecting the LED function.
pub const GPIO_MODE_LED: u8 = 1;
/// PMSC_CTRL0 bit: GPIO de-bounce clock enable.
pub const GPDCE_BIT: u16 = 18;
/// PMSC_CTRL0 bit: kilohertz clock enable.
pub const KHZCLKEN_BIT: u16 = 23;
/// PMSC_LEDC bit: blink enable.
pub const BLNKEN_BIT: u16 = 8;

/// Common `SYS_STATUS` bits represented as a 64-bit mask.
pub mod status {
    use crate::device::SysStatus;

    /// Automatic acknowledge trigger.
    pub const AUTO_ACK_TRIGGER: SysStatus = SysStatus(1u64 << 3);
    /// TX frame sent.
    pub const TX_FRAME_SENT: SysStatus = SysStatus(1u64 << 7);
    /// Receiver preamble detected.
    pub const RX_PREAMBLE_DETECTED: SysStatus = SysStatus(1u64 << 8);
    /// Receiver SFD detected.
    pub const RX_SFD_DETECTED: SysStatus = SysStatus(1u64 << 9);
    /// Receiver PHY header detected.
    pub const RX_PHY_HEADER_DETECTED: SysStatus = SysStatus(1u64 << 11);
    /// Receive data frame ready.
    pub const RX_FRAME_READY: SysStatus = SysStatus(1u64 << 13);
    /// Receive frame check good.
    pub const RX_FRAME_GOOD: SysStatus = SysStatus(1u64 << 14);
    /// Receive frame check error.
    pub const RX_FRAME_CHECK_ERROR: SysStatus = SysStatus(1u64 << 15);
    /// Receive Reed-Solomon sync loss.
    pub const RX_REED_SOLOMON_ERROR: SysStatus = SysStatus(1u64 << 16);
    /// Receive frame wait timeout.
    pub const RX_TIMEOUT: SysStatus = SysStatus(1u64 << 17);
    /// Leading-edge detection done.
    pub const LDE_DONE: SysStatus = SysStatus(1u64 << 10);
    /// Leading-edge detection error.
    pub const LDE_ERROR: SysStatus = SysStatus(1u64 << 18);
    /// RX PHY header error.
    pub const RX_HEADER_ERROR: SysStatus = SysStatus(1u64 << 12);
    /// Receiver overrun.
    pub const RX_OVERRUN: SysStatus = SysStatus(1u64 << 20);
    /// Preamble detection timeout.
    pub const RX_PREAMBLE_TIMEOUT: SysStatus = SysStatus(1u64 << 21);
    /// Receive SFD timeout.
    pub const RX_SFD_TIMEOUT: SysStatus = SysStatus(1u64 << 26);
    /// Half period delay warning (delayed TX/RX programmed too late).
    pub const HALF_PERIOD_DELAY_WARNING: SysStatus = SysStatus(1u64 << 27);
    /// Automatic frame filtering rejection.
    pub const FRAME_FILTER_REJECTION: SysStatus = SysStatus(1u64 << 29);
    /// TX frame begin.
    pub const TX_FRAME_BEGIN: SysStatus = SysStatus(1u64 << 4);
    /// TX preamble sent.
    pub const TX_PREAMBLE_SENT: SysStatus = SysStatus(1u64 << 5);
    /// TX PHY header sent.
    pub const TX_HEADER_SENT: SysStatus = SysStatus(1u64 << 6);

    /// All events reported for a correctly received frame
    /// (`SYS_STATUS_ALL_RX_GOOD` in the official driver).
    pub const ALL_RX_GOOD: SysStatus = SysStatus(
        RX_FRAME_READY.0
            | RX_FRAME_GOOD.0
            | RX_PREAMBLE_DETECTED.0
            | RX_SFD_DETECTED.0
            | RX_PHY_HEADER_DETECTED.0
            | LDE_DONE.0,
    );
    /// All receive error events (`SYS_STATUS_ALL_RX_ERR` in the official driver).
    pub const ALL_RX_ERRORS: SysStatus = SysStatus(
        RX_HEADER_ERROR.0
            | RX_FRAME_CHECK_ERROR.0
            | RX_REED_SOLOMON_ERROR.0
            | RX_SFD_TIMEOUT.0
            | FRAME_FILTER_REJECTION.0
            | LDE_ERROR.0
            | RX_OVERRUN.0,
    );
    /// All receive timeout events (`SYS_STATUS_ALL_RX_TO` in the official driver).
    pub const ALL_RX_TIMEOUTS: SysStatus = SysStatus(RX_TIMEOUT.0 | RX_PREAMBLE_TIMEOUT.0);
    /// Every receive-related event the driver reacts to.
    pub const ALL_RX_EVENTS: SysStatus =
        SysStatus(ALL_RX_GOOD.0 | ALL_RX_ERRORS.0 | ALL_RX_TIMEOUTS.0);
    /// All transmit events (`SYS_STATUS_ALL_TX` in the official driver).
    pub const ALL_TX: SysStatus = SysStatus(
        AUTO_ACK_TRIGGER.0
            | TX_FRAME_BEGIN.0
            | TX_PREAMBLE_SENT.0
            | TX_HEADER_SENT.0
            | TX_FRAME_SENT.0,
    );
}

/// Converts a little-endian bitfield slice into `SysStatus`.
pub fn sys_status_from_bytes(bytes: &[u8; LEN_SYS_STATUS]) -> SysStatus {
    let mut raw = 0u64;
    let mut index = 0usize;
    while index < LEN_SYS_STATUS {
        raw |= (bytes[index] as u64) << (index * 8);
        index += 1;
    }
    SysStatus(raw)
}

/// Encodes a `SysStatus` mask back into the 5-byte wire format.
pub fn sys_status_to_bytes(status: SysStatus) -> [u8; LEN_SYS_STATUS] {
    let mut bytes = [0u8; LEN_SYS_STATUS];
    let mut index = 0usize;
    while index < LEN_SYS_STATUS {
        bytes[index] = ((status.0 >> (index * 8)) & 0xFF) as u8;
        index += 1;
    }
    bytes
}
