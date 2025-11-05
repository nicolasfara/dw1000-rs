use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::{Operation, SpiDevice};
use crate::{ClockMode, DeviceMode};
use crate::config::{DataRate, PacSize, PreambleLength, PulseFrequency};
use crate::constants::{CHAN_CTRL, DIS_DRXB_BIT, DWSFD_BIT, EUI, HIRQ_POL_BIT, LDE_IF, LDE_RXANTD_SUB, LEN_CHAN_CTRL, LEN_EUI, LEN_OTP_ADDR, LEN_OTP_CTRL, LEN_OTP_RDAT, LEN_PANADR, LEN_PMSC_CTRL0, LEN_SYS_CFG, LEN_SYS_CTRL, LEN_SYS_MASK, LEN_TX_FCTRL, NO_SUB, OTP_ADDR_SUB, OTP_CTRL_SUB, OTP_IF, OTP_RDAT_SUB, PANADR, PMSC, PMSC_CTRL0_SUB, RNSSFD_BIT, RXM110K_BIT, SFD_LENGTH_SUB, SYS_CFG, SYS_CTRL, SYS_MASK, TNSSFD_BIT, TRXOFF_BIT, TX_ANTD, TX_FCTRL, USR_SFD};

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Dw1000Error<SPI> {
    SpiError(SPI),
    RestError,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Dw1000<SPI, IRQ, RST, DELAY> {
    spi: SPI,
    irq: IRQ,
    rst: RST,
    delay: DELAY,
    sysctrl: [u8; LEN_SYS_CTRL],
    syscfg: [u8; LEN_SYS_CFG],
    sysmask: [u8; LEN_SYS_MASK],
    txfctrl: [u8; LEN_TX_FCTRL],
    chanctrl: [u8; LEN_CHAN_CTRL],
    network_and_address: [u8; LEN_PANADR],
    device_mode: DeviceMode,
    data_rate: DataRate,
    pulse_frequency: PulseFrequency,
    preamble_length: PreambleLength,
    pac_size: PacSize,
    antenna_delay: u16,
    antenna_calibrated: bool,
    vmeas3v3: u8,
    tmeas23c: u8,
}

impl<SPI, IRQ, RST, DELAY> Dw1000<SPI, IRQ, RST, DELAY>
where
    SPI: SpiDevice,
    IRQ: InputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    const WRITE: u8 = 0x80;
    const WRITE_SUB: u8 = 0xC0;
    const READ: u8 = 0x00;
    const READ_SUB: u8 = 0x40;
    const RW_SUB_EXT: u8 = 0x80;

    pub fn new(spi: SPI, irq: IRQ, rst: RST, delay: DELAY) -> Self {
        Dw1000 {
            spi,
            irq,
            rst,
            delay,
            sysctrl: [0u8; LEN_SYS_CTRL],
            syscfg: [0u8; LEN_SYS_CFG],
            sysmask: [0u8; LEN_SYS_MASK],
            txfctrl: [0u8; LEN_TX_FCTRL],
            chanctrl: [0u8; LEN_CHAN_CTRL],
            network_and_address: [0u8; LEN_PANADR],
            device_mode: DeviceMode::Idle,
            data_rate: DataRate::Kbps110,
            pulse_frequency: PulseFrequency::Mhz16,
            preamble_length: PreambleLength::Symbols2048,
            pac_size: PacSize::Symbols64,
            antenna_delay: 0,
            antenna_calibrated: false,
            vmeas3v3: 0,
            tmeas23c: 0,
        }
    }

    pub fn init(&mut self) -> Result<(), Dw1000Error<SPI::Error>> {
        #[cfg(feature = "defmt")]
        defmt::debug!("Initializing DW1000 with clock auto");
        self.enable_clock(ClockMode::Auto)?;
        self.delay.delay_ms(5);
        #[cfg(feature = "defmt")]
        defmt::debug!("Performing soft reset");
        self.reset()?;
        self.network_and_address = [0xFF; LEN_PANADR];
        let addr = self.network_and_address.clone();
        self.write_bytes(PANADR, NO_SUB as u16, &addr)?;
        self.syscfg = [0u8; LEN_SYS_CFG];
        self.set_double_buffering(false);
        self.set_interrupt_polarity(true);
        self.write_system_configuration_register()?;
        self.clear_interrupts();
        self.write_system_event_mask_register()?;
        self.enable_clock(ClockMode::Xti)?;
        self.delay.delay_ms(5);
        self.manage_lde()?;
        self.delay.delay_ms(5);
        self.enable_clock(ClockMode::Auto)?;
        self.delay.delay_ms(5);
        let mut buf_otp = [0u8, 4];
        self.read_bytes_otp(0x008, &mut buf_otp)?;
        #[cfg(feature = "defmt")]
        defmt::debug!("OTP Read 0x008: {:?}", buf_otp);
        self.vmeas3v3 = buf_otp[0];
        #[cfg(feature = "defmt")]
        defmt::debug!("Vmeas3v3: {}", self.vmeas3v3);
        self.read_bytes_otp(0x009, &mut buf_otp)?;
        #[cfg(feature = "defmt")]
        defmt::debug!("OTP Read 0x009: {:?}", buf_otp);
        self.tmeas23c = buf_otp[0];
        #[cfg(feature = "defmt")]
        {
            defmt::debug!("Tmeas23c: {}", self.tmeas23c);
            defmt::debug!("DW1000 initialization complete");
        }
        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), Dw1000Error<SPI::Error>> {
        self.soft_reset()?;
        self.rst.set_low().map_err(|_| Dw1000Error::RestError)?;
        self.delay.delay_ms(2);
        self.rst.set_high().map_err(|_| Dw1000Error::RestError)
    }

    pub fn soft_reset(&mut self) -> Result<(), Dw1000Error<SPI::Error>> {
        let mut pmscctrl0 = [0u8; LEN_PMSC_CTRL0];
        self.read_bytes(PMSC, PMSC_CTRL0_SUB as u16, &mut pmscctrl0)?;
        #[cfg(feature = "defmt")]
        defmt::debug!("PMSC CTRL0 before soft reset: {:?}", pmscctrl0);
        pmscctrl0[0] = 0x01;
        self.write_bytes(PMSC, PMSC_CTRL0_SUB as u16, &mut pmscctrl0)?;
        pmscctrl0[3] = 0x00;
        self.write_bytes(PMSC, PMSC_CTRL0_SUB as u16, &mut pmscctrl0)?;
        self.delay.delay_ms(10);
        pmscctrl0[0] = 0x00;
        pmscctrl0[3] = 0xF0;
        self.write_bytes(PMSC, PMSC_CTRL0_SUB as u16, &mut pmscctrl0)?;
        self.idle()
    }

    pub fn idle(&mut self) -> Result<(), Dw1000Error<SPI::Error>> {
        self.sysctrl = [0u8; LEN_SYS_CTRL];
        Self::set_bit(&mut self.sysctrl, TRXOFF_BIT as u16, true);
        self.device_mode = DeviceMode::Idle;
        let data = self.sysctrl.clone();
        self.write_bytes(SYS_CTRL, NO_SUB as u16, &data)
    }

    pub fn set_eui(&mut self, eui: &str) -> Result<(), Dw1000Error<SPI::Error>> {
        let eui_bytes = Self::convert_to_byte(eui);
        self.set_eui_array(&eui_bytes)
    }

    /// Convert a hex string like "AA:FF:1C:..." to a byte array
    fn convert_to_byte(value: &str) -> [u8; LEN_EUI] {
        let mut eui_byte = [0u8; LEN_EUI];
        // Fill the array from the string in the form of "AA:FF:1C:..."
        for i in 0..LEN_EUI {
            let high_nibble = Self::nibble_from_char(value.as_bytes()[i * 3] as char);
            let low_nibble = Self::nibble_from_char(value.as_bytes()[i * 3 + 1] as char);
            eui_byte[i] = (high_nibble << 4) + low_nibble;
        }
        eui_byte
    }

    /// Convert a hex character to its nibble value (0-15)
    fn nibble_from_char(c: char) -> u8 {
        match c {
            '0'..='9' => c as u8 - b'0',
            'a'..='f' => c as u8 - b'a' + 10,
            'A'..='F' => c as u8 - b'A' + 10,
            _ => 255,
        }
    }

    fn set_eui_array(&mut self, eui: &[u8; 8]) -> Result<(), Dw1000Error<SPI::Error>> {
        let mut reversed_eui = [0u8; 8];
        for i in 0..8 {
            reversed_eui[i] = eui[7 - i];
        }
        self.write_bytes(EUI, NO_SUB as u16, &reversed_eui)
    }

    pub fn set_device_address(&mut self, value: u16) {
        self.network_and_address[2] = (value & 0xFF) as u8;
        self.network_and_address[3] = ((value >> 8) & 0xFF) as u8;
    }

    pub fn set_network_id(&mut self, value: u16) {
        self.network_and_address[0] = (value & 0xFF) as u8;
        self.network_and_address[1] = ((value >> 8) & 0xFF) as u8;
    }

    pub fn enable_mode(&mut self, mode: &[u8]) {

    }

    pub fn set_data_rate(&mut self, rate: DataRate) -> Result<(), Dw1000Error<SPI::Error>> {
        let rate_value = rate as u8;
        // Set the data rate in TX_FCTRL register (bits 5-6 of byte 1)
        self.txfctrl[1] &= 0x83; // Clear bits 5-6
        self.txfctrl[1] |= (rate_value << 5) & 0xFF;
        // Special 110kbps flag in SYS_CFG
        if rate == DataRate::Kbps110 {
            Self::set_bit(&mut self.syscfg, RXM110K_BIT as u16, true);
        } else {
            Self::set_bit(&mut self.syscfg, RXM110K_BIT as u16, false);
        }
        // SFD mode and type configuration based on data rate
        let sfd_length: u8 = match rate {
            DataRate::Mbps6800 => {
                // 6.8 Mbps: standard SFD
                Self::set_bit(&mut self.chanctrl, DWSFD_BIT as u16, false);
                Self::set_bit(&mut self.chanctrl, TNSSFD_BIT as u16, false);
                Self::set_bit(&mut self.chanctrl, RNSSFD_BIT as u16, false);
                0x08
            }
            DataRate::Kbps850 => {
                // 850 kbps: non-standard SFD (Decawave proprietary)
                Self::set_bit(&mut self.chanctrl, DWSFD_BIT as u16, true);
                Self::set_bit(&mut self.chanctrl, TNSSFD_BIT as u16, true);
                Self::set_bit(&mut self.chanctrl, RNSSFD_BIT as u16, true);
                0x10
            }
            DataRate::Kbps110 => {
                // 110 kbps: non-standard SFD (Decawave proprietary, RX only)
                Self::set_bit(&mut self.chanctrl, DWSFD_BIT as u16, true);
                Self::set_bit(&mut self.chanctrl, TNSSFD_BIT as u16, false);
                Self::set_bit(&mut self.chanctrl, RNSSFD_BIT as u16, false);
                0x40
            }
        };
        // Write SFD length
        self.write_bytes(USR_SFD, SFD_LENGTH_SUB as u16, &[sfd_length])?;

        self.data_rate = rate;

        Ok(())
    }

    pub fn set_pulse_frequency(&mut self, freq: PulseFrequency) {
        let freq_value = freq as u8;

        // Set pulse frequency in TX_FCTRL register (bits 0-1 of byte 2)
        self.txfctrl[2] &= 0xFC; // Clear bits 0-1
        self.txfctrl[2] |= freq_value & 0xFF;

        // Set pulse frequency in CHAN_CTRL register (bits 2-3 of byte 2)
        self.chanctrl[2] &= 0xF3; // Clear bits 2-3
        self.chanctrl[2] |= (freq_value << 2) & 0xFF;

        self.pulse_frequency = freq;
    }

    pub fn set_preamble_length(&mut self, prealen: PreambleLength) {
        let prealen_value = prealen as u8;

        // Set preamble length in TX_FCTRL register (bits 2-5 of byte 2)
        self.txfctrl[2] &= 0xC3; // Clear bits 2-5
        self.txfctrl[2] |= (prealen_value << 2) & 0xFF;

        // Determine PAC size based on preamble length
        // According to DW1000 User Manual Table 8
        self.pac_size = match prealen {
            PreambleLength::Symbols64 | PreambleLength::Symbols128 => PacSize::Symbols8,
            PreambleLength::Symbols256 | PreambleLength::Symbols512 => PacSize::Symbols16,
            PreambleLength::Symbols1024 => PacSize::Symbols32,
            _ => PacSize::Symbols64, // 1536, 2048, 4096
        };

        self.preamble_length = prealen;
    }

    /// Commit all configuration changes to the DW1000 device
    ///
    /// This method writes all cached configuration registers back to the device
    /// and performs device tuning according to the current configuration.
    /// It also sets the antenna delay if not already calibrated.
    pub fn commit_configuration(&mut self) -> Result<(), Dw1000Error<SPI::Error>> {
        // Write network ID and device address
        self.write_network_id_and_device_address()?;

        // Write system configuration register
        self.write_system_configuration_register()?;

        // Write channel control register
        self.write_channel_control_register()?;

        // Write transmit frame control register
        self.write_transmit_frame_control_register()?;

        // Write system event mask register
        self.write_system_event_mask_register()?;

        // TODO: Implement tune() method for full device tuning
        // tune()?;

        // Set default antenna delay if not calibrated
        if self.antenna_delay == 0 && !self.antenna_calibrated {
            self.antenna_delay = 16384;
            self.antenna_calibrated = true;
        }

        // Write antenna delay to both TX and RX registers
        let antenna_delay_bytes = [
            (self.antenna_delay & 0xFF) as u8,
            ((self.antenna_delay >> 8) & 0xFF) as u8,
        ];

        self.write_bytes(TX_ANTD, NO_SUB as u16, &antenna_delay_bytes)?;
        self.write_bytes(LDE_IF, LDE_RXANTD_SUB, &antenna_delay_bytes)?;

        Ok(())
    }

    /// Write network ID and device address register to the device
    fn write_network_id_and_device_address(&mut self) -> Result<(), Dw1000Error<SPI::Error>> {
        let data = self.network_and_address;
        self.write_bytes(PANADR, NO_SUB as u16, &data)
    }

    /// Write channel control register to the device
    fn write_channel_control_register(&mut self) -> Result<(), Dw1000Error<SPI::Error>> {
        let data = self.chanctrl;
        self.write_bytes(CHAN_CTRL, NO_SUB as u16, &data)
    }

    /// Write transmit frame control register to the device
    fn write_transmit_frame_control_register(&mut self) -> Result<(), Dw1000Error<SPI::Error>> {
        let data = self.txfctrl;
        self.write_bytes(TX_FCTRL, NO_SUB as u16, &data)
    }

    fn read_bytes_otp(&mut self, address: u16, data: &mut [u8]) -> Result<(), Dw1000Error<SPI::Error>> {
        let mut address_bytes = [0u8; LEN_OTP_ADDR];
        address_bytes[0] = (address & 0xFF) as u8;
        address_bytes[1] = ((address >> 8) & 0xFF) as u8;
        self.write_bytes(OTP_IF, OTP_ADDR_SUB as u16, &address_bytes)?;
        self.write_bytes(OTP_IF, OTP_CTRL_SUB as u16, &[0x03])?;
        self.write_bytes(OTP_IF, OTP_CTRL_SUB as u16, &[0x01])?;
        self.read_bytes(OTP_IF, OTP_RDAT_SUB as u16, data)?;
        self.write_bytes(OTP_IF, OTP_CTRL_SUB as u16, &[0x00])
    }

    fn manage_lde(&mut self) -> Result<(), Dw1000Error<SPI::Error>> {
        let mut ldo_tune = [0u8; LEN_OTP_RDAT];
        self.read_bytes_otp(0x04, &mut ldo_tune)?;
        if ldo_tune[0] != 0 {
            // TODO: tuning available, copy over to RAM: use OTP_LDO bit
        }
        let mut pmscctrl0 = [0u8; LEN_PMSC_CTRL0];
        let mut otpctrl = [0u8; LEN_OTP_CTRL];
        self.read_bytes(PMSC, PMSC_CTRL0_SUB as u16, &mut pmscctrl0)?;
        self.read_bytes(OTP_IF, OTP_CTRL_SUB as u16, &mut otpctrl)?;
        pmscctrl0[0] = 0x01;
        pmscctrl0[1] = 0x03;
        otpctrl[0] = 0x00;
        otpctrl[1] = 0x80;
        self.write_bytes(PMSC, PMSC_CTRL0_SUB as u16, &pmscctrl0)?;
        self.write_bytes(OTP_IF, OTP_CTRL_SUB as u16, &otpctrl)?;
        self.delay.delay_ms(5);
        pmscctrl0[0] = 0x00;
        pmscctrl0[1] &= 0x02;
        self.write_bytes(PMSC, PMSC_CTRL0_SUB as u16, &pmscctrl0)
    }

    fn write_system_event_mask_register(&mut self) -> Result<(), Dw1000Error<SPI::Error>> {
        let data = self.sysmask.clone();
        self.write_bytes(SYS_MASK, NO_SUB as u16, &data)
    }

    fn clear_interrupts(&mut self) {
        self.sysmask = [0u8; LEN_SYS_MASK];
    }

    fn set_double_buffering(&mut self, value: bool) {
        Self::set_bit(&mut self.syscfg, DIS_DRXB_BIT as u16, !value);
    }

    fn set_interrupt_polarity(&mut self, rising: bool) {
        Self::set_bit(&mut self.syscfg, HIRQ_POL_BIT as u16, rising);
    }

    fn write_system_configuration_register(&mut self) -> Result<(), Dw1000Error<SPI::Error>> {
        let data = self.syscfg.clone();
        self.write_bytes(SYS_CFG, NO_SUB as u16, &data)
    }

    fn set_bit(data: &mut [u8], bit: u16, value: bool) {
        let byte_index = (bit / 8) as usize;
        let bit_index = (bit % 8) as u8;
        if value {
            data[byte_index] |= 1 << bit_index;
        } else {
            data[byte_index] &= !(1 << bit_index);
        }
    }

    fn enable_clock(&mut self, clock: ClockMode) -> Result<(), Dw1000Error<SPI::Error>> {
        let mut pmscctrl0 = [0u8; LEN_PMSC_CTRL0];
        self.read_bytes(PMSC, PMSC_CTRL0_SUB as u16, &mut pmscctrl0)?;
        match clock {
            ClockMode::Auto => {
                pmscctrl0[0] = ClockMode::Auto.to_u8();
                pmscctrl0[1] &= 0xFE;
            }
            ClockMode::Xti => {
                pmscctrl0[0] &= 0xFC;
                pmscctrl0[0] |= ClockMode::Xti.to_u8();
            }
            ClockMode::Pll => {
                pmscctrl0[0] &= 0xFC;
                pmscctrl0[0] |= ClockMode::Pll.to_u8();
            }
        }
        self.write_bytes(PMSC, PMSC_CTRL0_SUB as u16, &mut pmscctrl0[..2])
    }

    fn read_bytes(&mut self, command: u8, offset: u16, data: &mut [u8]) -> Result<(), Dw1000Error<SPI::Error>> {
        let (header, size) = self.prepare_header(command, offset, false);
        self.spi.transaction(&mut [
            Operation::Write(&header[..size]),
            Operation::Read(data),
        ]).map_err(Dw1000Error::SpiError)
    }

    fn write_bytes(&mut self, command: u8, offset: u16, data: &[u8]) -> Result<(), Dw1000Error<SPI::Error>> {
        let (header, size) = self.prepare_header(command, offset, true);
        self.spi.transaction(&mut [
            Operation::Write(&header[..size]),
            Operation::Write(data),
        ]).map_err(Dw1000Error::SpiError)
    }

    fn prepare_header(&self, command: u8, offset: u16, is_write: bool) -> ([u8; 3], usize) {
        let mut header = [0u8; 3];
        let header_len: usize;
        if offset == NO_SUB as u16 {
            header[0] = if is_write { Self::WRITE | command } else { Self::READ | command };
            header_len = 1;
        } else {
            header[0] = if is_write { Self::WRITE_SUB | command } else { Self::READ_SUB | command };
            if offset < 128 {
                header[1] = offset as u8;
                header_len = 2;
            } else {
                header[1] = Self::RW_SUB_EXT | offset as u8;
                header[2] = (offset >> 7) as u8;
                header_len = 3;
            }
        }
        (header, header_len)
    }
}