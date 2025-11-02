use core::ptr::write_bytes;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::{Operation, SpiDevice};
use crate::{ClockMode, DeviceMode};
use crate::constants::{DIS_DRXB_BIT, HIRQ_POL_BIT, LEN_OTP_ADDR, LEN_OTP_CTRL, LEN_OTP_RDAT, LEN_PANADR, LEN_PMSC_CTRL0, LEN_SYS_CFG, LEN_SYS_CTRL, LEN_SYS_MASK, NO_SUB, OTP_ADDR_SUB, OTP_CTRL_SUB, OTP_IF, OTP_RDAT_SUB, PANADR, PMSC, PMSC_CTRL0_SUB, SYS_CFG, SYS_CTRL, SYS_MASK, TRXOFF_BIT};

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
    network_and_address: [u8; LEN_PANADR],
    device_mode: DeviceMode,
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
            network_and_address: [0u8; LEN_PANADR],
            device_mode: DeviceMode::Idle,
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