use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Dw1000Error<SPI> {
    SpiError(SPI),
}

pub struct Dw1000<SPI, IRQ, RST> {
    spi: SPI,
    irq: IRQ,
    rst: RST,
}

impl<SPI, IRQ, RST> Dw1000<SPI, IRQ, RST>
where
    SPI: SpiDevice,
    IRQ: InputPin,
    RST: OutputPin,
{
    pub fn new(spi: SPI, irq: IRQ, rst: RST) -> Self {
        Dw1000 { spi, irq, rst }
    }
}