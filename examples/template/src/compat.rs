use core::convert::Infallible;
use core::fmt::Debug;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{ErrorType as DigitalErrorType, InputPin, OutputPin};
use embedded_hal::spi::{Error as SpiErrorTrait, ErrorKind, ErrorType, SpiBus};
use stm32l4xx_hal::hal::blocking::delay::{DelayMs, DelayUs};
use stm32l4xx_hal::hal::blocking::spi::{Transfer, Write};
use stm32l4xx_hal::hal::digital::v2::{InputPin as OldInputPin, OutputPin as OldOutputPin};

/// Wraps a legacy embedded-hal 0.2 delay provider as an embedded-hal 1.0 delay.
pub struct DelayCompat<D>(D);

impl<D> DelayCompat<D> {
    /// Creates a new delay wrapper.
    pub const fn new(delay: D) -> Self {
        Self(delay)
    }
}

impl<D> DelayNs for DelayCompat<D>
where
    D: DelayMs<u32> + DelayUs<u32>,
{
    fn delay_ns(&mut self, ns: u32) {
        let micros = ns.saturating_add(999) / 1000;
        if micros != 0 {
            self.0.delay_us(micros);
        }
    }

    fn delay_us(&mut self, us: u32) {
        self.0.delay_us(us);
    }

    fn delay_ms(&mut self, ms: u32) {
        self.0.delay_ms(ms);
    }
}

/// Wraps an embedded-hal 0.2 input pin as an embedded-hal 1.0 input pin.
pub struct InputPinCompat<P>(P);

impl<P> InputPinCompat<P> {
    /// Creates a new input-pin wrapper.
    pub const fn new(pin: P) -> Self {
        Self(pin)
    }
}

impl<P> DigitalErrorType for InputPinCompat<P> {
    type Error = Infallible;
}

impl<P> InputPin for InputPinCompat<P>
where
    P: OldInputPin<Error = Infallible>,
{
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.0.is_high()
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.0.is_low()
    }
}

/// Wraps an embedded-hal 0.2 output pin as an embedded-hal 1.0 output pin.
pub struct OutputPinCompat<P>(P);

impl<P> OutputPinCompat<P> {
    /// Creates a new output-pin wrapper.
    pub const fn new(pin: P) -> Self {
        Self(pin)
    }
}

impl<P> DigitalErrorType for OutputPinCompat<P> {
    type Error = Infallible;
}

impl<P> OutputPin for OutputPinCompat<P>
where
    P: OldOutputPin<Error = Infallible>,
{
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_low()
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_high()
    }
}

/// Error wrapper used by the SPI bus compatibility layer.
#[derive(Debug)]
pub enum SpiCompatError<E> {
    /// Error returned by the underlying HAL SPI implementation.
    Bus(E),
    /// Unsupported transfer shape requested by the caller.
    UnsupportedTransfer,
}

impl<E> SpiErrorTrait for SpiCompatError<E>
where
    E: Debug,
{
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

/// Wraps an embedded-hal 0.2 blocking SPI peripheral as an embedded-hal 1.0 SPI bus.
pub struct SpiBusCompat<B>(B);

impl<B> SpiBusCompat<B> {
    /// Creates a new SPI bus wrapper.
    pub const fn new(bus: B) -> Self {
        Self(bus)
    }
}

impl<B, E> ErrorType for SpiBusCompat<B>
where
    E: Debug,
    B: Transfer<u8, Error = E> + Write<u8, Error = E>,
{
    type Error = SpiCompatError<E>;
}

impl<B, E> SpiBus<u8> for SpiBusCompat<B>
where
    E: Debug,
    B: Transfer<u8, Error = E> + Write<u8, Error = E>,
{
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        words.fill(0);
        self.0
            .transfer(words)
            .map(|_| ())
            .map_err(SpiCompatError::Bus)
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.0.write(words).map_err(SpiCompatError::Bus)
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        if read.len() < write.len() {
            return Err(SpiCompatError::UnsupportedTransfer);
        }
        read[..write.len()].copy_from_slice(write);
        read[write.len()..].fill(0);
        self.0
            .transfer(read)
            .map(|_| ())
            .map_err(SpiCompatError::Bus)
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.0
            .transfer(words)
            .map(|_| ())
            .map_err(SpiCompatError::Bus)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
