use defmt::info;
use embedded_hal::delay::DelayNs;

/// Shared platform hooks used by the STM32 reference example.
pub trait Platform {
    /// Returns the current monotonic millisecond counter.
    fn now_ms(&self) -> u32;

    /// Delays for the requested number of milliseconds.
    fn delay_ms(&mut self, millis: u32);

    /// Logs a short diagnostic message.
    fn log(&mut self, message: &str);
}

/// Minimal STM32 platform state for the reference tag/anchor binaries.
pub struct Stm32Platform<DELAY> {
    delay: DELAY,
    now_ms: u32,
}

impl<DELAY> Stm32Platform<DELAY> {
    /// Creates a new platform wrapper around a HAL delay provider.
    pub const fn new(delay: DELAY) -> Self {
        Self { delay, now_ms: 0 }
    }

    /// Returns the underlying delay implementation for driver setup.
    pub fn delay(&mut self) -> &mut DELAY {
        &mut self.delay
    }
}

impl<DELAY> Platform for Stm32Platform<DELAY>
where
    DELAY: DelayNs,
{
    fn now_ms(&self) -> u32 {
        self.now_ms
    }

    fn delay_ms(&mut self, millis: u32) {
        self.delay.delay_ms(millis);
        self.now_ms = self.now_ms.wrapping_add(millis);
    }

    fn log(&mut self, message: &str) {
        info!("{=str}", message);
    }
}
