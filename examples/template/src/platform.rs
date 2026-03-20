use core::cell::Cell;
use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::peripheral::DWT;
use defmt::info;
use embedded_hal::delay::DelayNs;

const MIN_LOG_INTERVAL_MS: u32 = 250;

static LOG_TIMESTAMP_MS: AtomicU32 = AtomicU32::new(0);

/// Shared `defmt` timestamp source updated by the STM32 platform clock.
pub fn defmt_timestamp_ms() -> u32 {
    LOG_TIMESTAMP_MS.load(Ordering::Relaxed)
}

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
    cycles_per_ms: u32,
    last_cycle: Cell<u32>,
    elapsed_cycles: Cell<u64>,
    last_log_ms: u32,
}

impl<DELAY> Stm32Platform<DELAY> {
    /// Creates a new platform wrapper around a HAL delay provider.
    pub fn new(delay: DELAY, core_hz: u32) -> Self {
        let now_cycle = DWT::cycle_count();
        Self {
            delay,
            cycles_per_ms: core_hz / 1_000,
            last_cycle: Cell::new(now_cycle),
            elapsed_cycles: Cell::new(0),
            last_log_ms: 0,
        }
    }

    /// Returns the underlying delay implementation for driver setup.
    pub fn delay(&mut self) -> &mut DELAY {
        &mut self.delay
    }

    fn refresh_now_ms(&self) -> u32 {
        let now_cycle = DWT::cycle_count();
        let last_cycle = self.last_cycle.replace(now_cycle);
        let elapsed_cycles = self
            .elapsed_cycles
            .get()
            .wrapping_add(now_cycle.wrapping_sub(last_cycle) as u64);
        self.elapsed_cycles.set(elapsed_cycles);
        let now_ms = (elapsed_cycles / self.cycles_per_ms.max(1) as u64) as u32;
        LOG_TIMESTAMP_MS.store(now_ms, Ordering::Relaxed);
        now_ms
    }
}

impl<DELAY> Platform for Stm32Platform<DELAY>
where
    DELAY: DelayNs,
{
    fn now_ms(&self) -> u32 {
        self.refresh_now_ms()
    }

    fn delay_ms(&mut self, millis: u32) {
        self.delay.delay_ms(millis);
        let _ = self.refresh_now_ms();
    }

    fn log(&mut self, message: &str) {
        let now_ms = self.refresh_now_ms();
        if now_ms.wrapping_sub(self.last_log_ms) < MIN_LOG_INTERVAL_MS {
            return;
        }
        self.last_log_ms = now_ms;
        info!("{=str}", message);
    }
}
