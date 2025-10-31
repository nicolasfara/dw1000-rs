//! DW1000 Ranging Anchor
//!
//! This example demonstrates how to use the DW1000 as an anchor device for ranging.
//! An anchor receives ranging requests from tags and responds with distance measurements.
//!
//! This is the Rust equivalent of the Arduino DW1000Ranging_ANCHOR example.
//!
//! # Hardware Setup
//! - Connect DW1000 module to SPI bus
//! - Connect RST pin to GPIO (default: pin 9)
//! - Connect IRQ pin to an interrupt-capable GPIO (default: pin 2)
//! - Connect CS pin to SPI SS pin
//!
//! # Expected Behavior
//! - The anchor will wait for blink messages from tags
//! - When a tag is detected, it will perform ranging measurements
//! - Distance, RX power, and other metrics will be reported via callbacks

#![no_std]
#![no_main]

use {defmt_rtt as _, panic_halt as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Input, Level, Output, Pull, Speed},
    spi::{Config as SpiConfig, Spi, Mode as SpiMode, Phase, Polarity},
    time::Hertz,
};
use embassy_time::{Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

use dw1000_rs::{DW1000, DW1000Ranging, RangingEvent};

// connection pins
const PIN_RST: u8 = 9;
const PIN_IRQ: u8 = 2;

// Anchor configuration
const ANCHOR_ADDRESS: &str = "82:17:5B:D5:A9:9A:E2:9C";

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    defmt::info!("DW1000 Ranging Anchor");
    Timer::after(Duration::from_millis(1000)).await;

    // Configure SPI
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(2_000_000);
    spi_config.mode = SpiMode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };

    let spi = Spi::new(
        p.SPI1,
        p.PA5, // SCK
        p.PA7, // MOSI
        p.PA6, // MISO
        p.DMA1_CH3, // TX DMA
        p.DMA1_CH2, // RX DMA
        spi_config,
    );

    let cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    let spi_device = ExclusiveDevice::new(spi, cs, embassy_time::Delay).ok().unwrap();

    let mut rst = Output::new(p.PA9, Level::High, Speed::VeryHigh);
    let _irq = ExtiInput::new(Input::new(p.PA2, Pull::Down), p.EXTI2);

    // Initialize communication (Reset, CS, IRQ)
    rst.set_low();
    Timer::after(Duration::from_millis(10)).await;
    rst.set_high();
    Timer::after(Duration::from_millis(100)).await;

    // Note: ExclusiveDevice already manages CS, so we pass a dummy Output pin
    let dummy_cs = Output::new(p.PA3, Level::High, Speed::VeryHigh);
    let mut dw1000 = DW1000::new(spi_device, dummy_cs);
    let mut ranging = DW1000Ranging::new();

    // Initialize DW1000 communication
    ranging.init_communication(&mut dw1000);

    // Attach callbacks
    // Note: In Rust we handle these inline in the loop rather than function pointers

    // Enable the filter to smooth the distance (optional)
    // ranging.use_range_filter(true);

    // Start as anchor
    ranging.start_as_anchor(&mut dw1000, ANCHOR_ADDRESS);

    defmt::info!("Anchor started, waiting for tags...");

    // Main loop
    loop {
        ranging.loop_step(&mut dw1000, |event| {
            match event {
                RangingEvent::NewRange(device) => {
                    defmt::info!(
                        "from: {:04X} Range: {} m RX power: {} dBm",
                        device.get_short_address(),
                        device.get_range(),
                        device.get_rx_power()
                    );
                }
                RangingEvent::BlinkDevice(device) => {
                    defmt::info!(
                        "blink; 1 device added ! -> short: {:04X}",
                        device.get_short_address()
                    );
                }
                RangingEvent::InactiveDevice(device) => {
                    defmt::info!(
                        "delete inactive device: {:04X}",
                        device.get_short_address()
                    );
                }
                _ => {}
            }
        });

        Timer::after(Duration::from_micros(100)).await;
    }
}
