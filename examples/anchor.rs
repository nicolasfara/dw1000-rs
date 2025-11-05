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

use cortex_m::asm::delay;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embassy_executor::Spawner;
use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Input, Level, Output, Pull, Speed},
    spi::{Config as SpiConfig, Mode as SpiMode, Phase, Polarity, Spi},
    time::Hertz,
};
use embassy_stm32::spi::{BitOrder, MODE_0};
use embassy_time::{Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use {defmt_rtt as _, panic_halt as _};

use dw1000_rs::dw1000::Dw1000;
use dw1000_rs::ranging::{DeviceType, Dw1000Ranging, RangingEvent};

// Anchor configuration
const ANCHOR_ADDRESS: &str = "82:17:5B:D5:A9:9A:E2:9C";

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    defmt::info!("DW1000 Ranging Anchor");
    Timer::after(Duration::from_millis(1000)).await;

    // Configure SPI
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(5_000_000);
    spi_config.mode = MODE_0;
    spi_config.bit_order = BitOrder::MsbFirst;

    let spi = Spi::new(
        p.SPI1, p.PA5,      // SCK
        p.PA7,      // MOSI
        p.PA6,      // MISO
        p.DMA1_CH3, // TX DMA
        p.DMA1_CH2, // RX DMA
        spi_config,
    );

    let cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    let spi_device = ExclusiveDevice::new(spi, cs, embassy_time::Delay)
        .ok()
        .unwrap();

    let mut rst = Output::new(p.PB12, Level::High, Speed::VeryHigh);
    let _irq = ExtiInput::new(Input::new(p.PA2, Pull::Down), p.EXTI2);

    // Initialize communication (Reset, CS, IRQ)
    rst.set_low();
    Timer::after(Duration::from_millis(10)).await;
    rst.set_high();
    Timer::after(Duration::from_millis(100)).await;

    // Note: ExclusiveDevice already manages CS, so we pass a dummy Output pin
    let mut dw1000 = Dw1000::new(spi_device, _irq, rst, embassy_time::Delay);
    let mut ranging = Dw1000Ranging::new(&mut dw1000, DeviceType::Anchor);
    ranging.init_communication().expect("DW1000 init failed");
    ranging.configure_network(0x1234, 0xDECA, &[0x00, 0x01, 0x0C]).expect("FD");

    defmt::info!("DW1000 initialized for Anchor at address {}", ANCHOR_ADDRESS);

    loop {
        match ranging.round().expect("ranging failed") {
            RangingEvent::None => { }
            RangingEvent::BlinkReceived { .. } => {
                defmt::info!("Blink message received from tag");
            }
            RangingEvent::NewDevice { .. } => {
                defmt::info!("New device discovered");
            }
            RangingEvent::InactiveDevice { .. } => {
                defmt::info!("Device became inactive");
            }
            RangingEvent::NewRange { .. } => {
                defmt::info!("New range measurement received");
            }
            RangingEvent::RangingInitReceived { .. } => {
                defmt::info!("Ranging init message received");
            }
            RangingEvent::DeviceNotFound { .. } => {
                defmt::warn!("Device not found for ranging");
            }
            RangingEvent::UnexpectedMessage => {
                defmt::warn!("DW1000 unexpected message");
            }
            RangingEvent::ProtocolFailed => {
                defmt::warn!("DW1000 protocol failed");
            }
        }
    }
}
