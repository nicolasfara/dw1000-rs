//! DW1000 Ranging Tag Example
//!
//! This example demonstrates how to use the DW1000 as a tag device for ranging.
//! A tag initiates ranging requests to anchors and receives distance measurements.
//!
//! This is the Rust equivalent of the Arduino DW1000Ranging_TAG example.
//!
//! # Hardware Setup
//! - Connect DW1000 module to SPI bus
//! - Connect RST pin to GPIO (default: pin 9)
//! - Connect IRQ pin to an interrupt-capable GPIO (default: pin 2)
//! - Connect CS pin to SPI SS pin
//!
//! # Expected Behavior
//! - The tag will send blink messages to discover anchors
//! - Once anchors respond, it will initiate ranging sequences
//! - Distance measurements from anchors will be displayed

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Input, Level, Output, Pull, Speed},
    spi::{Config as SpiConfig, Mode as SpiMode, Phase, Polarity, Spi},
    time::Hertz,
};
use embassy_time::{Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use {defmt_rtt as _, panic_halt as _};

use dw1000_rs::{DW1000Ranging, RangingEvent, DW1000};

// connection pins
const PIN_RST: u8 = 9;
const PIN_IRQ: u8 = 2;

// Tag configuration
const TAG_ADDRESS: &str = "7D:00:22:EA:82:60:3B:9C";

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    defmt::info!("DW1000 Ranging Tag");
    Timer::after(Duration::from_millis(1000)).await;

    // Configure SPI
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(2_000_000);
    spi_config.mode = SpiMode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };

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

    // Start as tag
    ranging.start_as_tag(&mut dw1000, TAG_ADDRESS);

    defmt::info!("Tag started, searching for anchors...");

    // Main loop
    loop {
        ranging.loop_step(&mut dw1000, |event| match event {
            RangingEvent::NewRange(device) => {
                defmt::info!(
                    "from: {:04X} Range: {} m RX power: {} dBm",
                    device.get_short_address(),
                    device.get_range(),
                    device.get_rx_power()
                );
            }
            RangingEvent::NewDevice(device) => {
                defmt::info!(
                    "ranging init; 1 device added ! -> short: {:04X}",
                    device.get_short_address()
                );
            }
            RangingEvent::InactiveDevice(device) => {
                defmt::info!("delete inactive device: {:04X}", device.get_short_address());
            }
            _ => {}
        });

        Timer::after(Duration::from_micros(100)).await;
    }
}
