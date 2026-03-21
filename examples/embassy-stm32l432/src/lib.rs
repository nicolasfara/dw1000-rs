#![no_std]

use defmt::{info, warn};
use dw1000_rs::registers::status;
use dw1000_rs::{
    AntennaDelay, AsyncDw1000, DeviceIdentity, Error, OperatingMode, RadioConfig, RangingConfig,
    RangingEvent, RangingNode, Role, SysStatus,
};
use embassy_futures::select::{select, Either};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Flex, Level, Output, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, Peripherals};
use embassy_time::{with_timeout, Delay, Duration, Instant, Ticker, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

const PEER_CAPACITY: usize = 4;
const RX_BUFFER_LEN: usize = 127;
const DW1000_SPI_BAUD_HZ: u32 = 3_000_000;
const STM6600_BOOT_TIMEOUT_MS: u64 = 1_000;
const DW1000_LINK_RECOVERY_TIMEOUT_MS: u32 = 1_000;
const RANGING_REPLY_DELAY_US: u16 = 12_000;
const PEER_INACTIVITY_TIMEOUT_MS: u32 = 3_000;
const FAULT_LED_ON_MS: u64 = 120;
const FAULT_LED_OFF_MS: u64 = 120;
const FAULT_LED_PAUSE_MS: u64 = 700;
const FAULT_BLINK_COUNT: usize = 3;

pub async fn run(role: Role, identity: DeviceIdentity, antenna_delay: AntennaDelay) -> ! {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.hsi = true;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL10,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV8),
        });
        config.rcc.mux.clk48sel = mux::Clk48sel::HSI48;
    }

    let p: Peripherals = embassy_stm32::init(config);

    let mut charger_enable = Output::new(p.PA10, Level::Low, Speed::Low);
    let mut charger_en1 = Output::new(p.PB0, Level::Low, Speed::Low);
    let mut charger_en2 = Output::new(p.PB1, Level::Low, Speed::Low);
    let mut power_int = ExtiInput::new(p.PB7, p.EXTI7, Pull::Up);
    let mut ps_hold = Output::new(p.PB5, Level::Low, Speed::Low);
    let mut orange_led = Output::new(p.PA8, Level::Low, Speed::Low);
    let mut green_led = Output::new(p.PA9, Level::Low, Speed::Low);

    if let Err(message) = bootstrap_board(
        &mut power_int,
        &mut ps_hold,
        &mut charger_enable,
        &mut charger_en1,
        &mut charger_en2,
    )
    .await
    {
        fault_loop(&mut orange_led, &mut green_led, message).await;
    }

    green_led.set_high();
    info!(
        "{=str} starting pan={=u16} short={=u16} antenna_delay={=u16}",
        role_label(role),
        identity.pan_id.raw(),
        identity.short_address.raw(),
        antenna_delay.raw()
    );

    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(DW1000_SPI_BAUD_HZ);
    let spi: Spi<'static, Async> = Spi::new(
        p.SPI1,
        p.PA5,
        p.PA7,
        p.PA6,
        p.DMA1_CH3,
        p.DMA1_CH2,
        spi_config,
    );
    let cs = Output::new(p.PA4, Level::High, Speed::High);
    let irq = ExtiInput::new(p.PA2, p.EXTI2, Pull::None);
    let mut reset = Flex::new(p.PA1);
    reset.set_high();
    reset.set_as_input_output(Speed::Low);

    let spi_device = ExclusiveDevice::new(spi, cs, Delay).expect("spi device");
    let mut radio = AsyncDw1000::new(spi_device, irq, reset);
    let radio_config = default_radio_config(identity, antenna_delay);
    let ranging_config = default_ranging_config(identity);
    let mut node = RangingNode::<PEER_CAPACITY>::new(role, ranging_config);
    let mut rx_buffer = [0u8; RX_BUFFER_LEN];

    if radio.init(&mut Delay, &radio_config).await.is_err() {
        fault_loop(&mut orange_led, &mut green_led, "dw1000 init failed").await;
    }
    if node.start_async(&mut radio, now_ms()).await.is_err() {
        fault_loop(&mut orange_led, &mut green_led, "ranging start failed").await;
    }

    let mut ticker = Ticker::every(Duration::from_millis(
        ranging_config.timer_period_ms as u64,
    ));
    let mut last_radio_activity_ms = now_ms();
    let mut last_range_update_ms = now_ms();

    'app: loop {
        let now = now_ms();
        match select(ticker.next(), radio.wait_for_irq()).await {
            Either::First(_) => {
                let tx_before = node.tx_debug_snapshot();
                match node.tick_async(&mut radio, now).await {
                    Ok(event) => {
                        if node.tx_debug_snapshot() != tx_before {
                            last_radio_activity_ms = now;
                        }
                        if event.is_some() {
                            last_radio_activity_ms = now;
                        }
                        if handle_event(role, event) {
                            last_range_update_ms = now;
                        }
                    }
                    Err(_) => {
                        recover_or_fault(
                            &mut radio,
                            &mut node,
                            &radio_config,
                            &mut orange_led,
                            &mut green_led,
                            now,
                            "tick failed",
                        )
                        .await;
                        last_radio_activity_ms = now;
                        last_range_update_ms = now;
                    }
                }
            }
            Either::Second(irq_result) => {
                if irq_result.is_err() {
                    recover_or_fault(
                        &mut radio,
                        &mut node,
                        &radio_config,
                        &mut orange_led,
                        &mut green_led,
                        now,
                        "irq wait failed",
                    )
                    .await;
                    continue;
                }

                loop {
                    let now = now_ms();
                    let irq_status = match radio.read_sys_status().await {
                        Ok(status) => status,
                        Err(_) => {
                            recover_or_fault(
                                &mut radio,
                                &mut node,
                                &radio_config,
                                &mut orange_led,
                                &mut green_led,
                                now,
                                "status read failed",
                            )
                            .await;
                            continue 'app;
                        }
                    };

                    if irq_status.contains(status::TX_FRAME_SENT) {
                        last_radio_activity_ms = now;
                        match node.on_tx_done_async(&mut radio).await {
                            Ok(event) => {
                                if handle_event(role, event) {
                                    last_range_update_ms = now;
                                }
                            }
                            Err(_) => {
                                recover_or_fault(
                                    &mut radio,
                                    &mut node,
                                    &radio_config,
                                    &mut orange_led,
                                    &mut green_led,
                                    now,
                                    "tx handling failed",
                                )
                                .await;
                                continue 'app;
                            }
                        }
                    }

                    if has_rx_work(irq_status) {
                        match node.on_rx_async(&mut radio, now, &mut rx_buffer).await {
                            Ok(event) => {
                                last_radio_activity_ms = now;
                                if handle_event(role, event) {
                                    last_range_update_ms = now;
                                }
                            }
                            Err(Error::Receive(_)) => {
                                last_radio_activity_ms = now;
                                warn!("rx error");
                            }
                            Err(Error::Protocol(_)) => {
                                last_radio_activity_ms = now;
                                warn!("protocol error");
                            }
                            Err(_) => {
                                recover_or_fault(
                                    &mut radio,
                                    &mut node,
                                    &radio_config,
                                    &mut orange_led,
                                    &mut green_led,
                                    now,
                                    "rx handling failed",
                                )
                                .await;
                                continue 'app;
                            }
                        }
                    }

                    if irq_status != SysStatus::EMPTY && radio.clear_events(irq_status).await.is_err() {
                        recover_or_fault(
                            &mut radio,
                            &mut node,
                            &radio_config,
                            &mut orange_led,
                            &mut green_led,
                            now,
                            "clear events failed",
                        )
                        .await;
                        continue 'app;
                    }

                    match radio.irq_asserted() {
                        Ok(true) => {}
                        Ok(false) => break,
                        Err(_) => {
                            recover_or_fault(
                                &mut radio,
                                &mut node,
                                &radio_config,
                                &mut orange_led,
                                &mut green_led,
                                now,
                                "irq state read failed",
                            )
                            .await;
                            continue 'app;
                        }
                    }
                }
            }
        }

        let now = now_ms();
        let radio_gap_ms = now.wrapping_sub(last_radio_activity_ms);
        let range_gap_ms = now.wrapping_sub(last_range_update_ms);
        if node.peers().next().is_some()
            && (radio_gap_ms >= DW1000_LINK_RECOVERY_TIMEOUT_MS
                || range_gap_ms >= DW1000_LINK_RECOVERY_TIMEOUT_MS)
        {
            warn!(
                "recovering link radio_gap={=u32} range_gap={=u32}",
                radio_gap_ms,
                range_gap_ms
            );
            recover_or_fault(
                &mut radio,
                &mut node,
                &radio_config,
                &mut orange_led,
                &mut green_led,
                now,
                "stalled link",
            )
            .await;
            last_radio_activity_ms = now;
            last_range_update_ms = now;
        }
    }
}

fn default_radio_config(identity: DeviceIdentity, antenna_delay: AntennaDelay) -> RadioConfig {
    let mut config = RadioConfig::from_mode(identity, OperatingMode::LongDataRangeAccuracy);
    config.antenna_delay = antenna_delay;
    config
}

fn default_ranging_config(identity: DeviceIdentity) -> RangingConfig {
    let mut config = RangingConfig::new(identity);
    config.reply_delay_us = RANGING_REPLY_DELAY_US;
    config.reset_period_ms = PEER_INACTIVITY_TIMEOUT_MS;
    config
}

fn now_ms() -> u32 {
    Instant::now().as_millis() as u32
}

fn role_label(role: Role) -> &'static str {
    match role {
        Role::Tag => "tag",
        Role::Anchor => "anchor",
    }
}

fn has_rx_work(irq_status: SysStatus) -> bool {
    irq_status.contains(status::RX_FRAME_READY)
        || irq_status.contains(status::RX_FRAME_GOOD)
        || irq_status.contains(status::RX_FRAME_CHECK_ERROR)
        || irq_status.contains(status::RX_REED_SOLOMON_ERROR)
        || irq_status.contains(status::RX_TIMEOUT)
        || irq_status.contains(status::RX_HEADER_ERROR)
        || irq_status.contains(status::LDE_ERROR)
}

fn handle_event(role: Role, event: Option<RangingEvent>) -> bool {
    match event {
        Some(RangingEvent::BlinkReceived(snapshot)) => {
            info!(
                "{=str} blink from {=u16}",
                role_label(role),
                snapshot.short_address.raw()
            );
            false
        }
        Some(RangingEvent::NewPeer(snapshot)) => {
            info!(
                "{=str} peer {=u16}",
                role_label(role),
                snapshot.short_address.raw()
            );
            false
        }
        Some(RangingEvent::PeerInactive(short)) => {
            warn!("{=str} peer inactive {=u16}", role_label(role), short.raw());
            false
        }
        Some(RangingEvent::RangeUpdated(snapshot)) => {
            if role == Role::Tag {
                info!(
                    "tag distance anchor={=u16} range_m={=f32}",
                    snapshot.short_address.raw(),
                    snapshot.range_m
                );
            }
            true
        }
        Some(RangingEvent::RangingInitReceived(short)) => {
            info!("tag ranging init from {=u16}", short.raw());
            false
        }
        None => false,
    }
}

async fn bootstrap_board(
    power_int: &mut ExtiInput<'static>,
    ps_hold: &mut Output<'static>,
    charger_enable: &mut Output<'static>,
    charger_en1: &mut Output<'static>,
    charger_en2: &mut Output<'static>,
) -> Result<(), &'static str> {
    charger_enable.set_low();
    charger_en1.set_high();
    charger_en2.set_low();

    if with_timeout(
        Duration::from_millis(STM6600_BOOT_TIMEOUT_MS),
        power_int.wait_for_high(),
    )
    .await
    .is_err()
    {
        return Err("stm6600 bootstrap timed out");
    }

    ps_hold.set_high();
    Timer::after(Duration::from_millis(10)).await;
    Ok(())
}

async fn recover_or_fault<SPI, IRQ, RST, const N: usize>(
    radio: &mut AsyncDw1000<SPI, IRQ, RST>,
    node: &mut RangingNode<N>,
    radio_config: &RadioConfig,
    orange_led: &mut Output<'static>,
    green_led: &mut Output<'static>,
    now_ms: u32,
    reason: &'static str,
) where
    SPI: embedded_hal_async::spi::SpiDevice,
    IRQ: embedded_hal::digital::InputPin + embedded_hal_async::digital::Wait,
    RST: embedded_hal::digital::OutputPin<Error = <IRQ as embedded_hal::digital::ErrorType>::Error>,
{
    orange_led.set_high();
    warn!("recovering radio: {=str}", reason);
    if radio.init(&mut Delay, radio_config).await.is_err() {
        fault_loop(orange_led, green_led, "radio reinit failed").await;
    }
    if node.recover_link_async(radio, now_ms).await.is_err() {
        fault_loop(orange_led, green_led, "link recovery failed").await;
    }
    orange_led.set_low();
}

async fn fault_loop(
    orange_led: &mut Output<'static>,
    green_led: &mut Output<'static>,
    message: &'static str,
) -> ! {
    green_led.set_low();
    orange_led.set_low();
    warn!("{=str}", message);
    loop {
        for _ in 0..FAULT_BLINK_COUNT {
            orange_led.set_high();
            Timer::after(Duration::from_millis(FAULT_LED_ON_MS)).await;
            orange_led.set_low();
            Timer::after(Duration::from_millis(FAULT_LED_OFF_MS)).await;
        }
        Timer::after(Duration::from_millis(FAULT_LED_PAUSE_MS)).await;
    }
}
