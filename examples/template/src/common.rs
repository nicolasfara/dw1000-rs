use cortex_m::Peripherals as CorePeripherals;
use defmt::info;
use dw1000_rs::{
    protocol::FrameKind,
    registers::status, AntennaDelay, DeviceIdentity, Dw1000, Error, OperatingMode, RadioConfig,
    RangingConfig, RangingEvent, RangingNode, Role,
};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use stm32l4xx_hal as hal;
use stm32l4xx_hal::hal::digital::v2::{InputPin as OldInputPin, OutputPin as OldOutputPin};
use stm32l4xx_hal::prelude::*;

use crate::compat::{DelayCompat, InputPinCompat, OutputPinCompat, SpiBusCompat};
use crate::platform::{Platform, Stm32Platform};

const PEER_CAPACITY: usize = 4;
const MAIN_LOOP_DELAY_MS: u32 = 0;
const RX_BUFFER_LEN: usize = 127;
const DW1000_SPI_BAUD_HZ: u32 = 3_000_000;
const STM6600_BOOT_TIMEOUT_MS: u32 = 1_000;
const DW1000_RESET_PULSE_MS: u32 = 100;
const DW1000_IDLE_LOG_PERIOD_MS: u32 = 1_000;
const LINK_RECOVERY_TIMEOUT_MS: u32 = 1_000;
const TELEMETRY_LOG_PERIOD_MS: u32 = 1_000;
const RUNTIME_FAULT_LOG_PERIOD_MS: u32 = 1_000;
const PEER_INACTIVITY_TIMEOUT_MS: u32 = 3_000;
const RANGING_REPLY_DELAY_US: u16 = 12_000;
const STARTUP_LED_PULSE_MS: u32 = 100;
const DISCOVERY_LED_PULSE_MS: u32 = 100;
const RANGE_UPDATE_LED_PULSE_MS: u32 = 50;
const DW1000_SPI_MODE: hal::hal::spi::Mode = hal::hal::spi::Mode {
    polarity: hal::hal::spi::Polarity::IdleLow,
    phase: hal::hal::spi::Phase::CaptureOnFirstTransition,
};

/// Default example antenna delay calibration.
pub const DEFAULT_ANTENNA_DELAY: AntennaDelay = AntennaDelay::new(16_456);

/// Shared application state for the tag/anchor binaries.
pub struct App<RADIO, const N: usize> {
    pub radio: RADIO,
    pub node: RangingNode<N>,
}

struct RadioIrqOutcome {
    had_radio_activity: bool,
    range_updated: bool,
    led_pulse_ms: Option<u32>,
}

#[derive(Clone, Copy)]
struct LedPulse {
    started_ms: u32,
    duration_ms: u32,
}

#[derive(Default)]
struct TelemetryCounters {
    irq_seen: u32,
    rx_error: u32,
    protocol_error: u32,
    blink_tx: u32,
    poll_tx: u32,
    ranging_init_rx: u32,
    range_updated: u32,
    recoveries: u32,
}

/// Constructs the shared app state.
pub fn build_app<RADIO, const N: usize>(
    radio: RADIO,
    role: Role,
    identity: DeviceIdentity,
) -> App<RADIO, N> {
    App {
        radio,
        node: RangingNode::new(role, default_ranging_config(identity)),
    }
}

/// Logs high-level ranging events.
pub fn handle_event<P: Platform>(platform: &mut P, role: Role, event: Option<RangingEvent>) {
    match event {
        Some(RangingEvent::PeerInactive(_)) => platform.log("peer inactive"),
        Some(RangingEvent::RangeUpdated(snapshot)) if role == Role::Tag => {
            info!(
                "tag distance to anchor {=u16}: {=f32} m",
                snapshot.short_address.raw(),
                snapshot.range_m
            );
        }
        Some(RangingEvent::RangeUpdated(_)) => {}
        None => {}
        _ => {}
    }
}

fn is_range_update(event: Option<RangingEvent>) -> bool {
    matches!(event, Some(RangingEvent::RangeUpdated(_)))
}

fn record_event_telemetry(counters: &mut TelemetryCounters, event: Option<RangingEvent>) {
    match event {
        Some(RangingEvent::RangingInitReceived(_)) => {
            counters.ranging_init_rx = counters.ranging_init_rx.wrapping_add(1);
        }
        Some(RangingEvent::RangeUpdated(_)) => {
            counters.range_updated = counters.range_updated.wrapping_add(1);
        }
        _ => {}
    }
}

fn record_tick_transmit(
    counters: &mut TelemetryCounters,
    before: (u8, Option<FrameKind>),
    after: (u8, Option<FrameKind>),
) {
    if before.0 == after.0 {
        return;
    }
    match after.1 {
        Some(FrameKind::Blink) => counters.blink_tx = counters.blink_tx.wrapping_add(1),
        Some(FrameKind::Poll) => counters.poll_tx = counters.poll_tx.wrapping_add(1),
        _ => {}
    }
}

fn log_telemetry(role: Role, now_ms: u32, peers: usize, counters: &mut TelemetryCounters) {
    let role = match role {
        Role::Tag => "tag",
        Role::Anchor => "anchor",
    };
    info!(
        "{=str} stats t={=u32} peers={=u8} irq={=u32} blink_tx={=u32} poll_tx={=u32} init_rx={=u32} range={=u32} rx_err={=u32} proto_err={=u32} recoveries={=u32}",
        role,
        now_ms,
        peers as u8,
        counters.irq_seen,
        counters.blink_tx,
        counters.poll_tx,
        counters.ranging_init_rx,
        counters.range_updated,
        counters.rx_error,
        counters.protocol_error,
        counters.recoveries,
    );
    *counters = TelemetryCounters::default();
}

fn led_pulse_duration_ms(event: Option<RangingEvent>) -> Option<u32> {
    match event {
        Some(RangingEvent::BlinkReceived(_))
        | Some(RangingEvent::NewPeer(_))
        | Some(RangingEvent::RangingInitReceived(_)) => Some(DISCOVERY_LED_PULSE_MS),
        Some(RangingEvent::RangeUpdated(_)) => Some(RANGE_UPDATE_LED_PULSE_MS),
        _ => None,
    }
}

fn merge_led_pulse_ms(current: Option<u32>, event: Option<RangingEvent>) -> Option<u32> {
    match (current, led_pulse_duration_ms(event)) {
        (Some(left), Some(right)) => Some(left.max(right)),
        (Some(left), None) => Some(left),
        (None, pulse) => pulse,
    }
}

fn start_led_pulse<Led>(
    orange_led: &mut Led,
    pulse: &mut Option<LedPulse>,
    now_ms: u32,
    duration_ms: u32,
) where
    Led: OldOutputPin,
{
    let _ = orange_led.set_high();
    *pulse = Some(LedPulse {
        started_ms: now_ms,
        duration_ms,
    });
}

fn update_led_pulse<Led>(
    orange_led: &mut Led,
    pulse: &mut Option<LedPulse>,
    now_ms: u32,
) where
    Led: OldOutputPin,
{
    if let Some(active_pulse) = *pulse {
        if now_ms.wrapping_sub(active_pulse.started_ms) >= active_pulse.duration_ms {
            let _ = orange_led.set_low();
            *pulse = None;
        }
    }
}

fn pulse_led_blocking<Led, Delay>(orange_led: &mut Led, delay: &mut Delay, duration_ms: u32)
where
    Led: OldOutputPin,
    Delay: embedded_hal::delay::DelayNs,
{
    let _ = orange_led.set_high();
    delay.delay_ms(duration_ms);
    let _ = orange_led.set_low();
}

fn maybe_log_fault<P: Platform>(
    platform: &mut P,
    last_fault_log_ms: &mut u32,
    now_ms: u32,
    message: &str,
) {
    if now_ms.wrapping_sub(*last_fault_log_ms) < RUNTIME_FAULT_LOG_PERIOD_MS {
        return;
    }
    *last_fault_log_ms = now_ms;
    platform.log(message);
}

fn recover_runtime_fault<P, SPI, IRQ, RST, Led, const N: usize>(
    platform: &mut P,
    app: &mut App<Dw1000<SPI, IRQ, RST>, N>,
    orange_led: &mut Led,
    orange_led_pulse: &mut Option<LedPulse>,
    last_fault_log_ms: &mut u32,
    now_ms: u32,
    message: &str,
    telemetry: &mut TelemetryCounters,
) where
    P: Platform,
    SPI: SpiDevice,
    IRQ: InputPin,
    RST: OutputPin<Error = IRQ::Error>,
    Led: OldOutputPin,
{
    maybe_log_fault(platform, last_fault_log_ms, now_ms, message);
    telemetry.recoveries = telemetry.recoveries.wrapping_add(1);
    start_led_pulse(
        orange_led,
        orange_led_pulse,
        now_ms,
        DISCOVERY_LED_PULSE_MS,
    );
    if app.node.recover_link(&mut app.radio, now_ms).is_err() {
        maybe_log_fault(platform, last_fault_log_ms, now_ms, "recovery failed");
    }
}

/// Builds a DW1000 config suitable for the STM32L432KB reference setup.
pub fn default_radio_config(
    identity: DeviceIdentity,
    antenna_delay: AntennaDelay,
) -> RadioConfig {
    let mut config = RadioConfig::from_mode(identity, OperatingMode::LongDataRangeAccuracy);
    // The reference STM32L432KB firmware uses separate TX/RX antenna delays.
    // This v1 driver exposes one symmetric value, so the example makes the
    // calibration explicit and board-tunable per node.
    config.antenna_delay = antenna_delay;
    config
}

/// Builds a ranging config tuned for real STM32L432KB example boards.
pub fn default_ranging_config(identity: DeviceIdentity) -> RangingConfig {
    let mut config = RangingConfig::new(identity);
    // The older Embassy firmware leaves about 10 ms between RX and the next TX.
    // Give the blocking template a bit more slack so delayed replies are not
    // missed once SPI work and logging are included.
    config.reply_delay_us = RANGING_REPLY_DELAY_US;
    // The stock 200 ms timeout is easy to hit on real hardware after a few
    // dropped frames. Give the example room to recover before pruning a peer.
    config.reset_period_ms = PEER_INACTIVITY_TIMEOUT_MS;
    config
}

/// Runs the shared STM32L432KB tag/anchor application loop.
pub fn run(role: Role, identity: DeviceIdentity) -> ! {
    run_with_antenna_delay(role, identity, DEFAULT_ANTENNA_DELAY)
}

/// Runs the shared STM32L432KB tag/anchor application loop with an explicit
/// per-board antenna delay calibration.
pub fn run_with_antenna_delay(
    role: Role,
    identity: DeviceIdentity,
    antenna_delay: AntennaDelay,
) -> ! {
    let dp = hal::pac::Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc.cfgr.sysclk(80.MHz()).freeze(&mut flash.acr, &mut pwr);
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
    let delay = DelayCompat::new(hal::delay::Delay::new(cp.SYST, clocks));
    let mut platform = Stm32Platform::new(delay, clocks.hclk().raw());

    // Board-level power management pins copied from the STM6600-based
    // reference firmware. The blocking example keeps the logic simple but
    // preserves the same bootstrap and charger wiring.
    let mut charger_enable = gpioa
        .pa10
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let mut charger_en1 = gpiob
        .pb0
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut charger_en2 = gpiob
        .pb1
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut power_int = gpiob
        .pb7
        .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let mut ps_hold = gpiob
        .pb5
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut orange_led = gpioa
        .pa8
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let mut green_led = gpioa
        .pa9
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    initialize_reference_board(
        role,
        &mut power_int,
        &mut ps_hold,
        &mut charger_enable,
        &mut charger_en1,
        &mut charger_en2,
        &mut orange_led,
        &mut green_led,
        platform.delay(),
    );
    pulse_led_blocking(&mut orange_led, platform.delay(), STARTUP_LED_PULSE_MS);

    let sck = gpioa
        .pa5
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let miso = gpioa
        .pa6
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let mosi = gpioa
        .pa7
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

    let mut cs = gpioa
        .pa4
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let _ = cs.set_high();
    let irq = gpioa
        .pa2
        .into_floating_input(&mut gpioa.moder, &mut gpioa.pupdr);
    let mut reset = gpioa
        .pa1
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let _ = reset.set_low();
    platform.delay_ms(DW1000_RESET_PULSE_MS);
    let _ = reset.set_high();
    platform.delay_ms(10);

    let spi_bus = hal::spi::Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        DW1000_SPI_MODE,
        DW1000_SPI_BAUD_HZ.Hz(),
        clocks,
        &mut rcc.apb2,
    );
    let spi = ExclusiveDevice::new_no_delay(SpiBusCompat::new(spi_bus), OutputPinCompat::new(cs))
        .unwrap();
    let mut radio = Dw1000::new(spi, InputPinCompat::new(irq), OutputPinCompat::new(reset));
    let radio_config = default_radio_config(identity, antenna_delay);
    radio.init(platform.delay(), &radio_config).unwrap();

    let mut app = build_app::<_, PEER_CAPACITY>(radio, role, identity);
    app.node.start(&mut app.radio, platform.now_ms()).unwrap();

    info!("dw1000 antenna delay {=u16} ticks", antenna_delay.raw());
    info!("ranging reply delay {=u16} us", RANGING_REPLY_DELAY_US);
    info!("peer inactivity timeout {=u32} ms", PEER_INACTIVITY_TIMEOUT_MS);
    match role {
        Role::Tag => platform.log("stm32l432kb tag started"),
        Role::Anchor => platform.log("stm32l432kb anchor started"),
    }

    let mut rx_buffer = [0u8; RX_BUFFER_LEN];
    let mut last_radio_activity_ms = platform.now_ms();
    let mut last_range_update_ms = platform.now_ms();
    let mut last_idle_log_ms = platform.now_ms();
    let mut last_telemetry_log_ms = platform.now_ms();
    let mut last_fault_log_ms = platform.now_ms().wrapping_sub(RUNTIME_FAULT_LOG_PERIOD_MS);
    let mut orange_led_pulse = None;
    let mut telemetry = TelemetryCounters::default();
    loop {
        let now_ms = platform.now_ms();
        let tick_tx_before = app.node.tx_debug_snapshot();
        let event = match app.node.tick(&mut app.radio, now_ms) {
            Ok(event) => event,
            Err(_) => {
                recover_runtime_fault(
                    &mut platform,
                    &mut app,
                    &mut orange_led,
                    &mut orange_led_pulse,
                    &mut last_fault_log_ms,
                    now_ms,
                    "tick fault",
                    &mut telemetry,
                );
                update_led_pulse(&mut orange_led, &mut orange_led_pulse, now_ms);
                if MAIN_LOOP_DELAY_MS != 0 {
                    platform.delay_ms(MAIN_LOOP_DELAY_MS);
                }
                continue;
            }
        };
        record_tick_transmit(&mut telemetry, tick_tx_before, app.node.tx_debug_snapshot());
        if event.is_some() {
            last_radio_activity_ms = now_ms;
        }
        if is_range_update(event) {
            last_range_update_ms = now_ms;
        }
        record_event_telemetry(&mut telemetry, event);
        handle_event(&mut platform, role, event);
        if let Some(duration_ms) = led_pulse_duration_ms(event) {
            start_led_pulse(
                &mut orange_led,
                &mut orange_led_pulse,
                now_ms,
                duration_ms,
            );
        }

        let irq_outcome = match service_radio_irq(
            &mut platform,
            &mut app,
            &mut rx_buffer,
            &mut telemetry,
        ) {
            Ok(outcome) => outcome,
            Err(_) => {
                let now_ms = platform.now_ms();
                recover_runtime_fault(
                    &mut platform,
                    &mut app,
                    &mut orange_led,
                    &mut orange_led_pulse,
                    &mut last_fault_log_ms,
                    now_ms,
                    "irq fault",
                    &mut telemetry,
                );
                update_led_pulse(&mut orange_led, &mut orange_led_pulse, now_ms);
                if MAIN_LOOP_DELAY_MS != 0 {
                    platform.delay_ms(MAIN_LOOP_DELAY_MS);
                }
                continue;
            }
        };
        let now_ms = platform.now_ms();
        if irq_outcome.range_updated {
            last_range_update_ms = now_ms;
        }
        if let Some(duration_ms) = irq_outcome.led_pulse_ms {
            start_led_pulse(
                &mut orange_led,
                &mut orange_led_pulse,
                now_ms,
                duration_ms,
            );
        }
        if irq_outcome.had_radio_activity {
            last_radio_activity_ms = now_ms;
        }

        let now_ms = platform.now_ms();
        if app.node.peers().next().is_some()
            && now_ms.wrapping_sub(last_radio_activity_ms) >= LINK_RECOVERY_TIMEOUT_MS
            && now_ms.wrapping_sub(last_range_update_ms) >= LINK_RECOVERY_TIMEOUT_MS
        {
            recover_runtime_fault(
                &mut platform,
                &mut app,
                &mut orange_led,
                &mut orange_led_pulse,
                &mut last_fault_log_ms,
                now_ms,
                "link recovery",
                &mut telemetry,
            );
            last_radio_activity_ms = now_ms;
            last_idle_log_ms = now_ms;
        }
        if now_ms.wrapping_sub(last_telemetry_log_ms) >= TELEMETRY_LOG_PERIOD_MS {
            log_telemetry(role, now_ms, app.node.peers().count(), &mut telemetry);
            last_telemetry_log_ms = now_ms;
        }
        update_led_pulse(&mut orange_led, &mut orange_led_pulse, now_ms);
        if now_ms.wrapping_sub(last_idle_log_ms) >= DW1000_IDLE_LOG_PERIOD_MS {
            last_idle_log_ms = now_ms;
        }
        if MAIN_LOOP_DELAY_MS != 0 {
            platform.delay_ms(MAIN_LOOP_DELAY_MS);
        }
    }
}

fn initialize_reference_board<
    PowerInt,
    PsHold,
    ChargerEnable,
    ChargerEn1,
    ChargerEn2,
    Orange,
    Green,
    Delay,
>(
    _role: Role,
    power_int: &mut PowerInt,
    ps_hold: &mut PsHold,
    charger_enable: &mut ChargerEnable,
    charger_en1: &mut ChargerEn1,
    charger_en2: &mut ChargerEn2,
    orange_led: &mut Orange,
    green_led: &mut Green,
    delay: &mut Delay,
) where
    PowerInt: OldInputPin,
    PsHold: OldOutputPin,
    ChargerEnable: OldOutputPin,
    ChargerEn1: OldOutputPin,
    ChargerEn2: OldOutputPin,
    Orange: OldOutputPin,
    Green: OldOutputPin,
    Delay: embedded_hal::delay::DelayNs,
{
    let _ = orange_led.set_high();
    let _ = green_led.set_low();
    let _ = charger_enable.set_low();

    let _ = charger_en1.set_high();
    let _ = charger_en2.set_low();

    info!("waiting for stm6600 power_int");
    for _ in 0..STM6600_BOOT_TIMEOUT_MS {
        if power_int.is_high().unwrap_or(false) {
            let _ = ps_hold.set_high();
            delay.delay_ms(10);
            let _ = orange_led.set_low();
            let _ = green_led.set_high();
            info!("stm6600 bootstrap complete");
            return;
        }
        delay.delay_ms(1);
    }

    panic!("stm6600 power bootstrap timed out");
}

fn service_radio_irq<P, SPI, IRQ, RST, const N: usize>(
    platform: &mut P,
    app: &mut App<Dw1000<SPI, IRQ, RST>, N>,
    rx_buffer: &mut [u8],
    telemetry: &mut TelemetryCounters,
) -> Result<RadioIrqOutcome, Error<SPI::Error, IRQ::Error>>
where
    P: Platform,
    SPI: SpiDevice,
    IRQ: InputPin,
    RST: OutputPin<Error = IRQ::Error>,
{
    if !app.radio.irq_asserted()? {
        return Ok(RadioIrqOutcome {
            had_radio_activity: false,
            range_updated: false,
            led_pulse_ms: None,
        });
    }

    telemetry.irq_seen = telemetry.irq_seen.wrapping_add(1);
    let irq_status = app.radio.read_sys_status()?;
    let mut range_updated = false;
    let mut led_pulse_ms = None;

    if irq_status.contains(status::TX_FRAME_SENT) {
        let event = app.node.on_tx_done(&mut app.radio)?;
        range_updated |= is_range_update(event);
        record_event_telemetry(telemetry, event);
        led_pulse_ms = merge_led_pulse_ms(led_pulse_ms, event);
        handle_event(platform, app.node.role(), event);
    }

    let has_rx_work = irq_status.contains(status::RX_FRAME_READY)
        || irq_status.contains(status::RX_FRAME_GOOD)
        || irq_status.contains(status::RX_FRAME_CHECK_ERROR)
        || irq_status.contains(status::RX_REED_SOLOMON_ERROR)
        || irq_status.contains(status::RX_TIMEOUT)
        || irq_status.contains(status::RX_HEADER_ERROR)
        || irq_status.contains(status::LDE_ERROR);

    if has_rx_work {
        match app.node.on_rx(&mut app.radio, platform.now_ms(), rx_buffer) {
            Ok(event) => {
                range_updated |= is_range_update(event);
                record_event_telemetry(telemetry, event);
                led_pulse_ms = merge_led_pulse_ms(led_pulse_ms, event);
                handle_event(platform, app.node.role(), event);
            }
            Err(Error::Receive(_)) => {
                telemetry.rx_error = telemetry.rx_error.wrapping_add(1);
            }
            Err(Error::Protocol(_)) => {
                telemetry.protocol_error = telemetry.protocol_error.wrapping_add(1);
            }
            Err(error) => return Err(error),
        }
    }

    app.radio.clear_events(irq_status)?;
    Ok(RadioIrqOutcome {
        had_radio_activity: true,
        range_updated,
        led_pulse_ms,
    })
}
