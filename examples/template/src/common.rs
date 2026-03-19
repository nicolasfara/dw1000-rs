use cortex_m::Peripherals as CorePeripherals;
use defmt::info;
use dw1000_rs::{
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
const MAIN_LOOP_DELAY_MS: u32 = 1;
const RX_BUFFER_LEN: usize = 127;
const DW1000_SPI_BAUD_HZ: u32 = 3_000_000;
const STM6600_BOOT_TIMEOUT_MS: u32 = 1_000;
const DW1000_RESET_PULSE_MS: u32 = 100;
const DW1000_IDLE_LOG_PERIOD_MS: u32 = 1_000;
const PEER_INACTIVITY_TIMEOUT_MS: u32 = 3_000;
const RANGING_REPLY_DELAY_US: u16 = 12_000;
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
        Some(RangingEvent::BlinkReceived(_)) => platform.log("blink received"),
        Some(RangingEvent::NewPeer(_)) => platform.log("new peer"),
        Some(RangingEvent::PeerInactive(_)) => platform.log("peer inactive"),
        Some(RangingEvent::RangeUpdated(snapshot)) if role == Role::Tag => {
            info!(
                "tag distance to anchor {=u16}: {=f32} m",
                snapshot.short_address.raw(),
                snapshot.range_m
            );
        }
        Some(RangingEvent::RangeUpdated(_)) => {}
        Some(RangingEvent::RangingInitReceived(_)) => platform.log("ranging init received"),
        None => {}
    }
}

fn is_range_update(event: Option<RangingEvent>) -> bool {
    matches!(event, Some(RangingEvent::RangeUpdated(_)))
}

fn start_range_update_pulse<Led>(
    orange_led: &mut Led,
    pulse_started_ms: &mut Option<u32>,
    now_ms: u32,
) where
    Led: OldOutputPin,
{
    let _ = orange_led.set_high();
    *pulse_started_ms = Some(now_ms);
}

fn update_range_update_pulse<Led>(
    orange_led: &mut Led,
    pulse_started_ms: &mut Option<u32>,
    now_ms: u32,
) where
    Led: OldOutputPin,
{
    if let Some(started_ms) = *pulse_started_ms {
        if now_ms.wrapping_sub(started_ms) >= RANGE_UPDATE_LED_PULSE_MS {
            let _ = orange_led.set_low();
            *pulse_started_ms = None;
        }
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
    let cp = CorePeripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc.cfgr.sysclk(80.MHz()).freeze(&mut flash.acr, &mut pwr);
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
    let delay = DelayCompat::new(hal::delay::Delay::new(cp.SYST, clocks));
    let mut platform = Stm32Platform::new(delay);

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
    let mut last_idle_log_ms = platform.now_ms();
    let mut range_update_pulse_started_ms = None;
    loop {
        let now_ms = platform.now_ms();
        let event = app.node.tick(&mut app.radio, now_ms).unwrap();
        if event.is_some() {
            last_radio_activity_ms = now_ms;
        }
        handle_event(&mut platform, role, event);
        if is_range_update(event) {
            start_range_update_pulse(
                &mut orange_led,
                &mut range_update_pulse_started_ms,
                platform.now_ms(),
            );
        }

        let irq_outcome =
            service_radio_irq(&mut platform, &mut app, &mut rx_buffer).unwrap();
        if irq_outcome.range_updated {
            start_range_update_pulse(
                &mut orange_led,
                &mut range_update_pulse_started_ms,
                platform.now_ms(),
            );
        }
        if irq_outcome.had_radio_activity {
            last_radio_activity_ms = platform.now_ms();
        } else if platform.now_ms().wrapping_sub(last_idle_log_ms) >= DW1000_IDLE_LOG_PERIOD_MS
            && platform.now_ms().wrapping_sub(last_radio_activity_ms) >= DW1000_IDLE_LOG_PERIOD_MS
        {
            platform.log("waiting for dw1000 traffic");
            last_idle_log_ms = platform.now_ms();
        }
        update_range_update_pulse(
            &mut orange_led,
            &mut range_update_pulse_started_ms,
            platform.now_ms(),
        );
        platform.delay_ms(MAIN_LOOP_DELAY_MS);
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
    role: Role,
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

    match role {
        Role::Anchor => {
            let _ = charger_en1.set_high();
            let _ = charger_en2.set_low();
        }
        Role::Tag => {
            let _ = charger_en1.set_low();
            let _ = charger_en2.set_low();
        }
    }

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
        });
    }

    let irq_status = app.radio.read_sys_status()?;
    let mut range_updated = false;

    if irq_status.contains(status::TX_FRAME_SENT) {
        let event = app.node.on_tx_done(&mut app.radio)?;
        range_updated |= is_range_update(event);
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
                handle_event(platform, app.node.role(), event);
            }
            Err(Error::Receive(_)) => platform.log("rx error"),
            Err(Error::Protocol(_)) => platform.log("protocol resync"),
            Err(error) => return Err(error),
        }
    }

    app.radio.clear_events(irq_status)?;
    Ok(RadioIrqOutcome {
        had_radio_activity: true,
        range_updated,
    })
}
