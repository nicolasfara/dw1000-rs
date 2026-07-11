use defmt::{info, warn};
use dw1000_rs::{AntennaDelay, AsyncDw1000, DeviceIdentity, Role};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Flex, Level, Output, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, Peripherals};
use embassy_time::{with_timeout, Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

use crate::{
    DW1000_SPI_BAUD_HZ, FAULT_BLINK_COUNT, FAULT_LED_OFF_MS, FAULT_LED_ON_MS, FAULT_LED_PAUSE_MS,
    STM6600_BOOT_TIMEOUT_MS,
};

type SpiBus = Spi<'static, Async>;
type ChipSelect = Output<'static>;
type ResetPin = Flex<'static>;
type IrqPin = ExtiInput<'static>;
type SpiDevice = ExclusiveDevice<SpiBus, ChipSelect, Delay>;
pub(crate) type Radio = AsyncDw1000<SpiDevice, IrqPin, ResetPin>;

pub(crate) struct Board {
    pub(crate) radio: Radio,
    orange_led: Output<'static>,
    green_led: Output<'static>,
}

pub(crate) struct BoardInitError {
    orange_led: Output<'static>,
    green_led: Output<'static>,
    message: &'static str,
}

impl Board {
    pub(crate) async fn init() -> Result<Self, BoardInitError> {
        let mut config = Config::default();
        configure_clocks(&mut config);

        let p: Peripherals = embassy_stm32::init(config);

        let orange_led = Output::new(p.PA8, Level::Low, Speed::Low);
        let green_led = Output::new(p.PA9, Level::Low, Speed::Low);
        let mut charger_enable = Output::new(p.PA10, Level::Low, Speed::Low);
        let mut charger_en1 = Output::new(p.PB0, Level::Low, Speed::Low);
        let mut charger_en2 = Output::new(p.PB1, Level::Low, Speed::Low);
        let mut power_int = ExtiInput::new(p.PB7, p.EXTI7, Pull::Up);
        let mut ps_hold = Output::new(p.PB5, Level::Low, Speed::Low);

        if let Err(message) = bootstrap_board(
            &mut power_int,
            &mut ps_hold,
            &mut charger_enable,
            &mut charger_en1,
            &mut charger_en2,
        )
        .await
        {
            return Err(BoardInitError {
                orange_led,
                green_led,
                message,
            });
        }

        let mut spi_config = SpiConfig::default();
        spi_config.frequency = Hertz(DW1000_SPI_BAUD_HZ);
        let spi = Spi::new(
            p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH2, spi_config,
        );
        let cs = Output::new(p.PA4, Level::High, Speed::High);
        let irq = ExtiInput::new(p.PA2, p.EXTI2, Pull::None);
        let mut reset = Flex::new(p.PA1);
        reset.set_high();
        reset.set_as_input_output(Speed::Low);

        let spi_device = ExclusiveDevice::new(spi, cs, Delay).expect("spi device");
        Ok(Self {
            radio: AsyncDw1000::new(spi_device, irq, reset),
            orange_led,
            green_led,
        })
    }

    pub(crate) fn announce_start(
        &mut self,
        role: Role,
        identity: DeviceIdentity,
        antenna_delay: AntennaDelay,
    ) {
        self.green_led.set_high();
        info!(
            "{=str} starting pan={=u16} short={=u16} antenna_delay={=u16}",
            role_label(role),
            identity.pan_id.raw(),
            identity.short_address.raw(),
            antenna_delay.raw()
        );
    }

    pub(crate) async fn recover_or_fault<const N: usize>(
        &mut self,
        node: &mut dw1000_rs::RangingNode<N>,
        radio_config: &dw1000_rs::RadioConfig,
        now_ms: u32,
        reason: &'static str,
    ) {
        self.orange_led.set_high();
        warn!("recovering radio: {=str}", reason);
        if self.radio.init(&mut Delay, radio_config).await.is_err() {
            self.fault_loop("radio reinit failed").await;
        }
        if self.radio.enable_leds().await.is_err() {
            self.fault_loop("failed to re-enable DW1000 RX/TX leds")
                .await;
        }
        if node
            .recover_link_async(&mut self.radio, now_ms)
            .await
            .is_err()
        {
            self.fault_loop("link recovery failed").await;
        }
        self.orange_led.set_low();
    }

    pub(crate) async fn fault_loop(&mut self, message: &'static str) -> ! {
        self.green_led.set_low();
        self.orange_led.set_low();
        warn!("{=str}", message);
        loop {
            for _ in 0..FAULT_BLINK_COUNT {
                self.orange_led.set_high();
                Timer::after(Duration::from_millis(FAULT_LED_ON_MS)).await;
                self.orange_led.set_low();
                Timer::after(Duration::from_millis(FAULT_LED_OFF_MS)).await;
            }
            Timer::after(Duration::from_millis(FAULT_LED_PAUSE_MS)).await;
        }
    }
}

impl BoardInitError {
    pub(crate) async fn fault_loop(mut self) -> ! {
        self.green_led.set_low();
        self.orange_led.set_low();
        warn!("{=str}", self.message);
        loop {
            for _ in 0..FAULT_BLINK_COUNT {
                self.orange_led.set_high();
                Timer::after(Duration::from_millis(FAULT_LED_ON_MS)).await;
                self.orange_led.set_low();
                Timer::after(Duration::from_millis(FAULT_LED_OFF_MS)).await;
            }
            Timer::after(Duration::from_millis(FAULT_LED_PAUSE_MS)).await;
        }
    }
}

fn configure_clocks(config: &mut Config) {
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

fn role_label(role: Role) -> &'static str {
    match role {
        Role::Tag => "tag",
        Role::Anchor => "anchor",
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
