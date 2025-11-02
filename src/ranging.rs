use embassy_time::Instant;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;
use crate::constants::{FC_1, FC_1_BLINK, FC_2, FC_2_SHORT, LONG_MAC_LEN, SHORT_MAC_LEN};
use crate::dw1000::{Dw1000, Dw1000Error};

pub struct Dw1000Ranging<'a, SPI, IRQ, RST, DELAY> {
    module: &'a mut Dw1000<SPI, IRQ, RST, DELAY>,
    sent_ack: bool,
    received_ack: bool,
    last_activity: Instant,
    data: [u8; 90]
}

impl <'a, SPI, IRQ, RST, DELAY> Dw1000Ranging<'a, SPI, IRQ, RST, DELAY>
where
    SPI: SpiDevice,
    IRQ: InputPin,
    RST: OutputPin,
    DELAY: DelayNs
{
    const DEFAULT_RESET_PERIOD: u8 = 200;
    const DEFAULT_REPLY_DELAY_TIME_US: u16 = 7000;
    const DEFAULT_TIMER_DELAY: u16 = 1000;
    const BLINK: u8 = 4;
    const POLL_ACK: u8 = 1;
    const POLL: u8 = 0;
    const RANGE: u8 = 2;

    pub fn new(module: &'a mut Dw1000<SPI, IRQ, RST, DELAY>) -> Self {
        Dw1000Ranging { module, sent_ack: false, received_ack: false, last_activity: Instant::now(), data: [0u8; 90] }
    }

    pub fn init_communication(&mut self) -> Result<(), Dw1000Error<SPI::Error>> {
        self.module.init()
    }

    pub fn round(&mut self) -> Result<(), Dw1000Error<SPI::Error>> {
        if self.sent_ack {
            self.sent_ack = false;
            let message_type = Self::detect_message_type(&self.data);
            if message_type != Self::POLL_ACK && message_type != Self::POLL && message_type != Self::RANGE {
                return Ok(());
            }
        }
        Ok(())
    }
    
    fn detect_message_type(datas: &[u8]) -> u8 {
        if datas[0] == FC_1_BLINK {
            Self::BLINK
        } else if datas[0] == FC_1 && datas[1] == FC_2 {
            datas[LONG_MAC_LEN as usize]
        } else if datas[0] == FC_1 && datas[1] == FC_2_SHORT {
            datas[SHORT_MAC_LEN as usize]
        } else {
            panic!("Unknown message type");
        }
    }

    fn check_for_reset(&mut self) {
        let now = Instant::now();
        if !self.sent_ack && !self.received_ack && (now.duration_since(self.last_activity).as_millis() > Self::DEFAULT_RESET_PERIOD as u64) {
            

        }
    }
}