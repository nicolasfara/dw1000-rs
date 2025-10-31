#[cfg(feature = "defmt")]
use defmt::Format;
use libm::round;
use rand_core::RngCore;

pub struct DW1000Device<R: RngCore> {
    randomizer: R,
    address: [u8; 8],
    short_address: [u8; 2],
    activity: i32,
    reply_delay_time_us: u16,
    index: i8, // Not used -- TODO: remove
    range: i16,
    rx_power: i16,
    fp_power: i16,
    quality: i16
}

impl <R: RngCore> DW1000Device<R> {

    pub fn new(rng: &mut R) -> DW1000Device<R> {
        DW1000Device {
            randomizer: rng.clone(),
            address: [0u8; 8],
            short_address: Self::random_short_address(rng),
            activity: 0,
            reply_delay_time_us: 0,
            index: -1,
            range: -1,
            rx_power: -1,
            fp_power: -1,
            quality: -1
        }
    }

    pub fn from_addresses(rng: &mut R, address: [u8; 8], short_address: [u8; 2]) -> DW1000Device<R> {
        DW1000Device {
            randomizer: rng.clone(),
            address,
            short_address,
            activity: 0,
            reply_delay_time_us: 0,
            index: -1,
            range: -1,
            rx_power: -1,
            fp_power: -1,
            quality: -1
        }
    }

    pub fn set_reply_time(&mut self, delay_time_us: u16) {
        self.reply_delay_time_us = delay_time_us;
    }

    pub fn set_address(&mut self, address: [u8; 8]) {
        self.address = address;
    }

    pub fn set_short_address(&mut self, short_address: [u8; 2]) {
        self.short_address = short_address;
    }

    pub fn set_short_range(&mut self, range: f64) {
        self.range = round(range * 100f64) as i16;
    }

    pub fn set_rx_power(&mut self, power: f64) {
        self.rx_power = round(power * 100f64) as i16;
    }

    pub fn set_fp_power(&mut self, power: f64) {
        self.fp_power = round(power * 100f64) as i16;
    }

    pub fn set_quality(&mut self, quality: f64) {
        self.quality = round(quality * 100f64) as i16;
    }

    pub fn get_address(&self) -> [u8; 8] {
        self.address
    }

    pub fn get_short_address(&self) -> [u8; 2] {
        self.short_address
    }

    fn random_short_address(&mut self) -> [u8; 2] {
        let mut addr = [0u8; 2];
        self.randomizer.fill_bytes(&mut addr);
        addr
    }
}