use embassy_time::Instant;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;
use crate::constants::{FC_1, FC_1_BLINK, FC_2, FC_2_SHORT, LONG_MAC_LEN, NO_SUB, PANADR, SHORT_MAC_LEN};
use crate::dw1000::{Dw1000, Dw1000Error};
use crate::time::DW1000Time;

const LEN_DATA: usize = 90;
const MAX_DEVICES: usize = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceType {
    Anchor,
    Tag,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MessageType {
    Poll = 0,
    PollAck = 1,
    Range = 2,
    RangeReport = 3,
    RangeFailed = 255,
    Blink = 4,
    RangingInit = 5,
}

impl MessageType {
    fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(MessageType::Poll),
            1 => Some(MessageType::PollAck),
            2 => Some(MessageType::Range),
            3 => Some(MessageType::RangeReport),
            4 => Some(MessageType::Blink),
            5 => Some(MessageType::RangingInit),
            255 => Some(MessageType::RangeFailed),
            _ => None,
        }
    }
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceInfo {
    pub address: [u8; 8],
    pub short_address: [u8; 2],
    pub range: f32,
    pub rx_power: f32,
    pub fp_power: f32,
    pub quality: f32,
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RangingEvent {
    None,
    BlinkReceived { device: DeviceInfo },
    NewDevice { device: DeviceInfo },
    InactiveDevice { short_address: [u8; 2] },
    NewRange { device: DeviceInfo },
    RangingInitReceived { short_address: [u8; 2] },
    DeviceNotFound { short_address: [u8; 2] },
    UnexpectedMessage,
    ProtocolFailed,
}

struct NetworkDevice {
    address: [u8; 8],
    short_address: [u8; 2],
    index: u8,
    range: f32,
    rx_power: f32,
    fp_power: f32,
    quality: f32,
    reply_time: u16,
    time_poll_sent: DW1000Time,
    time_poll_received: DW1000Time,
    time_poll_ack_sent: DW1000Time,
    time_poll_ack_received: DW1000Time,
    time_range_sent: DW1000Time,
    time_range_received: DW1000Time,
    last_activity: Instant,
}

impl NetworkDevice {
    fn new(address: [u8; 8], short_address: [u8; 2], index: u8) -> Self {
        Self {
            address,
            short_address,
            index,
            range: 0.0,
            rx_power: 0.0,
            fp_power: 0.0,
            quality: 0.0,
            reply_time: 0,
            time_poll_sent: DW1000Time::default(),
            time_poll_received: DW1000Time::default(),
            time_poll_ack_sent: DW1000Time::default(),
            time_poll_ack_received: DW1000Time::default(),
            time_range_sent: DW1000Time::default(),
            time_range_received: DW1000Time::default(),
            last_activity: Instant::now(),
        }
    }

    fn note_activity(&mut self) {
        self.last_activity = Instant::now();
    }

    fn is_inactive(&self, timeout_ms: u64) -> bool {
        Instant::now().duration_since(self.last_activity).as_millis() > timeout_ms
    }

    fn to_device_info(&self) -> DeviceInfo {
        DeviceInfo {
            address: self.address,
            short_address: self.short_address,
            range: self.range,
            rx_power: self.rx_power,
            fp_power: self.fp_power,
            quality: self.quality,
        }
    }
}

pub struct Dw1000Ranging<'a, SPI, IRQ, RST, DELAY> {
    module: &'a mut Dw1000<SPI, IRQ, RST, DELAY>,
    device_type: DeviceType,
    current_address: [u8; 8],
    current_short_address: [u8; 2],
    last_sent_to_short_address: [u8; 2],
    network_devices: heapless::Vec<NetworkDevice, MAX_DEVICES>,
    sent_ack: bool,
    received_ack: bool,
    last_activity: Instant,
    data: [u8; LEN_DATA],
    expected_msg_id: MessageType,
    protocol_failed: bool,
    reply_delay_time_us: u16,
    timer_delay: u16,
    reset_period: u32,
    use_range_filter: bool,
    range_filter_value: u16,
    timer: Instant,
    counter_for_blink: u16,
}

impl<'a, SPI, IRQ, RST, DELAY> Dw1000Ranging<'a, SPI, IRQ, RST, DELAY>
where
    SPI: SpiDevice,
    IRQ: InputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    const DEFAULT_RESET_PERIOD: u32 = 200;
    const DEFAULT_REPLY_DELAY_TIME_US: u16 = 7000;
    const DEFAULT_TIMER_DELAY: u16 = 80;

    pub fn new(module: &'a mut Dw1000<SPI, IRQ, RST, DELAY>, device_type: DeviceType) -> Self {
        Dw1000Ranging {
            module,
            device_type,
            current_address: [0; 8],
            current_short_address: [0; 2],
            last_sent_to_short_address: [0; 2],
            network_devices: heapless::Vec::new(),
            sent_ack: false,
            received_ack: false,
            last_activity: Instant::now(),
            data: [0u8; LEN_DATA],
            expected_msg_id: MessageType::Poll,
            protocol_failed: false,
            reply_delay_time_us: Self::DEFAULT_REPLY_DELAY_TIME_US,
            timer_delay: Self::DEFAULT_TIMER_DELAY,
            reset_period: Self::DEFAULT_RESET_PERIOD,
            use_range_filter: false,
            range_filter_value: 15,
            timer: Instant::now(),
            counter_for_blink: 0,
        }
    }

    pub fn init_communication(&mut self) -> Result<(), Dw1000Error<SPI::Error>> {
        self.module.init()
    }

    /// Configure the DW1000 network parameters
    ///
    /// This corresponds to DW1000Ranging::configureNetwork() in the C++ implementation.
    /// It sets up device address, network ID, and operation mode (data rate, pulse frequency, preamble length).
    ///
    /// # Arguments
    /// * `device_address` - The short device address (16-bit)
    /// * `network_id` - The network/PAN ID (16-bit)
    /// * `mode` - Configuration mode as [data_rate, pulse_frequency, preamble_length]
    ///
    /// # Example
    /// ```ignore
    /// // MODE_LONGDATA_RANGE_LOWPOWER equivalent: [TRX_RATE_110KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_2048]
    /// ranging.configure_network(0x1234, 0xDECA, &[0x00, 0x01, 0x0C])?;
    /// ```
    pub fn configure_network(
        &mut self,
        device_address: u16,
        network_id: u16,
        mode: &[u8],
    ) -> Result<(), Dw1000Error<SPI::Error>> {
        // Put device in idle mode
        self.module.idle()?;
        self.module.set_device_address(device_address);
        self.module.set_network_id(network_id);
        self.module.enable_mode(mode);
        self.module.commit_configuration()
    }

    pub fn set_current_address(&mut self, address: [u8; 8], short_address: [u8; 2]) {
        self.current_address = address;
        self.current_short_address = short_address;
    }

    pub fn set_use_range_filter(&mut self, enabled: bool) {
        self.use_range_filter = enabled;
    }

    pub fn set_range_filter_value(&mut self, value: u16) {
        self.range_filter_value = if value < 2 { 2 } else { value };
    }

    pub fn round(&mut self) -> Result<RangingEvent, Dw1000Error<SPI::Error>> {
        self.check_for_reset();

        let now = Instant::now();
        if now.duration_since(self.timer).as_millis() > self.timer_delay as u64 {
            self.timer = now;
            return self.timer_tick();
        }

        if self.sent_ack {
            self.sent_ack = false;
            let message_type = Self::detect_message_type(&self.data);

            if let Some(msg_type) = message_type {
                if msg_type != MessageType::PollAck
                    && msg_type != MessageType::Poll
                    && msg_type != MessageType::Range
                {
                    return Ok(RangingEvent::None);
                }

                match self.device_type {
                    DeviceType::Anchor => {
                        if msg_type == MessageType::PollAck {
                            let short_address = self.last_sent_to_short_address;
                            if let Some(_device) = self.search_distant_device_mut(&short_address) {
                                // Get transmit timestamp - placeholder for actual implementation
                                // _device.time_poll_ack_sent = self.module.get_transmit_timestamp()?;
                            }
                        }
                    }
                    DeviceType::Tag => {
                        if msg_type == MessageType::Poll {
                            // Get transmit timestamp - placeholder
                            // let time_poll_sent = self.module.get_transmit_timestamp()?;

                            if self.last_sent_to_short_address == [0xFF, 0xFF] {
                                // Broadcast - update all devices
                                for _device in &mut self.network_devices {
                                    // _device.time_poll_sent = time_poll_sent;
                                }
                            } else {
                                let short_address = self.last_sent_to_short_address;
                                if let Some(_device) = self.search_distant_device_mut(&short_address) {
                                    // _device.time_poll_sent = time_poll_sent;
                                }
                            }
                        } else if msg_type == MessageType::Range {
                            // Similar logic for Range
                            if self.last_sent_to_short_address == [0xFF, 0xFF] {
                                for _device in &mut self.network_devices {
                                    // _device.time_range_sent = time_range_sent;
                                }
                            } else {
                                let short_address = self.last_sent_to_short_address;
                                if let Some(_device) = self.search_distant_device_mut(&short_address) {
                                    // _device.time_range_sent = time_range_sent;
                                }
                            }
                        }
                    }
                }
            }
        }

        if self.received_ack {
            self.received_ack = false;

            // Read data from module - placeholder
            // self.module.get_data(&mut self.data)?;

            let message_type = Self::detect_message_type(&self.data);

            if let Some(msg_type) = message_type {
                return self.handle_received_message(msg_type);
            }
        }

        Ok(RangingEvent::None)
    }

    fn handle_received_message(
        &mut self,
        message_type: MessageType,
    ) -> Result<RangingEvent, Dw1000Error<SPI::Error>> {
        match (message_type, self.device_type) {
            (MessageType::Blink, DeviceType::Anchor) => {
                let mut address = [0u8; 8];
                let mut short_address = [0u8; 2];
                self.decode_blink_frame(&mut address, &mut short_address);

                if self.add_network_device(address, short_address) {
                    // transmitRangingInit - placeholder
                    self.note_activity();
                    self.expected_msg_id = MessageType::Poll;
                    return Ok(RangingEvent::BlinkReceived {
                        device: DeviceInfo {
                            address,
                            short_address,
                            range: 0.0,
                            rx_power: 0.0,
                            fp_power: 0.0,
                            quality: 0.0,
                        },
                    });
                }
            }

            (MessageType::RangingInit, DeviceType::Tag) => {
                let mut short_address = [0u8; 2];
                self.decode_long_mac_frame(&mut short_address);

                if self.add_network_device_short(short_address) {
                    self.note_activity();
                    return Ok(RangingEvent::NewDevice {
                        device: DeviceInfo {
                            address: [0; 8],
                            short_address,
                            range: 0.0,
                            rx_power: 0.0,
                            fp_power: 0.0,
                            quality: 0.0,
                        },
                    });
                }
            }

            _ => {
                // Short MAC frame
                let mut address = [0u8; 2];
                self.decode_short_mac_frame(&mut address);

                let device_index = self.search_distant_device_index(&address);

                if device_index.is_none() {
                    return Ok(RangingEvent::DeviceNotFound {
                        short_address: address,
                    });
                }

                return match self.device_type {
                    DeviceType::Anchor => {
                        self.handle_anchor_message(message_type, device_index.unwrap())
                    }
                    DeviceType::Tag => {
                        self.handle_tag_message(message_type, device_index.unwrap())
                    }
                }
            }
        }

        Ok(RangingEvent::None)
    }

    fn handle_anchor_message(
        &mut self,
        message_type: MessageType,
        device_index: usize,
    ) -> Result<RangingEvent, Dw1000Error<SPI::Error>> {
        if message_type != self.expected_msg_id {
            self.protocol_failed = true;
        }

        match message_type {
            MessageType::Poll => {
                let number_devices = self.data[SHORT_MAC_LEN as usize + 1] as usize;

                for i in 0..number_devices {
                    let offset = SHORT_MAC_LEN as usize + 2 + i * 4;
                    let short_address = [self.data[offset], self.data[offset + 1]];

                    if short_address == self.current_short_address {
                        let reply_time =
                            u16::from_le_bytes([self.data[offset + 2], self.data[offset + 3]]);
                        self.reply_delay_time_us = reply_time;
                        self.protocol_failed = false;

                        if let Some(device) = self.network_devices.get_mut(device_index) {
                            // device.time_poll_received = self.module.get_receive_timestamp()?;
                            device.note_activity();
                        }

                        self.expected_msg_id = MessageType::Range;
                        // transmitPollAck - placeholder
                        self.note_activity();
                        break;
                    }
                }
            }

            MessageType::Range => {
                let number_devices = self.data[SHORT_MAC_LEN as usize + 1] as usize;

                for i in 0..number_devices {
                    let offset = SHORT_MAC_LEN as usize + 2 + i * 17;
                    let short_address = [self.data[offset], self.data[offset + 1]];

                    if short_address == self.current_short_address {
                        if let Some(device) = self.network_devices.get_mut(device_index) {
                            // device.time_range_received = self.module.get_receive_timestamp()?;
                            self.expected_msg_id = MessageType::Poll;

                            if !self.protocol_failed {
                                // Extract timestamps
                                // device.time_poll_sent.set_timestamp(&self.data[offset + 4..]);
                                // device.time_poll_ack_received.set_timestamp(&self.data[offset + 9..]);
                                // device.time_range_sent.set_timestamp(&self.data[offset + 14..]);

                                // Compute range
                                let time_poll_sent = device.time_poll_sent;
                                let time_poll_received = device.time_poll_received;
                                let time_poll_ack_sent = device.time_poll_ack_sent;
                                let time_poll_ack_received = device.time_poll_ack_received;
                                let time_range_sent = device.time_range_sent;
                                let time_range_received = device.time_range_received;

                                let distance = Self::compute_range_asymmetric_static(
                                    time_poll_sent,
                                    time_poll_received,
                                    time_poll_ack_sent,
                                    time_poll_ack_received,
                                    time_range_sent,
                                    time_range_received,
                                );

                                let filtered_distance = if self.use_range_filter && device.range != 0.0
                                {
                                    Self::filter_value(
                                        distance,
                                        device.range,
                                        self.range_filter_value,
                                    )
                                } else {
                                    distance
                                };

                                // device.rx_power = self.module.get_receive_power()?;
                                device.range = filtered_distance;
                                // device.fp_power = self.module.get_first_path_power()?;
                                // device.quality = self.module.get_receive_quality()?;

                                // transmitRangeReport - placeholder
                                let device_info = device.to_device_info();
                                self.note_activity();

                                return Ok(RangingEvent::NewRange {
                                    device: device_info,
                                });
                            } else {
                                // transmitRangeFailed - placeholder
                            }
                        }
                        self.note_activity();
                        break;
                    }
                }
            }

            _ => {}
        }

        Ok(RangingEvent::None)
    }

    fn handle_tag_message(
        &mut self,
        message_type: MessageType,
        device_index: usize,
    ) -> Result<RangingEvent, Dw1000Error<SPI::Error>> {
        if message_type != self.expected_msg_id {
            return Ok(RangingEvent::UnexpectedMessage);
        }

        match message_type {
            MessageType::PollAck => {
                if let Some(device) = self.network_devices.get_mut(device_index) {
                    // device.time_poll_ack_received = self.module.get_receive_timestamp()?;
                    device.note_activity();

                    if device.index == (self.network_devices.len() - 1) as u8 {
                        self.expected_msg_id = MessageType::RangeReport;
                        // transmitRange(nullptr) - broadcast
                    }
                }
            }

            MessageType::RangeReport => {
                if let Some(device) = self.network_devices.get_mut(device_index) {
                    let offset = 1 + SHORT_MAC_LEN as usize;
                    let cur_range = f32::from_le_bytes([
                        self.data[offset],
                        self.data[offset + 1],
                        self.data[offset + 2],
                        self.data[offset + 3],
                    ]);
                    let cur_rx_power = f32::from_le_bytes([
                        self.data[offset + 4],
                        self.data[offset + 5],
                        self.data[offset + 6],
                        self.data[offset + 7],
                    ]);

                    let filtered_range = if self.use_range_filter && device.range != 0.0 {
                        Self::filter_value(cur_range, device.range, self.range_filter_value)
                    } else {
                        cur_range
                    };

                    device.range = filtered_range;
                    device.rx_power = cur_rx_power;

                    return Ok(RangingEvent::NewRange {
                        device: device.to_device_info(),
                    });
                }
            }

            MessageType::RangeFailed => {
                return Ok(RangingEvent::ProtocolFailed);
            }

            _ => {}
        }

        Ok(RangingEvent::None)
    }

    fn detect_message_type(datas: &[u8]) -> Option<MessageType> {
        if datas[0] == FC_1_BLINK {
            Some(MessageType::Blink)
        } else if datas[0] == FC_1 && datas[1] == FC_2 {
            MessageType::from_u8(datas[LONG_MAC_LEN as usize])
        } else if datas[0] == FC_1 && datas[1] == FC_2_SHORT {
            MessageType::from_u8(datas[SHORT_MAC_LEN as usize])
        } else {
            None
        }
    }

    fn check_for_reset(&mut self) {
        let now = Instant::now();
        if !self.sent_ack
            && !self.received_ack
            && (now.duration_since(self.last_activity).as_millis() > self.reset_period as u64)
        {
            self.reset_inactive();
        }
    }

    fn reset_inactive(&mut self) {
        if self.device_type == DeviceType::Anchor {
            self.expected_msg_id = MessageType::Poll;
            // receiver() - placeholder
        }
        self.note_activity();
    }

    fn timer_tick(&mut self) -> Result<RangingEvent, Dw1000Error<SPI::Error>> {
        if !self.network_devices.is_empty() && self.counter_for_blink != 0 {
            if self.device_type == DeviceType::Tag {
                self.expected_msg_id = MessageType::PollAck;
                // transmitPoll(nullptr) - broadcast poll
            }
        } else if self.counter_for_blink == 0 {
            if self.device_type == DeviceType::Tag {
                // transmitBlink()
            }
            return self.check_for_inactive_devices();
        }

        self.counter_for_blink += 1;
        if self.counter_for_blink > 20 {
            self.counter_for_blink = 0;
        }

        Ok(RangingEvent::None)
    }

    fn check_for_inactive_devices(&mut self) -> Result<RangingEvent, Dw1000Error<SPI::Error>> {
        let mut inactive_index = None;

        for (i, device) in self.network_devices.iter().enumerate() {
            if device.is_inactive(self.reset_period as u64) {
                inactive_index = Some(i);
                break;
            }
        }

        if let Some(index) = inactive_index {
            let short_address = self.network_devices[index].short_address;
            self.network_devices.swap_remove(index);

            // Update indices
            for (i, device) in self.network_devices.iter_mut().enumerate() {
                device.index = i as u8;
            }

            return Ok(RangingEvent::InactiveDevice { short_address });
        }

        Ok(RangingEvent::None)
    }

    fn search_distant_device(&self, short_address: &[u8; 2]) -> Option<&NetworkDevice> {
        self.network_devices
            .iter()
            .find(|d| d.short_address == *short_address)
    }

    fn search_distant_device_mut(
        &mut self,
        short_address: &[u8; 2],
    ) -> Option<&mut NetworkDevice> {
        self.network_devices
            .iter_mut()
            .find(|d| d.short_address == *short_address)
    }

    fn search_distant_device_index(&self, short_address: &[u8; 2]) -> Option<usize> {
        self.network_devices
            .iter()
            .position(|d| d.short_address == *short_address)
    }

    fn add_network_device(&mut self, address: [u8; 8], short_address: [u8; 2]) -> bool {
        if self.search_distant_device(&short_address).is_some() {
            return false;
        }

        if self.device_type == DeviceType::Anchor {
            self.network_devices.clear();
        }

        let index = self.network_devices.len() as u8;
        if self
            .network_devices
            .push(NetworkDevice::new(address, short_address, index))
            .is_err()
        {
            return false;
        }

        true
    }

    fn add_network_device_short(&mut self, short_address: [u8; 2]) -> bool {
        self.add_network_device([0; 8], short_address)
    }

    fn note_activity(&mut self) {
        self.last_activity = Instant::now();
    }

    fn compute_range_asymmetric_static(
        time_poll_sent: DW1000Time,
        time_poll_received: DW1000Time,
        time_poll_ack_sent: DW1000Time,
        time_poll_ack_received: DW1000Time,
        time_range_sent: DW1000Time,
        time_range_received: DW1000Time,
    ) -> f32 {
        let round1 = (time_poll_ack_received - time_poll_sent).wrapped();
        let reply1 = (time_poll_ack_sent - time_poll_received).wrapped();
        let round2 = (time_range_received - time_poll_ack_sent).wrapped();
        let reply2 = (time_range_sent - time_poll_ack_received).wrapped();

        let tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
        tof.as_meters()
    }

    fn filter_value(value: f32, previous_value: f32, number_of_elements: u16) -> f32 {
        let k = 2.0 / (number_of_elements as f32 + 1.0);
        value * k + previous_value * (1.0 - k)
    }

    fn decode_blink_frame(&self, _address: &mut [u8; 8], _short_address: &mut [u8; 2]) {
        // Placeholder for MAC frame decoding
        // Implementation depends on DW1000Mac functionality
    }

    fn decode_long_mac_frame(&self, _address: &mut [u8; 2]) {
        // Placeholder for MAC frame decoding
    }

    fn decode_short_mac_frame(&self, _address: &mut [u8; 2]) {
        // Placeholder for MAC frame decoding
    }
}