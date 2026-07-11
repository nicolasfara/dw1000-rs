//! Async DW1000 driver implementation.

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::Operation;
use embedded_hal_async::{delay::DelayNs, digital::Wait, spi::SpiDevice};

use crate::config::{RadioConfig, RxOptions, TxOptions};
use crate::device::{DeviceIdentity, RxFrame, SignalMetrics, SysStatus, Timestamps};
use crate::driver_core::{
    apply_clock_mode, build_header, cleared_interrupt_mask, compose_base_register_fields,
    compose_gpio_led_mode, compose_led_blink_enable, compose_led_clock_enable,
    compose_phy_register_fields, compose_receive_sys_ctrl, compose_transmit_sys_ctrl,
    config_register_writes, fs_xtalt_value, header_len, parse_rx_snapshot, prepare_idle_state,
    receive_status_clear_mask, select_tuning_values, set_lde_load_preamble,
    set_lde_restore_preamble, transmit_status_clear_mask, ClockMode, DriverRuntime,
    DELAYED_TX_LATE_MASK, HPDWARN_HI_BIT, OTP_ADDRESS_LDOTUNE, OTP_ADDRESS_XTAL_TRIM,
    OTP_SF_LDO_KICK, PMSC_SOFTRESET_CLEAR, PMSC_SOFTRESET_RX, SYS_STATUS_HI_SUB, TXFRS_LOW_BIT,
};
use crate::error::{Error, RxError};
use crate::registers::{
    sys_status_from_bytes, sys_status_to_bytes, Register, EXPECTED_DEVICE_ID, GPIO_MODE_SUB,
    LEN_CHAN_CTRL, LEN_DEV_ID, LEN_GPIO_MODE, LEN_OTP_ADDR, LEN_OTP_CTRL, LEN_OTP_RDAT,
    LEN_PMSC_CTRL0, LEN_PMSC_LEDC, LEN_RX_FINFO, LEN_RX_FQUAL, LEN_RX_STAMP, LEN_RX_TIME,
    LEN_SYS_CFG, LEN_SYS_CTRL, LEN_SYS_MASK, LEN_SYS_STATUS, LEN_TX_FCTRL, LEN_TX_STAMP,
    NO_SUBADDRESS, OTP_ADDR_SUB, OTP_CTRL_SUB, OTP_RDAT_SUB, OTP_SF_SUB, PMSC_CTRL0_SUB,
    PMSC_LEDC_SUB, PMSC_SOFTRESET_SUB, RX_STAMP_SUB, TX_STAMP_SUB,
};
use crate::time::{DelayedTime, DwTime};

/// Async DW1000 driver.
#[derive(Debug)]
pub struct AsyncDw1000<SPI, IRQ, RST> {
    spi: SPI,
    irq: IRQ,
    reset: RST,
    sys_ctrl: [u8; LEN_SYS_CTRL],
    tx_fctrl: [u8; LEN_TX_FCTRL],
    runtime: DriverRuntime,
}

impl<SPI, IRQ, RST, PinE> AsyncDw1000<SPI, IRQ, RST>
where
    SPI: SpiDevice,
    IRQ: InputPin<Error = PinE> + Wait<Error = PinE>,
    RST: OutputPin<Error = PinE>,
{
    /// Creates a new driver instance.
    pub fn new(spi: SPI, irq: IRQ, reset: RST) -> Self {
        Self {
            spi,
            irq,
            reset,
            sys_ctrl: [0; LEN_SYS_CTRL],
            tx_fctrl: [0; LEN_TX_FCTRL],
            runtime: DriverRuntime::new(),
        }
    }

    /// Initializes the DW1000 and applies the supplied configuration.
    pub async fn init(
        &mut self,
        delay: &mut impl DelayNs,
        config: &RadioConfig,
    ) -> Result<(), Error<SPI::Error, PinE>> {
        self.hard_reset(delay).await?;
        self.verify_device_id().await?;
        // OTP reads and the LDE microcode load require the XTI system clock.
        self.enable_clock(ClockMode::Xti).await?;
        delay.delay_ms(5).await;
        self.clear_interrupts().await?;
        self.kick_ldo_tune().await?;
        self.runtime.xtal_trim = Some(self.read_otp(OTP_ADDRESS_XTAL_TRIM).await?[0]);
        self.manage_lde(delay).await?;
        self.enable_clock(ClockMode::Auto).await?;
        delay.delay_ms(5).await;
        self.reconfigure(config).await
    }

    /// Applies a new radio configuration without performing a reset.
    pub async fn reconfigure(
        &mut self,
        config: &RadioConfig,
    ) -> Result<(), Error<SPI::Error, PinE>> {
        let phy = self.runtime.reconfigure(config)?;
        let tuning = select_tuning_values(phy).map_err(Error::InvalidConfig)?;

        self.idle().await?;

        let mut sys_cfg = [0u8; LEN_SYS_CFG];
        let mut sys_mask = [0u8; LEN_SYS_MASK];
        let mut chan_ctrl = [0u8; LEN_CHAN_CTRL];
        self.tx_fctrl = [0; LEN_TX_FCTRL];
        compose_base_register_fields(
            &mut sys_cfg,
            &mut sys_mask,
            config.interrupt_polarity_high,
            config.receiver_auto_reenable,
            phy.smart_power,
        );
        let sfd_len =
            compose_phy_register_fields(&mut sys_cfg, &mut self.tx_fctrl, &mut chan_ctrl, phy);

        let xtal_trim = match self.runtime.xtal_trim {
            Some(trim) => trim,
            None => self.read_otp(OTP_ADDRESS_XTAL_TRIM).await?[0],
        };

        let tx_fctrl = self.tx_fctrl;
        let writes = config_register_writes(
            &config.address.identity,
            &sys_cfg,
            &sys_mask,
            &chan_ctrl,
            &tx_fctrl,
            sfd_len,
            &tuning,
            self.runtime.antenna_delay,
            fs_xtalt_value(xtal_trim),
        );
        for write in &writes {
            self.write_register(write.register, write.subaddress, write.data())
                .await?;
        }
        Ok(())
    }

    /// Starts a receive session.
    pub async fn start_receive(
        &mut self,
        options: RxOptions,
    ) -> Result<(), Error<SPI::Error, PinE>> {
        self.idle().await?;
        self.clear_receive_status().await?;
        self.sys_ctrl = [0; LEN_SYS_CTRL];
        self.runtime.begin_receive_session(options.permanent);
        if let Some(scheduled) = options.delayed_time {
            self.write_register(
                Register::DxTime,
                NO_SUBADDRESS,
                &scheduled.dx_time().to_bytes(),
            )
            .await?;
        }
        compose_receive_sys_ctrl(
            &mut self.sys_ctrl,
            self.runtime.frame_check,
            options.delayed_time.is_some(),
        );
        let sys_ctrl = self.sys_ctrl;
        self.write_register(Register::SysCtrl, NO_SUBADDRESS, &sys_ctrl)
            .await?;

        if options.delayed_time.is_some() {
            // If the programmed time already passed the receiver would stall
            // until timer wrap; fall back to an immediate receive instead.
            let mut status_hi = [0u8; 1];
            self.read_register(Register::SysStatus, SYS_STATUS_HI_SUB, &mut status_hi)
                .await?;
            if status_hi[0] & HPDWARN_HI_BIT != 0 {
                self.idle().await?;
                self.sys_ctrl = [0; LEN_SYS_CTRL];
                self.runtime.begin_receive_session(options.permanent);
                compose_receive_sys_ctrl(&mut self.sys_ctrl, self.runtime.frame_check, false);
                let sys_ctrl = self.sys_ctrl;
                self.write_register(Register::SysCtrl, NO_SUBADDRESS, &sys_ctrl)
                    .await?;
                return Err(Error::DelayedReceiveTooLate);
            }
        }
        Ok(())
    }

    /// Transmits a frame.
    pub async fn transmit(
        &mut self,
        frame: &[u8],
        options: TxOptions,
    ) -> Result<(), Error<SPI::Error, PinE>> {
        let frame_len = self
            .runtime
            .checked_frame_len(frame.len())
            .map_err(|(len, max)| Error::FrameTooLong { len, max })?;

        self.idle().await?;
        self.clear_transmit_status().await?;
        self.write_register(Register::TxBuffer, NO_SUBADDRESS, frame)
            .await?;
        self.tx_fctrl[0] = (frame_len & 0xFF) as u8;
        self.tx_fctrl[1] &= 0xFC;
        self.tx_fctrl[1] |= ((frame_len >> 8) & 0x03) as u8;
        let tx_fctrl = self.tx_fctrl;
        self.write_register(Register::TxFctrl, NO_SUBADDRESS, &tx_fctrl)
            .await?;

        self.sys_ctrl = [0; LEN_SYS_CTRL];
        self.runtime.begin_transmit_session();
        if let Some(scheduled) = options.delayed_time {
            self.write_register(
                Register::DxTime,
                NO_SUBADDRESS,
                &scheduled.dx_time().to_bytes(),
            )
            .await?;
        }
        compose_transmit_sys_ctrl(
            &mut self.sys_ctrl,
            self.runtime.frame_check,
            options.wait_for_response,
            options.delayed_time.is_some(),
        );
        let sys_ctrl = self.sys_ctrl;
        self.write_register(Register::SysCtrl, NO_SUBADDRESS, &sys_ctrl)
            .await?;

        if options.delayed_time.is_some() {
            // HPDWARN/TXPUTE means the delayed send was scheduled too late;
            // abort instead of stalling until timer wrap (~17 s).
            let mut status_hi = [0u8; 2];
            self.read_register(Register::SysStatus, SYS_STATUS_HI_SUB, &mut status_hi)
                .await?;
            if u16::from_le_bytes(status_hi) & DELAYED_TX_LATE_MASK != 0 {
                self.idle().await?;
                // Do not leave the node deaf: restore the permanent receive
                // session the aborted transmission interrupted.
                if self.runtime.permanent_receive {
                    self.start_receive(RxOptions {
                        delayed_time: None,
                        permanent: true,
                    })
                    .await?;
                }
                return Err(Error::DelayedSendTooLate);
            }
        }
        self.runtime.complete_transmit_session();
        Ok(())
    }

    /// Schedules a delayed TX/RX activation `delay` from the current system
    /// time.
    ///
    /// The returned value carries both the `DX_TIME` register value and the
    /// exact transmit timestamp the chip will report, so ranging payloads can
    /// embed the timestamp before the frame is sent.
    pub async fn schedule_delayed(
        &mut self,
        delay: DwTime,
    ) -> Result<DelayedTime, Error<SPI::Error, PinE>> {
        let now = self.read_system_timestamp().await?;
        Ok(self.runtime.schedule_delayed(now, delay))
    }

    /// Reads a received frame into `buffer`.
    pub async fn read_frame<'a>(
        &mut self,
        buffer: &'a mut [u8],
    ) -> Result<RxFrame<'a>, Error<SPI::Error, PinE>> {
        let status = self.read_sys_status().await?;
        if let Err(rx_error) = self.runtime.validate_rx_status(status) {
            if rx_error != RxError::FrameNotReady {
                // Errata: the receiver must be reset after any RX error or
                // timeout, or the next frame's timestamp may be wrong.
                self.reset_receiver().await?;
            }
            return Err(Error::Receive(rx_error));
        }

        let phy = self.runtime.phy.ok_or(Error::NotConfigured)?;
        let mut rx_finfo = [0u8; LEN_RX_FINFO];
        self.read_register(Register::RxFinfo, NO_SUBADDRESS, &mut rx_finfo)
            .await?;
        let needed = self.runtime.rx_payload_len(rx_finfo);
        if buffer.len() < needed {
            return Err(Error::BufferTooSmall {
                len: buffer.len(),
                needed,
            });
        }
        self.read_register(Register::RxBuffer, NO_SUBADDRESS, &mut buffer[..needed])
            .await?;
        let mut rx_time = [0u8; LEN_RX_TIME];
        self.read_register(Register::RxTime, RX_STAMP_SUB, &mut rx_time)
            .await?;
        let mut rx_fqual = [0u8; LEN_RX_FQUAL];
        self.read_register(Register::RxFqual, NO_SUBADDRESS, &mut rx_fqual)
            .await?;

        let (timestamp, metrics) = parse_rx_snapshot(phy, &rx_finfo, &rx_time, &rx_fqual);
        Ok(RxFrame {
            bytes: &buffer[..needed],
            timestamp: self
                .runtime
                .correct_receive_timestamp(timestamp, metrics.receive_power_dbm),
            metrics,
            status,
        })
    }

    /// Reads the latest timestamps.
    pub async fn read_timestamps(&mut self) -> Result<Timestamps, Error<SPI::Error, PinE>> {
        Ok(Timestamps {
            tx: self.read_transmit_timestamp().await?,
            rx: self.read_receive_timestamp().await?,
            system: self.read_system_timestamp().await?,
        })
    }

    /// Reads the current receive metrics.
    pub async fn read_signal_metrics(&mut self) -> Result<SignalMetrics, Error<SPI::Error, PinE>> {
        let phy = self.runtime.phy.ok_or(Error::NotConfigured)?;
        let mut rx_finfo = [0u8; LEN_RX_FINFO];
        self.read_register(Register::RxFinfo, NO_SUBADDRESS, &mut rx_finfo)
            .await?;
        let mut rx_time = [0u8; LEN_RX_TIME];
        self.read_register(Register::RxTime, RX_STAMP_SUB, &mut rx_time)
            .await?;
        let mut rx_fqual = [0u8; LEN_RX_FQUAL];
        self.read_register(Register::RxFqual, NO_SUBADDRESS, &mut rx_fqual)
            .await?;
        let (_, metrics) = parse_rx_snapshot(phy, &rx_finfo, &rx_time, &rx_fqual);
        Ok(metrics)
    }

    /// Reads `SYS_STATUS`.
    pub async fn read_sys_status(&mut self) -> Result<SysStatus, Error<SPI::Error, PinE>> {
        let mut bytes = [0u8; LEN_SYS_STATUS];
        self.read_register(Register::SysStatus, NO_SUBADDRESS, &mut bytes)
            .await?;
        Ok(sys_status_from_bytes(&bytes))
    }

    /// Clears the selected `SYS_STATUS` bits and re-arms permanent receive after RX/TX completion.
    pub async fn clear_events(
        &mut self,
        event_mask: SysStatus,
    ) -> Result<(), Error<SPI::Error, PinE>> {
        let should_rearm_after_good_receive =
            self.runtime.should_rearm_after_good_receive(event_mask);
        // Restarting the receiver issues TRXOFF, which would cancel a delayed
        // transmission scheduled after `event_mask` was read. Only restart
        // when the chip still reports a completed transmission; a pending
        // delayed TX re-arms the receiver from its own TX-done event instead.
        let mut should_restart_receive = self.runtime.should_restart_receive(event_mask);
        if should_restart_receive {
            let mut status_low = [0u8; 1];
            self.read_register(Register::SysStatus, NO_SUBADDRESS, &mut status_low)
                .await?;
            should_restart_receive = status_low[0] & TXFRS_LOW_BIT != 0;
        }
        self.write_register(
            Register::SysStatus,
            NO_SUBADDRESS,
            &sys_status_to_bytes(event_mask),
        )
        .await?;
        if should_restart_receive || should_rearm_after_good_receive {
            self.start_receive(RxOptions {
                delayed_time: None,
                permanent: true,
            })
            .await?;
        }
        Ok(())
    }

    /// Returns the currently configured identity, if the radio has been configured.
    pub const fn identity(&self) -> Option<DeviceIdentity> {
        self.runtime.identity()
    }

    /// Returns `true` if the IRQ pin is asserted.
    pub fn irq_asserted(&mut self) -> Result<bool, Error<SPI::Error, PinE>> {
        self.irq.is_high().map_err(Error::Pin)
    }

    /// Waits until the DW1000 IRQ line is asserted.
    pub async fn wait_for_irq(&mut self) -> Result<(), Error<SPI::Error, PinE>> {
        self.irq.wait_for_high().await.map_err(Error::Pin)
    }

    /// Enables RX and TX LED indicators on GPIO pins.
    /// GPIO2 will show RX activity and GPIO3 will show TX activity.
    pub async fn enable_leds(&mut self) -> Result<(), Error<SPI::Error, PinE>> {
        let mut pmsc_ctrl0 = [0u8; LEN_PMSC_CTRL0];
        self.read_register(Register::Pmsc, PMSC_CTRL0_SUB, &mut pmsc_ctrl0)
            .await?;
        compose_led_clock_enable(&mut pmsc_ctrl0);
        self.write_register(Register::Pmsc, PMSC_CTRL0_SUB, &pmsc_ctrl0)
            .await?;

        let mut pmsc_ledc = [0u8; LEN_PMSC_LEDC];
        self.read_register(Register::Pmsc, PMSC_LEDC_SUB, &mut pmsc_ledc)
            .await?;
        compose_led_blink_enable(&mut pmsc_ledc, 0x20);
        self.write_register(Register::Pmsc, PMSC_LEDC_SUB, &pmsc_ledc)
            .await?;

        let mut gpio_mode = [0u8; LEN_GPIO_MODE];
        self.read_register(Register::GpioCtrl, GPIO_MODE_SUB, &mut gpio_mode)
            .await?;
        compose_gpio_led_mode(&mut gpio_mode);
        self.write_register(Register::GpioCtrl, GPIO_MODE_SUB, &gpio_mode)
            .await?;
        Ok(())
    }

    async fn hard_reset(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error<SPI::Error, PinE>> {
        self.reset.set_low().map_err(Error::Pin)?;
        delay.delay_ms(2).await;
        self.reset.set_high().map_err(Error::Pin)?;
        delay.delay_ms(10).await;
        Ok(())
    }

    async fn verify_device_id(&mut self) -> Result<(), Error<SPI::Error, PinE>> {
        let mut bytes = [0u8; LEN_DEV_ID];
        self.read_register(Register::DevId, NO_SUBADDRESS, &mut bytes)
            .await?;
        let device_id = u32::from_le_bytes(bytes);
        if device_id != EXPECTED_DEVICE_ID {
            return Err(Error::InvalidDeviceId(device_id));
        }
        Ok(())
    }

    /// Loads the factory-calibrated LDO tune value from OTP, if programmed.
    async fn kick_ldo_tune(&mut self) -> Result<(), Error<SPI::Error, PinE>> {
        let ldo_tune = self.read_otp(OTP_ADDRESS_LDOTUNE).await?;
        if ldo_tune[0] != 0 {
            self.write_register(Register::OtpIf, OTP_SF_SUB, &[OTP_SF_LDO_KICK])
                .await?;
        }
        Ok(())
    }

    /// Resets the receiver after an RX error or timeout (official errata) and
    /// re-arms it when a permanent receive session is active.
    async fn reset_receiver(&mut self) -> Result<(), Error<SPI::Error, PinE>> {
        self.idle().await?;
        self.clear_receive_status().await?;
        self.write_register(Register::Pmsc, PMSC_SOFTRESET_SUB, &[PMSC_SOFTRESET_RX])
            .await?;
        self.write_register(Register::Pmsc, PMSC_SOFTRESET_SUB, &[PMSC_SOFTRESET_CLEAR])
            .await?;
        if self.runtime.permanent_receive {
            self.start_receive(RxOptions {
                delayed_time: None,
                permanent: true,
            })
            .await?;
        }
        Ok(())
    }

    async fn manage_lde(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error<SPI::Error, PinE>> {
        let mut pmsc_ctrl0 = [0u8; LEN_PMSC_CTRL0];
        self.read_register(Register::Pmsc, PMSC_CTRL0_SUB, &mut pmsc_ctrl0)
            .await?;
        let mut otp_ctrl = [0u8; LEN_OTP_CTRL];
        self.read_register(Register::OtpIf, OTP_CTRL_SUB, &mut otp_ctrl)
            .await?;
        set_lde_load_preamble(&mut pmsc_ctrl0, &mut otp_ctrl);
        self.write_register(Register::Pmsc, PMSC_CTRL0_SUB, &pmsc_ctrl0[..2])
            .await?;
        self.write_register(Register::OtpIf, OTP_CTRL_SUB, &otp_ctrl)
            .await?;
        delay.delay_ms(5).await;
        set_lde_restore_preamble(&mut pmsc_ctrl0);
        self.write_register(Register::Pmsc, PMSC_CTRL0_SUB, &pmsc_ctrl0[..2])
            .await?;
        Ok(())
    }

    async fn enable_clock(&mut self, mode: ClockMode) -> Result<(), Error<SPI::Error, PinE>> {
        let mut pmsc_ctrl0 = [0u8; LEN_PMSC_CTRL0];
        self.read_register(Register::Pmsc, PMSC_CTRL0_SUB, &mut pmsc_ctrl0)
            .await?;
        apply_clock_mode(&mut pmsc_ctrl0, mode);
        self.write_register(Register::Pmsc, PMSC_CTRL0_SUB, &pmsc_ctrl0[..2])
            .await
    }

    async fn read_otp(
        &mut self,
        address: u16,
    ) -> Result<[u8; LEN_OTP_RDAT], Error<SPI::Error, PinE>> {
        let mut address_bytes = [0u8; LEN_OTP_ADDR];
        address_bytes.copy_from_slice(&address.to_le_bytes());
        self.write_register(Register::OtpIf, OTP_ADDR_SUB, &address_bytes)
            .await?;
        // OTPRDEN | OTPREAD, then clear; OTPREAD self-latches the data.
        self.write_register(Register::OtpIf, OTP_CTRL_SUB, &[0x03, 0x00])
            .await?;
        self.write_register(Register::OtpIf, OTP_CTRL_SUB, &[0x00, 0x00])
            .await?;
        let mut data = [0u8; LEN_OTP_RDAT];
        self.read_register(Register::OtpIf, OTP_RDAT_SUB, &mut data)
            .await?;
        Ok(data)
    }

    async fn idle(&mut self) -> Result<(), Error<SPI::Error, PinE>> {
        self.sys_ctrl = [0; LEN_SYS_CTRL];
        prepare_idle_state(&mut self.runtime, &mut self.sys_ctrl);
        let sys_ctrl = self.sys_ctrl;
        self.write_register(Register::SysCtrl, NO_SUBADDRESS, &sys_ctrl)
            .await
    }

    async fn clear_interrupts(&mut self) -> Result<(), Error<SPI::Error, PinE>> {
        let sys_mask = cleared_interrupt_mask();
        self.write_register(Register::SysMask, NO_SUBADDRESS, &sys_mask)
            .await
    }

    async fn clear_receive_status(&mut self) -> Result<(), Error<SPI::Error, PinE>> {
        self.write_register(
            Register::SysStatus,
            NO_SUBADDRESS,
            &sys_status_to_bytes(receive_status_clear_mask()),
        )
        .await
    }

    async fn clear_transmit_status(&mut self) -> Result<(), Error<SPI::Error, PinE>> {
        self.write_register(
            Register::SysStatus,
            NO_SUBADDRESS,
            &sys_status_to_bytes(transmit_status_clear_mask()),
        )
        .await
    }

    async fn read_transmit_timestamp(&mut self) -> Result<DwTime, Error<SPI::Error, PinE>> {
        let mut bytes = [0u8; LEN_TX_STAMP];
        self.read_register(Register::TxTime, TX_STAMP_SUB, &mut bytes)
            .await?;
        Ok(DwTime::from_bytes(&bytes))
    }

    async fn read_receive_timestamp(&mut self) -> Result<DwTime, Error<SPI::Error, PinE>> {
        let mut bytes = [0u8; LEN_RX_STAMP];
        self.read_register(Register::RxTime, RX_STAMP_SUB, &mut bytes)
            .await?;
        Ok(DwTime::from_bytes(&bytes))
    }

    async fn read_system_timestamp(&mut self) -> Result<DwTime, Error<SPI::Error, PinE>> {
        let mut bytes = [0u8; LEN_RX_STAMP];
        self.read_register(Register::SysTime, NO_SUBADDRESS, &mut bytes)
            .await?;
        Ok(DwTime::from_bytes(&bytes))
    }

    async fn read_register(
        &mut self,
        register: Register,
        subaddress: u16,
        buffer: &mut [u8],
    ) -> Result<(), Error<SPI::Error, PinE>> {
        let header = build_header(register, subaddress, false);
        let header_len = header_len(subaddress);
        let mut operations = [
            Operation::Write(&header[..header_len]),
            Operation::Read(buffer),
        ];
        self.spi
            .transaction(&mut operations)
            .await
            .map_err(Error::Spi)
    }

    async fn write_register(
        &mut self,
        register: Register,
        subaddress: u16,
        data: &[u8],
    ) -> Result<(), Error<SPI::Error, PinE>> {
        let header = build_header(register, subaddress, true);
        let header_len = header_len(subaddress);
        let mut operations = [
            Operation::Write(&header[..header_len]),
            Operation::Write(data),
        ];
        self.spi
            .transaction(&mut operations)
            .await
            .map_err(Error::Spi)
    }
}
