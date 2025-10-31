use dw1000_rs::constants::*;
use dw1000_rs::DeviceMode;

#[test]
fn test_device_mode_enum() {
    assert_eq!(DeviceMode::Idle as u8, 0x00);
    assert_eq!(DeviceMode::Rx as u8, 0x01);
    assert_eq!(DeviceMode::Tx as u8, 0x02);
}

#[test]
fn test_device_mode_equality() {
    let mode1 = DeviceMode::Idle;
    let mode2 = DeviceMode::Idle;
    let mode3 = DeviceMode::Rx;

    assert_eq!(mode1, mode2);
    assert_ne!(mode1, mode3);
}

#[test]
fn test_device_mode_clone_copy() {
    let mode1 = DeviceMode::Tx;
    let mode2 = mode1; // Copy
    assert_eq!(mode1, mode2);

    let mode3 = mode1.clone();
    assert_eq!(mode1, mode3);
}

#[test]
fn test_register_addresses() {
    // Test that register addresses are unique
    assert_eq!(DEV_ID, 0x00);
    assert_eq!(EUI, 0x01);
    assert_eq!(PANADR, 0x03);
    assert_eq!(SYS_CFG, 0x04);
    assert_eq!(SYS_TIME, 0x06);
    assert_eq!(TX_FCTRL, 0x08);
    assert_eq!(TX_BUFFER, 0x09);
    assert_eq!(DX_TIME, 0x0A);
    assert_eq!(SYS_CTRL, 0x0D);
    assert_eq!(SYS_MASK, 0x0E);
    assert_eq!(SYS_STATUS, 0x0F);
    assert_eq!(RX_FINFO, 0x10);
    assert_eq!(RX_BUFFER, 0x11);
    assert_eq!(RX_FQUAL, 0x12);
    assert_eq!(RX_TIME, 0x15);
    assert_eq!(TX_TIME, 0x17);
    assert_eq!(TX_ANTD, 0x18);
    assert_eq!(TX_POWER, 0x1E);
    assert_eq!(CHAN_CTRL, 0x1F);
    assert_eq!(USR_SFD, 0x21);
    assert_eq!(AGC_TUNE, 0x23);
    assert_eq!(GPIO_CTRL, 0x26);
    assert_eq!(DRX_TUNE, 0x27);
    assert_eq!(RF_CONF, 0x28);
    assert_eq!(TX_CAL, 0x2A);
    assert_eq!(FS_CTRL, 0x2B);
    assert_eq!(AON, 0x2C);
    assert_eq!(OTP_IF, 0x2D);
    assert_eq!(LDE_IF, 0x2E);
    assert_eq!(PMSC, 0x36);
}

#[test]
fn test_register_lengths() {
    assert_eq!(LEN_DEV_ID, 4);
    assert_eq!(LEN_EUI, 8);
    assert_eq!(LEN_PANADR, 4);
    assert_eq!(LEN_SYS_CFG, 4);
    assert_eq!(LEN_SYS_CTRL, 4);
    assert_eq!(LEN_SYS_STATUS, 5);
    assert_eq!(LEN_SYS_MASK, 4);
    assert_eq!(LEN_TX_FCTRL, 5);
    assert_eq!(LEN_CHAN_CTRL, 4);
}

#[test]
fn test_timestamp_length() {
    assert_eq!(LEN_STAMP, 5); // 40-bit = 5 bytes
    assert_eq!(LEN_SYS_TIME, LEN_STAMP);
    assert_eq!(LEN_DX_TIME, LEN_STAMP);
    assert_eq!(LEN_RX_STAMP, LEN_STAMP);
    assert_eq!(LEN_TX_STAMP, LEN_STAMP);
}

#[test]
fn test_buffer_sizes() {
    assert_eq!(LEN_TX_BUFFER, 1024);
    assert_eq!(LEN_RX_BUFFER, 1024);
    assert_eq!(LEN_UWB_FRAMES, 127);
    assert_eq!(LEN_EXT_UWB_FRAMES, 1023);
}

#[test]
fn test_sys_cfg_bits() {
    assert_eq!(FFEN_BIT, 0);
    assert_eq!(FFBC_BIT, 1);
    assert_eq!(FFAB_BIT, 2);
    assert_eq!(FFAD_BIT, 3);
    assert_eq!(FFAA_BIT, 4);
    assert_eq!(FFAM_BIT, 5);
    assert_eq!(FFAR_BIT, 6);
    assert_eq!(HIRQ_POL_BIT, 9);
    assert_eq!(DIS_DRXB_BIT, 12);
    assert_eq!(PHR_MODE_SUB, 16);
    assert_eq!(DIS_STXP_BIT, 18);
    assert_eq!(RXM110K_BIT, 22);
    assert_eq!(RXAUTR_BIT, 29);
}

#[test]
fn test_sys_ctrl_bits() {
    assert_eq!(SFCST_BIT, 0);
    assert_eq!(TXSTRT_BIT, 1);
    assert_eq!(TXDLYS_BIT, 2);
    assert_eq!(TRXOFF_BIT, 6);
    assert_eq!(WAIT4RESP_BIT, 7);
    assert_eq!(RXENAB_BIT, 8);
    assert_eq!(RXDLYS_BIT, 9);
}

#[test]
fn test_sys_status_bits() {
    assert_eq!(CPLOCK_BIT, 1);
    assert_eq!(AAT_BIT, 3);
    assert_eq!(TXFRB_BIT, 4);
    assert_eq!(TXPRS_BIT, 5);
    assert_eq!(TXPHS_BIT, 6);
    assert_eq!(TXFRS_BIT, 7);
    assert_eq!(LDEDONE_BIT, 10);
    assert_eq!(RXPHE_BIT, 12);
    assert_eq!(RXDFR_BIT, 13);
    assert_eq!(RXFCG_BIT, 14);
    assert_eq!(RXFCE_BIT, 15);
    assert_eq!(RXRFSL_BIT, 16);
    assert_eq!(RXRFTO_BIT, 17);
    assert_eq!(LDEERR_BIT, 18);
    assert_eq!(RXPTO_BIT, 21);
    assert_eq!(RFPLL_LL_BIT, 24);
    assert_eq!(CLKPLL_LL_BIT, 25);
    assert_eq!(RXSFDTO_BIT, 26);
}

#[test]
fn test_special_constants() {
    assert_eq!(JUNK, 0x00);
    assert_eq!(NO_SUB, 0xFF);
}

#[test]
fn test_rx_time_sub_registers() {
    assert_eq!(LEN_RX_TIME, 14);
    assert_eq!(RX_STAMP_SUB, 0x00);
    assert_eq!(FP_AMPL1_SUB, 0x07);
    assert_eq!(LEN_FP_AMPL1, 2);
}

#[test]
fn test_rx_fqual_sub_registers() {
    assert_eq!(LEN_RX_FQUAL, 8);
    assert_eq!(STD_NOISE_SUB, 0x00);
    assert_eq!(FP_AMPL2_SUB, 0x02);
    assert_eq!(FP_AMPL3_SUB, 0x04);
    assert_eq!(CIR_PWR_SUB, 0x06);
    assert_eq!(LEN_STD_NOISE, 2);
    assert_eq!(LEN_FP_AMPL2, 2);
    assert_eq!(LEN_FP_AMPL3, 2);
    assert_eq!(LEN_CIR_PWR, 2);
}

#[test]
fn test_tx_time_register() {
    assert_eq!(LEN_TX_TIME, 10);
    assert_eq!(TX_STAMP_SUB, 0);
}

#[test]
fn test_channel_control_bits() {
    assert_eq!(DWSFD_BIT, 17);
    assert_eq!(TNSSFD_BIT, 20);
    assert_eq!(RNSSFD_BIT, 21);
}

#[test]
fn test_otp_constants() {
    assert_eq!(OTP_ADDR_SUB, 0x04);
    assert_eq!(OTP_CTRL_SUB, 0x06);
    assert_eq!(OTP_RDAT_SUB, 0x0A);
    assert_eq!(LEN_OTP_ADDR, 2);
    assert_eq!(LEN_OTP_CTRL, 2);
    assert_eq!(LEN_OTP_RDAT, 4);
}

#[test]
fn test_agc_tune_constants() {
    assert_eq!(AGC_TUNE1_SUB, 0x04);
    assert_eq!(AGC_TUNE2_SUB, 0x0C);
    assert_eq!(AGC_TUNE3_SUB, 0x12);
    assert_eq!(LEN_AGC_TUNE1, 2);
    assert_eq!(LEN_AGC_TUNE2, 4);
    assert_eq!(LEN_AGC_TUNE3, 2);
}

#[test]
fn test_drx_tune_constants() {
    assert_eq!(DRX_TUNE0B_SUB, 0x02);
    assert_eq!(DRX_TUNE1A_SUB, 0x04);
    assert_eq!(DRX_TUNE1B_SUB, 0x06);
    assert_eq!(DRX_TUNE2_SUB, 0x08);
    assert_eq!(DRX_TUNE4H_SUB, 0x26);
    assert_eq!(LEN_DRX_TUNE0B, 2);
    assert_eq!(LEN_DRX_TUNE1A, 2);
    assert_eq!(LEN_DRX_TUNE1B, 2);
    assert_eq!(LEN_DRX_TUNE2, 4);
    assert_eq!(LEN_DRX_TUNE4H, 2);
}

#[test]
fn test_lde_constants() {
    assert_eq!(LDE_CFG1_SUB, 0x0806);
    assert_eq!(LDE_RXANTD_SUB, 0x1804);
    assert_eq!(LDE_CFG2_SUB, 0x1806);
    assert_eq!(LDE_REPC_SUB, 0x2804);
    assert_eq!(LEN_LDE_CFG1, 1);
    assert_eq!(LEN_LDE_CFG2, 2);
    assert_eq!(LEN_LDE_REPC, 2);
    assert_eq!(LEN_LDE_RXANTD, 2);
}

#[test]
fn test_tx_power_constants() {
    assert_eq!(LEN_TX_POWER, 4);
}

#[test]
fn test_rf_conf_constants() {
    assert_eq!(RF_RXCTRLH_SUB, 0x0B);
    assert_eq!(RF_TXCTRL_SUB, 0x0C);
    assert_eq!(LEN_RF_RXCTRLH, 1);
    assert_eq!(LEN_RF_TXCTRL, 4);
}

#[test]
fn test_tx_cal_constants() {
    assert_eq!(TC_PGDELAY_SUB, 0x0B);
    assert_eq!(LEN_TC_PGDELAY, 1);
    assert_eq!(TC_SARC, 0x00);
    assert_eq!(TC_SARL, 0x03);
}

#[test]
fn test_fs_ctrl_constants() {
    assert_eq!(FS_PLLCFG_SUB, 0x07);
    assert_eq!(FS_PLLTUNE_SUB, 0x0B);
    assert_eq!(FS_XTALT_SUB, 0x0E);
    assert_eq!(LEN_FS_PLLCFG, 4);
    assert_eq!(LEN_FS_PLLTUNE, 1);
    assert_eq!(LEN_FS_XTALT, 1);
}

#[test]
fn test_aon_constants() {
    assert_eq!(AON_WCFG_SUB, 0x00);
    assert_eq!(LEN_AON_WCFG, 2);
    assert_eq!(ONW_LDC_BIT, 6);
    assert_eq!(ONW_LDD0_BIT, 12);
    assert_eq!(AON_CTRL_SUB, 0x02);
    assert_eq!(LEN_AON_CTRL, 1);
    assert_eq!(RESTORE_BIT, 0);
    assert_eq!(SAVE_BIT, 1);
    assert_eq!(UPL_CFG_BIT, 2);
    assert_eq!(AON_CFG0_SUB, 0x06);
    assert_eq!(LEN_AON_CFG0, 4);
    assert_eq!(SLEEP_EN_BIT, 0);
    assert_eq!(WAKE_PIN_BIT, 1);
    assert_eq!(WAKE_SPI_BIT, 2);
    assert_eq!(WAKE_CNT_BIT, 3);
}

#[test]
fn test_pmsc_constants() {
    assert_eq!(PMSC_CTRL0_SUB, 0x00);
    assert_eq!(PMSC_CTRL1_SUB, 0x04);
    assert_eq!(PMSC_LEDC_SUB, 0x28);
    assert_eq!(LEN_PMSC_CTRL0, 4);
    assert_eq!(LEN_PMSC_CTRL1, 4);
    assert_eq!(LEN_PMSC_LEDC, 4);
    assert_eq!(GPDCE_BIT, 18);
    assert_eq!(KHZCLKEN_BIT, 23);
    assert_eq!(BLNKEN, 8);
    assert_eq!(ATXSLP_BIT, 11);
    assert_eq!(ARXSLP_BIT, 12);
}

#[test]
fn test_tx_antd_constants() {
    assert_eq!(LEN_TX_ANTD, 2);
}

#[test]
fn test_gpio_constants() {
    assert_eq!(GPIO_MODE_SUB, 0x00);
    assert_eq!(LEN_GPIO_MODE, 4);
    assert_eq!(MSGP0, 6);
    assert_eq!(MSGP1, 8);
    assert_eq!(MSGP2, 10);
    assert_eq!(MSGP3, 12);
    assert_eq!(MSGP4, 14);
    assert_eq!(MSGP5, 16);
    assert_eq!(MSGP6, 18);
    assert_eq!(MSGP7, 20);
    assert_eq!(MSGP8, 22);
    assert_eq!(GPIO_MODE, 0);
    assert_eq!(LED_MODE, 1);
}

#[test]
fn test_usr_sfd_constants() {
    assert_eq!(LEN_USR_SFD, 41);
    assert_eq!(SFD_LENGTH_SUB, 0x00);
    assert_eq!(LEN_SFD_LENGTH, 1);
}

