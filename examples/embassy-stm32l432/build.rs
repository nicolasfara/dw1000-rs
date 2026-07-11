use std::{env, fs, path::PathBuf};

const DEFAULT_PAN_ID: u16 = 0x0D57;
const DEFAULT_SHORT_ADDRESS: u16 = 3344;
const DEFAULT_EUI: [u8; 8] = [0xB1, 0x4A, 0x7C, 0x00, 0x11, 0x22, 0x33, 0x44];
const DEFAULT_TAG_SHORT_ADDRESS: u16 = 3345;
const DEFAULT_TAG_EUI: [u8; 8] = [0x82, 0x17, 0x5B, 0xD5, 0xA9, 0x9A, 0xE2, 0x9C];
const DEFAULT_ANTENNA_DELAY: u16 = 16_456;
const DEFAULT_DISCOVERY_REPLY_DELAY_US: u16 = 7_000;
const DEFAULT_ANCHOR_COORDINATOR: bool = false;
const DEFAULT_TAG_SLOT: u16 = 0;
const DEFAULT_TAG_SLOT_COUNT: u16 = 1;
const DEFAULT_TAG_SLOT_MS: u16 = 250;
/// Twice the session timeout configured in `src/lib.rs`
/// (`RANGING_SESSION_TIMEOUT_MS`): the minimum slot budget one exchange needs.
const MIN_SHARED_SLOT_MS: u16 = 160;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR not set"));
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).expect("write memory.x");
    fs::write(out.join("anchor_config.rs"), anchor_config()).expect("write anchor config");
    fs::write(out.join("tag_config.rs"), tag_config()).expect("write tag config");
    fs::write(out.join("schedule_config.rs"), schedule_config()).expect("write schedule config");
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    for variable in [
        "DW1000_ANCHOR_PAN_ID",
        "DW1000_ANCHOR_SHORT",
        "DW1000_ANCHOR_EUI",
        "DW1000_ANCHOR_ANTENNA_DELAY",
        "DW1000_ANCHOR_DISCOVERY_REPLY_DELAY_US",
        "DW1000_ANCHOR_COORDINATOR",
        "DW1000_TAG_PAN_ID",
        "DW1000_TAG_SHORT",
        "DW1000_TAG_EUI",
        "DW1000_TAG_ANTENNA_DELAY",
        "DW1000_TAG_SLOT",
        "DW1000_TAG_SLOT_COUNT",
        "DW1000_TAG_SLOT_MS",
    ] {
        println!("cargo:rerun-if-env-changed={variable}");
    }
}

fn anchor_config() -> String {
    let pan_id = env_u16("DW1000_ANCHOR_PAN_ID", DEFAULT_PAN_ID);
    let short_address = env_u16("DW1000_ANCHOR_SHORT", DEFAULT_SHORT_ADDRESS);
    let eui = env_eui("DW1000_ANCHOR_EUI", DEFAULT_EUI);
    let antenna_delay = env_u16("DW1000_ANCHOR_ANTENNA_DELAY", DEFAULT_ANTENNA_DELAY);
    let discovery_reply_delay_us = env_u16(
        "DW1000_ANCHOR_DISCOVERY_REPLY_DELAY_US",
        default_discovery_reply_delay_us(short_address),
    );
    let anchor_is_coordinator = env_bool("DW1000_ANCHOR_COORDINATOR", DEFAULT_ANCHOR_COORDINATOR);

    assert_ne!(
        short_address, 0xFFFF,
        "DW1000_ANCHOR_SHORT must not be the broadcast address"
    );

    format!(
        "const ANCHOR_CONFIG: NodeConfig = NodeConfig {{\n\
             identity: DeviceIdentity::new(\n\
                 PanId::new({pan_id}),\n\
                 ShortAddress::new({short_address}),\n\
                 Eui64::new([{eui}]),\n\
             ),\n\
             antenna_delay: AntennaDelay::new({antenna_delay}),\n\
             discovery_reply_delay_us: {discovery_reply_delay_us},\n\
             anchor_is_coordinator: {anchor_is_coordinator},\n\
             tag_slot: 0,\n\
         }};\n",
        eui = format_eui(&eui),
    )
}

fn tag_config() -> String {
    let pan_id = env_u16("DW1000_TAG_PAN_ID", DEFAULT_PAN_ID);
    let short_address = env_u16("DW1000_TAG_SHORT", DEFAULT_TAG_SHORT_ADDRESS);
    let eui = env_eui("DW1000_TAG_EUI", DEFAULT_TAG_EUI);
    let antenna_delay = env_u16("DW1000_TAG_ANTENNA_DELAY", DEFAULT_ANTENNA_DELAY);
    let tag_slot = env_u16("DW1000_TAG_SLOT", DEFAULT_TAG_SLOT);
    let tag_slot_count = env_u16("DW1000_TAG_SLOT_COUNT", DEFAULT_TAG_SLOT_COUNT);

    assert_ne!(
        short_address, 0xFFFF,
        "DW1000_TAG_SHORT must not be the broadcast address"
    );
    assert!(
        tag_slot < tag_slot_count,
        "DW1000_TAG_SLOT must be lower than DW1000_TAG_SLOT_COUNT"
    );

    format!(
        "const TAG_CONFIG: NodeConfig = NodeConfig {{\n\
             identity: DeviceIdentity::new(\n\
                 PanId::new({pan_id}),\n\
                 ShortAddress::new({short_address}),\n\
                 Eui64::new([{eui}]),\n\
             ),\n\
             antenna_delay: AntennaDelay::new({antenna_delay}),\n\
             discovery_reply_delay_us: 0,\n\
             anchor_is_coordinator: false,\n\
             tag_slot: {tag_slot},\n\
         }};\n",
        eui = format_eui(&eui),
    )
}

/// TDMA schedule shared by every node on the PAN. Tags use it to pick their
/// transmit windows; the coordinator anchor broadcasts it, so build all
/// binaries with the same `DW1000_TAG_SLOT_COUNT` and `DW1000_TAG_SLOT_MS`.
fn schedule_config() -> String {
    let tag_slot_count = env_u16("DW1000_TAG_SLOT_COUNT", DEFAULT_TAG_SLOT_COUNT);
    let tag_slot_ms = env_u16("DW1000_TAG_SLOT_MS", DEFAULT_TAG_SLOT_MS);

    assert!(
        (1..=255).contains(&tag_slot_count),
        "DW1000_TAG_SLOT_COUNT must be between 1 and 255"
    );
    assert!(
        tag_slot_count == 1 || tag_slot_ms >= MIN_SHARED_SLOT_MS,
        "DW1000_TAG_SLOT_MS must be at least {MIN_SHARED_SLOT_MS} ms when several tags share the PAN"
    );

    format!(
        "const TAG_SLOT_COUNT: u8 = {tag_slot_count};\n\
         const TAG_SLOT_MS: u32 = {tag_slot_ms};\n"
    )
}

fn format_eui(eui: &[u8; 8]) -> String {
    eui.iter()
        .map(|byte| format!("0x{byte:02X}"))
        .collect::<Vec<_>>()
        .join(", ")
}

/// Staggers the discovery reply slot by short address so anchors flashed with
/// only a distinct `DW1000_ANCHOR_SHORT` never answer the same blink in the
/// same slot. Anchors whose short addresses are congruent modulo 8 would
/// still collide; set `DW1000_ANCHOR_DISCOVERY_REPLY_DELAY_US` explicitly for
/// those.
fn default_discovery_reply_delay_us(short_address: u16) -> u16 {
    DEFAULT_DISCOVERY_REPLY_DELAY_US * (1 + short_address % 8)
}

fn env_bool(name: &str, default: bool) -> bool {
    let Some(raw) = env::var_os(name) else {
        return default;
    };
    match raw
        .to_str()
        .unwrap_or_else(|| panic!("{name} must be valid UTF-8"))
        .trim()
    {
        "1" | "true" | "TRUE" | "yes" | "YES" | "on" | "ON" => true,
        "0" | "false" | "FALSE" | "no" | "NO" | "off" | "OFF" => false,
        value => panic!("{name} must be boolean, got {value:?}"),
    }
}

fn env_u16(name: &str, default: u16) -> u16 {
    let Some(raw) = env::var_os(name) else {
        return default;
    };
    let raw = raw
        .to_str()
        .unwrap_or_else(|| panic!("{name} must be valid UTF-8"));
    let value = raw.trim();
    let value = value
        .strip_prefix("0x")
        .or_else(|| value.strip_prefix("0X"))
        .map_or_else(|| value.parse(), |hex| u16::from_str_radix(hex, 16))
        .unwrap_or_else(|_| panic!("{name} must be a u16, got {raw:?}"));
    value
}

fn env_eui(name: &str, default: [u8; 8]) -> [u8; 8] {
    let Some(raw) = env::var_os(name) else {
        return default;
    };
    let raw = raw
        .to_str()
        .unwrap_or_else(|| panic!("{name} must be valid UTF-8"))
        .trim();
    let raw = raw
        .strip_prefix("0x")
        .or_else(|| raw.strip_prefix("0X"))
        .unwrap_or(raw);
    let mut digits = String::with_capacity(16);
    for character in raw.chars() {
        if character.is_ascii_hexdigit() {
            digits.push(character);
        } else if !matches!(character, ':' | '-' | '_' | ' ') {
            panic!("{name} contains an invalid character");
        }
    }
    assert_eq!(digits.len(), 16, "{name} must contain exactly 8 bytes");

    let mut eui = [0; 8];
    for (index, byte) in eui.iter_mut().enumerate() {
        let offset = index * 2;
        *byte = u8::from_str_radix(&digits[offset..offset + 2], 16)
            .unwrap_or_else(|_| panic!("{name} must contain hexadecimal bytes"));
    }
    eui
}
