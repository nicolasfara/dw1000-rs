use std::{env, fs, path::PathBuf};

const DEFAULT_ANCHOR_PAN_ID: u16 = 0x0D57;
const DEFAULT_ANCHOR_SHORT_ADDRESS: u16 = 3344;
const DEFAULT_ANCHOR_EUI: [u8; 8] = [0xB1, 0x4A, 0x7C, 0x00, 0x11, 0x22, 0x33, 0x44];
const DEFAULT_ANCHOR_SLOT: u8 = 0;
const DEFAULT_ANCHOR_COORDINATOR: bool = false;
const DEFAULT_TAG_PAN_ID: u16 = 0x0D57;
const DEFAULT_TAG_SHORT_ADDRESS: u16 = 3400;
const DEFAULT_TAG_EUI: [u8; 8] = [0x82, 0x17, 0x5B, 0xD5, 0xA9, 0x9A, 0xE2, 0x9C];
const DEFAULT_TAG_SLOT: u8 = 0;
const DEFAULT_TAG_SLOT_COUNT: u8 = 2;
const DEFAULT_TAG_SLOT_MS: u32 = 250;
const DEFAULT_DISCOVERY_SLOT_SPACING_US: u32 = 12_000;
const DEFAULT_SESSION_TIMEOUT_MS: u32 = 180;
const DEFAULT_RANGE_PERIOD_MS: u32 = 260;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR not set"));
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).expect("write memory.x");
    fs::write(out.join("anchor_identity.rs"), anchor_identity()).expect("write anchor_identity.rs");
    fs::write(out.join("tag_identity.rs"), tag_identity()).expect("write tag_identity.rs");
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-env-changed=DW1000_ANCHOR_PAN_ID");
    println!("cargo:rerun-if-env-changed=DW1000_ANCHOR_SHORT");
    println!("cargo:rerun-if-env-changed=DW1000_ANCHOR_EUI");
    println!("cargo:rerun-if-env-changed=DW1000_ANCHOR_SLOT");
    println!("cargo:rerun-if-env-changed=DW1000_ANCHOR_COORDINATOR");
    println!("cargo:rerun-if-env-changed=DW1000_TAG_PAN_ID");
    println!("cargo:rerun-if-env-changed=DW1000_TAG_SHORT");
    println!("cargo:rerun-if-env-changed=DW1000_TAG_EUI");
    println!("cargo:rerun-if-env-changed=DW1000_TAG_SLOT");
    println!("cargo:rerun-if-env-changed=DW1000_TAG_SLOT_COUNT");
    println!("cargo:rerun-if-env-changed=DW1000_TAG_SLOT_MS");
    println!("cargo:rerun-if-env-changed=DW1000_DISCOVERY_SLOT_SPACING_US");
    println!("cargo:rerun-if-env-changed=DW1000_SESSION_TIMEOUT_MS");
    println!("cargo:rerun-if-env-changed=DW1000_RANGE_PERIOD_MS");
}

fn anchor_identity() -> String {
    let pan_id = env_u16("DW1000_ANCHOR_PAN_ID", DEFAULT_ANCHOR_PAN_ID);
    let short_address = env_u16("DW1000_ANCHOR_SHORT", DEFAULT_ANCHOR_SHORT_ADDRESS);
    let eui = env_eui("DW1000_ANCHOR_EUI", DEFAULT_ANCHOR_EUI);
    let anchor_slot = env_u8("DW1000_ANCHOR_SLOT", DEFAULT_ANCHOR_SLOT);
    let anchor_coordinator = env_bool(
        "DW1000_ANCHOR_COORDINATOR",
        DEFAULT_ANCHOR_COORDINATOR,
    );
    let tag_slot_count = env_u8("DW1000_TAG_SLOT_COUNT", DEFAULT_TAG_SLOT_COUNT);
    let tag_slot_ms = env_u32("DW1000_TAG_SLOT_MS", DEFAULT_TAG_SLOT_MS);
    let discovery_slot_spacing_us = env_u32(
        "DW1000_DISCOVERY_SLOT_SPACING_US",
        DEFAULT_DISCOVERY_SLOT_SPACING_US,
    );
    let session_timeout_ms = env_u32("DW1000_SESSION_TIMEOUT_MS", DEFAULT_SESSION_TIMEOUT_MS);
    let range_period_ms = env_u32("DW1000_RANGE_PERIOD_MS", DEFAULT_RANGE_PERIOD_MS);

    if short_address == 0xFFFF {
        panic!("DW1000_ANCHOR_SHORT must not be the broadcast address 0xFFFF");
    }
    if tag_slot_count == 0 {
        panic!("DW1000_TAG_SLOT_COUNT must be at least 1");
    }

    format!(
        "const ANCHOR_PAN_ID: u16 = {pan_id};\n\
         const ANCHOR_SHORT_ADDRESS: u16 = {short_address};\n\
         const ANCHOR_EUI: [u8; 8] = [{eui}];\n\
         const ANCHOR_SLOT: u8 = {anchor_slot};\n\
         const ANCHOR_COORDINATOR: bool = {anchor_coordinator};\n\
         const TAG_SLOT_COUNT: u8 = {tag_slot_count};\n\
         const TAG_SLOT_MS: u32 = {tag_slot_ms};\n\
         const DISCOVERY_SLOT_SPACING_US: u32 = {discovery_slot_spacing_us};\n\
         const SESSION_TIMEOUT_MS: u32 = {session_timeout_ms};\n\
         const RANGE_PERIOD_MS: u32 = {range_period_ms};\n",
        eui = eui
            .iter()
            .map(|byte| format!("0x{byte:02X}"))
            .collect::<Vec<_>>()
            .join(", ")
    )
}

fn tag_identity() -> String {
    let pan_id = env_u16("DW1000_TAG_PAN_ID", DEFAULT_TAG_PAN_ID);
    let short_address = env_u16("DW1000_TAG_SHORT", DEFAULT_TAG_SHORT_ADDRESS);
    let eui = env_eui("DW1000_TAG_EUI", DEFAULT_TAG_EUI);
    let tag_slot = env_u8("DW1000_TAG_SLOT", DEFAULT_TAG_SLOT);
    let tag_slot_count = env_u8("DW1000_TAG_SLOT_COUNT", DEFAULT_TAG_SLOT_COUNT);
    let tag_slot_ms = env_u32("DW1000_TAG_SLOT_MS", DEFAULT_TAG_SLOT_MS);
    let discovery_slot_spacing_us = env_u32(
        "DW1000_DISCOVERY_SLOT_SPACING_US",
        DEFAULT_DISCOVERY_SLOT_SPACING_US,
    );
    let session_timeout_ms = env_u32("DW1000_SESSION_TIMEOUT_MS", DEFAULT_SESSION_TIMEOUT_MS);
    let range_period_ms = env_u32("DW1000_RANGE_PERIOD_MS", DEFAULT_RANGE_PERIOD_MS);

    if short_address == 0xFFFF {
        panic!("DW1000_TAG_SHORT must not be the broadcast address 0xFFFF");
    }
    if tag_slot_count == 0 {
        panic!("DW1000_TAG_SLOT_COUNT must be at least 1");
    }
    if tag_slot >= tag_slot_count {
        panic!("DW1000_TAG_SLOT must be less than DW1000_TAG_SLOT_COUNT");
    }

    format!(
        "const TAG_PAN_ID: u16 = {pan_id};\n\
         const TAG_SHORT_ADDRESS: u16 = {short_address};\n\
         const TAG_EUI: [u8; 8] = [{eui}];\n\
         const TAG_SLOT: u8 = {tag_slot};\n\
         const TAG_SLOT_COUNT: u8 = {tag_slot_count};\n\
         const TAG_SLOT_MS: u32 = {tag_slot_ms};\n\
         const DISCOVERY_SLOT_SPACING_US: u32 = {discovery_slot_spacing_us};\n\
         const SESSION_TIMEOUT_MS: u32 = {session_timeout_ms};\n\
         const RANGE_PERIOD_MS: u32 = {range_period_ms};\n",
        eui = eui
            .iter()
            .map(|byte| format!("0x{byte:02X}"))
            .collect::<Vec<_>>()
            .join(", ")
    )
}

fn env_u16(name: &str, default: u16) -> u16 {
    let Some(raw) = env::var_os(name) else {
        return default;
    };
    let raw = raw
        .to_str()
        .unwrap_or_else(|| panic!("{name} must be valid UTF-8"));
    parse_u16(raw).unwrap_or_else(|| panic!("{name} must be a u16, got {raw:?}"))
}

fn env_u8(name: &str, default: u8) -> u8 {
    let Some(raw) = env::var_os(name) else {
        return default;
    };
    let raw = raw
        .to_str()
        .unwrap_or_else(|| panic!("{name} must be valid UTF-8"));
    parse_u16(raw)
        .and_then(|value| u8::try_from(value).ok())
        .unwrap_or_else(|| panic!("{name} must be a u8, got {raw:?}"))
}

fn env_u32(name: &str, default: u32) -> u32 {
    let Some(raw) = env::var_os(name) else {
        return default;
    };
    let raw = raw
        .to_str()
        .unwrap_or_else(|| panic!("{name} must be valid UTF-8"));
    parse_u32(raw).unwrap_or_else(|| panic!("{name} must be a u32, got {raw:?}"))
}

fn env_bool(name: &str, default: bool) -> bool {
    let Some(raw) = env::var_os(name) else {
        return default;
    };
    let raw = raw
        .to_str()
        .unwrap_or_else(|| panic!("{name} must be valid UTF-8"))
        .trim();
    match raw {
        "1" | "true" | "TRUE" | "yes" | "YES" | "on" | "ON" => true,
        "0" | "false" | "FALSE" | "no" | "NO" | "off" | "OFF" => false,
        _ => panic!("{name} must be boolean, got {raw:?}"),
    }
}

fn parse_u16(raw: &str) -> Option<u16> {
    let raw = raw.trim();
    if raw.is_empty() {
        return None;
    }
    if let Some(hex) = raw.strip_prefix("0x").or_else(|| raw.strip_prefix("0X")) {
        return u16::from_str_radix(hex, 16).ok();
    }
    raw.parse().ok()
}

fn parse_u32(raw: &str) -> Option<u32> {
    let raw = raw.trim();
    if raw.is_empty() {
        return None;
    }
    if let Some(hex) = raw.strip_prefix("0x").or_else(|| raw.strip_prefix("0X")) {
        return u32::from_str_radix(hex, 16).ok();
    }
    raw.parse().ok()
}

fn env_eui(name: &str, default: [u8; 8]) -> [u8; 8] {
    let Some(raw) = env::var_os(name) else {
        return default;
    };
    let raw = raw
        .to_str()
        .unwrap_or_else(|| panic!("{name} must be valid UTF-8"));
    parse_eui(raw).unwrap_or_else(|| {
        panic!("{name} must contain exactly 8 hex bytes, got {raw:?}");
    })
}

fn parse_eui(raw: &str) -> Option<[u8; 8]> {
    let mut hex = String::with_capacity(16);
    for ch in raw.trim().chars() {
        if matches!(ch, ':' | '-' | '_' | ' ') {
            continue;
        }
        hex.push(ch);
    }
    let hex = hex
        .strip_prefix("0x")
        .or_else(|| hex.strip_prefix("0X"))
        .unwrap_or(&hex);
    if hex.len() != 16 || !hex.as_bytes().iter().all(u8::is_ascii_hexdigit) {
        return None;
    }

    let mut eui = [0u8; 8];
    for (index, byte) in eui.iter_mut().enumerate() {
        let start = index * 2;
        *byte = u8::from_str_radix(&hex[start..start + 2], 16).ok()?;
    }
    Some(eui)
}
