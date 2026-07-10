# Embassy STM32L432 Examples

The example firmware now lives in `examples/embassy-stm32l432`.

It contains one shared Embassy-based STM32L432 application with two binaries:

- `bin/anchor.rs`: anchor node that participates in discovery and responds to ranging
- `bin/tag.rs`: tag node that initiates ranging and logs the measured distance to the anchor

## Target Hardware

The firmware matches the STM32L432 reference board and DW1000 wiring already used in the project:

- `PA4`: DW1000 CS
- `PA5`: DW1000 SCK
- `PA6`: DW1000 MISO
- `PA7`: DW1000 MOSI
- `PA2`: DW1000 IRQ
- `PA1`: DW1000 RSTn, kept open-drain
- `PB7`: STM6600 `POWER_INT`
- `PB5`: STM6600 `PS_HOLD`
- `PA10`: charger enable
- `PB0`: charger current select 1
- `PB1`: charger current select 2
- `PA8`: orange status LED
- `PA9`: green status LED

## Build And Run

From the example crate directory:

```bash
cargo run --release --bin anchor
cargo run --release --bin tag
```

Or from the repo root:

```bash
cargo run --release --config examples/embassy-stm32l432/.cargo/config.toml --manifest-path examples/embassy-stm32l432/Cargo.toml --bin anchor
cargo run --release --config examples/embassy-stm32l432/.cargo/config.toml --manifest-path examples/embassy-stm32l432/Cargo.toml --bin tag
```

The example crate has its own `.cargo/config.toml` with:

- target: `thumbv7em-none-eabihf`
- runner: `probe-rs run --chip STM32L432KBUx --speed 950`

When invoking Cargo from the repo root with `--manifest-path`, Cargo does not automatically use the nested example crate's `.cargo/config.toml`, so the root command must also pass `--config .../.cargo/config.toml`.

Use `--release` for flashing/running. The `dev` profile does not fit in the STM32L432 128 KiB flash.

## Defaults

- PAN ID: `0x0D57`
- Anchor short addresses: `3344..3347`
- Tag short addresses: `3400..3401`
- Antenna delay start value: `16_456`
- Operating mode: `OperatingMode::LongDataRangeAccuracy`
- Discovery response slot spacing: `12_000 us`

Anchor and tag identity can be overridden at compile time with environment variables. Flash four anchors with distinct `DW1000_ANCHOR_SLOT` values and two tags with distinct `DW1000_TAG_SLOT` values:

```bash
DW1000_ANCHOR_SHORT=3344 \
DW1000_ANCHOR_EUI=B14A7C0011223344 \
DW1000_ANCHOR_SLOT=0 \
DW1000_ANCHOR_COORDINATOR=1 \
cargo run --release --bin anchor

DW1000_ANCHOR_SHORT=3345 \
DW1000_ANCHOR_EUI=B14A7C0011223345 \
DW1000_ANCHOR_SLOT=1 \
cargo run --release --bin anchor

DW1000_TAG_SHORT=3400 \
DW1000_TAG_EUI=82175BD5A99AE29C \
DW1000_TAG_SLOT=0 \
cargo run --release --bin tag

DW1000_TAG_SHORT=3401 \
DW1000_TAG_EUI=82175BD5A99AE29D \
DW1000_TAG_SLOT=1 \
cargo run --release --bin tag
```

`DW1000_ANCHOR_SHORT`, `DW1000_TAG_SHORT`, and PAN IDs accept decimal or `0x` hexadecimal. EUI variables accept 16 hex digits with optional `:`, `-`, `_`, or space separators. Shared schedule knobs are `DW1000_TAG_SLOT_COUNT`, `DW1000_TAG_SLOT_MS`, `DW1000_DISCOVERY_SLOT_SPACING_US`, `DW1000_SESSION_TIMEOUT_MS`, and `DW1000_RANGE_PERIOD_MS`.

Tags log structured range measurements:

```text
uwb_range tag=3400 anchor=3344 range_m=1.234 quality=1.500
```

The host dashboard lives in `tools/uwb-dashboard`:

```bash
cargo run --manifest-path tools/uwb-dashboard/Cargo.toml -- --config tools/uwb-dashboard/anchors.toml --bind 127.0.0.1:8080
```

It accepts range samples with `POST /api/ranges`, exposes `GET /api/state`, and broadcasts state updates on `GET /ws`.

Tune the antenna delay constants in the two binaries against a known fixed distance on real hardware.
