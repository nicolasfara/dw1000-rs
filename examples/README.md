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
- Anchor short address: `3344`
- Tag short address: `3345`
- Antenna delay start value: `16_456`
- Operating mode: `OperatingMode::LongDataRangeAccuracy`

Tune the antenna delay constants in the two binaries against a known fixed distance on real hardware.
