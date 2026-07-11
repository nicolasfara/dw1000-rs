# Embassy STM32L432 Examples

The example firmware now lives in `examples/embassy-stm32l432`.

It contains one shared Embassy-based STM32L432 application with two binaries:

- `bin/anchor.rs`: anchor node selected entirely by compile-time configuration
- `bin/tag.rs`: tag node that initiates ranging and logs the measured distance to every discovered anchor

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
- Tag short address: `3345`
- Default anchor short address: `3344`
- Antenna delay start value: `16_456`
- Operating mode: `OperatingMode::LongDataRangeAccuracy`
- Maximum discovered anchors: `4`
- Base reply delay: `7 ms`; poll acknowledgements and range reports use staggered anchor reply slots assigned in the broadcast poll (`7`, `21`, `35`, `49 ms`)
- Discovery replies are staggered per anchor: the default slot is derived from the anchor short address (`7 ms * (1 + short % 8)`), so anchors that only differ by `DW1000_ANCHOR_SHORT` never answer the same blink in the same slot
- The tag uses an `80 ms` collection window per phase and ranges the anchors that acknowledged the poll even if another anchor missed its reply

Both binaries are configured at build time:

| Variable | Applies to | Default |
| --- | --- | --- |
| `DW1000_ANCHOR_PAN_ID` / `DW1000_TAG_PAN_ID` | anchor / tag | `0x0D57` |
| `DW1000_ANCHOR_SHORT` / `DW1000_TAG_SHORT` | anchor / tag | `3344` / `3345` |
| `DW1000_ANCHOR_EUI` / `DW1000_TAG_EUI` | anchor / tag | fixed example EUIs |
| `DW1000_ANCHOR_ANTENNA_DELAY` / `DW1000_TAG_ANTENNA_DELAY` | anchor / tag | `16_456` |
| `DW1000_ANCHOR_DISCOVERY_REPLY_DELAY_US` | anchor | `7 ms * (1 + short % 8)` |
| `DW1000_ANCHOR_COORDINATOR` | anchor | `false` |
| `DW1000_TAG_SLOT` | tag | `0` |
| `DW1000_TAG_SLOT_COUNT` | every node | `1` |
| `DW1000_TAG_SLOT_MS` | every node | `250` |

## Single tag, multiple anchors

No coordinator or master anchor is required: the tag schedules the whole exchange itself through the broadcast poll. Flash each anchor with a unique short address and EUI-64 (the discovery reply slot follows automatically), then flash the tag with its defaults:

```bash
DW1000_ANCHOR_SHORT=3344 DW1000_ANCHOR_EUI=B14A7C0011223344 cargo run --release --bin anchor
DW1000_ANCHOR_SHORT=3346 DW1000_ANCHOR_EUI=B14A7C0011223345 cargo run --release --bin anchor
DW1000_ANCHOR_SHORT=3347 DW1000_ANCHOR_EUI=B14A7C0011223346 cargo run --release --bin anchor
DW1000_ANCHOR_SHORT=3348 DW1000_ANCHOR_EUI=B14A7C0011223347 cargo run --release --bin anchor
cargo run --release --bin tag
```

## Multiple tags on the same PAN

Several tags share the air time through a TDMA frame of `DW1000_TAG_SLOT_COUNT` slots of `DW1000_TAG_SLOT_MS` each. The slot boundaries come from one coordinator anchor, so this setup requires all of the following:

- `DW1000_TAG_SLOT_COUNT` (and `DW1000_TAG_SLOT_MS`, if overridden) set to the same value on **every** node, anchors included
- exactly one anchor built with `DW1000_ANCHOR_COORDINATOR=1`
- a unique `DW1000_TAG_SLOT` (from `0` to `count - 1`), short address, and EUI-64 per tag

Example with two tags and four anchors:

```bash
DW1000_TAG_SLOT_COUNT=2 DW1000_ANCHOR_SHORT=3344 DW1000_ANCHOR_EUI=B14A7C0011223344 DW1000_ANCHOR_COORDINATOR=1 cargo run --release --bin anchor
DW1000_TAG_SLOT_COUNT=2 DW1000_ANCHOR_SHORT=3346 DW1000_ANCHOR_EUI=B14A7C0011223345 cargo run --release --bin anchor
DW1000_TAG_SLOT_COUNT=2 DW1000_ANCHOR_SHORT=3347 DW1000_ANCHOR_EUI=B14A7C0011223346 cargo run --release --bin anchor
DW1000_TAG_SLOT_COUNT=2 DW1000_ANCHOR_SHORT=3348 DW1000_ANCHOR_EUI=B14A7C0011223347 cargo run --release --bin anchor

DW1000_TAG_SLOT_COUNT=2 DW1000_TAG_SLOT=0 DW1000_TAG_SHORT=3345 DW1000_TAG_EUI=82175BD5A99AE29C cargo run --release --bin tag
DW1000_TAG_SLOT_COUNT=2 DW1000_TAG_SLOT=1 DW1000_TAG_SHORT=3350 DW1000_TAG_EUI=82175BD5A99AE29D cargo run --release --bin tag
```

How the multi-tag schedule behaves:

- tags stay **silent until they hear the coordinator's schedule broadcast** (within one or two `250 ms` sync periods), so an unsynced tag never tramples another tag's exchange — if a tag with `DW1000_TAG_SLOT_COUNT > 1` never transmits, check that a coordinator anchor is running
- a tag only starts an exchange when enough of its slot remains for both collection phases, so exchanges cannot spill into the next slot
- anchors keep per-tag exchange state, so overlapping exchanges from different tags at a slot boundary still complete instead of resetting each other
- each tag ranges all anchors once per TDMA frame (`count * slot_ms`, i.e. every `500 ms` in the example above)

Keep the PAN ID shared, but assign every physical tag and anchor a unique short address and EUI-64. Avoid anchor short addresses that are congruent modulo 8 (they would share a discovery slot — set `DW1000_ANCHOR_DISCOVERY_REPLY_DELAY_US` explicitly in that case). Tune each node's antenna delay against a known fixed distance on real hardware.
