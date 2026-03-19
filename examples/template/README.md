# STM32L432KB Tag/Anchor Reference

This directory is a concrete STM32 reference project for the DW1000 tag/anchor examples.

It targets the STM32L432 board used by `/home/nicolas/Documents/repos/Project-Emerge/UWB-anchor-firmware/`
and keeps the main library crate board-neutral.

## Wiring

- `PA4`: DW1000 chip select
- `PA5`: DW1000 SPI SCK
- `PA6`: DW1000 SPI MISO
- `PA7`: DW1000 SPI MOSI
- `PA2`: DW1000 IRQ
- `PA1`: DW1000 reset
- `PB7`: STM6600 `POWER_INT`
- `PB5`: STM6600 `PS_HOLD`
- `PA10`: charger enable
- `PB0`: charger current select 1
- `PB1`: charger current select 2
- `PA8`: orange status LED
- `PA9`: green status LED
- `PA3`: battery sense input

## Run

1. Install the Cortex-M target: `rustup target add thumbv7em-none-eabihf`
2. Connect the STM32L432KB board and the DW1000 module with the wiring above
3. From this directory, flash the anchor image with `cargo run --bin anchor`
4. Flash the tag image with `cargo run --bin tag`

The local `.cargo/config.toml` already sets the default target to `thumbv7em-none-eabihf` and the
runner to `probe-rs run --chip STM32L432KBUx --speed 950`.

## Flash With probe-rs

If you prefer calling `probe-rs` directly instead of using the Cargo runner:

1. Build the anchor image with `cargo build --bin anchor --release`
2. Flash it with `probe-rs download --chip STM32L432KBUx --speed 950 target/thumbv7em-none-eabihf/release/anchor`
3. Build the tag image with `cargo build --bin tag --release`
4. Flash it with `probe-rs download --chip STM32L432KBUx --speed 950 target/thumbv7em-none-eabihf/release/tag`

To flash and start execution immediately, use `probe-rs run --chip STM32L432KBUx --speed 950 target/thumbv7em-none-eabihf/release/anchor`
or `probe-rs run --chip STM32L432KBUx --speed 950 target/thumbv7em-none-eabihf/release/tag`.

Use `--connect-under-reset` only if the board `NRST` pin is actually wired to the debug probe.
Without that connection, attach can fail before the firmware starts.

## Notes

- The reference PHY mode is `OperatingMode::LongDataRangeAccuracy`
- The tag computes and logs the measured distance to the anchor when it receives a range report
- The example exposes per-board antenna-delay calibration in
  `src/bin/anchor.rs` and `src/bin/tag.rs`
- The example uses a `3000 ms` peer inactivity timeout so occasional missed
  frames do not immediately drop a healthy anchor
- The example now mirrors the STM6600 bootstrap and charger wiring used by the reference firmware
- The anchor uses short address `3344`; the tag uses `3345`; both use PAN ID `0x0D57`
- The blocking example does not port the Embassy battery-monitor task; it only reserves the same pins
- The linker script uses the 64 KiB SRAM block at `0x2000_0000`; the extra 16 KiB SRAM2 bank at
  `0x1000_0000` is left unused by default

## Antenna Delay Calibration

The STM32 example keeps the working ranging code unchanged and exposes the
DW1000 calibration as two constants:

- `src/bin/anchor.rs`: `ANCHOR_ANTENNA_DELAY`
- `src/bin/tag.rs`: `TAG_ANTENNA_DELAY`

Both currently start from the reference value `16_456` ticks.

To calibrate on real hardware:

1. Place the tag and anchor at a known fixed distance.
2. Flash both examples and note the measured distance reported by the tag.
3. Adjust `ANCHOR_ANTENNA_DELAY` and `TAG_ANTENNA_DELAY` in small steps, rebuild,
   and reflash.
4. Repeat until the measured distance matches the known distance closely enough
   for your setup.

Because this driver currently uses one symmetric delay value per node, treat the
two constants as board-level calibration knobs rather than separate TX/RX
delays.

### Recommended Setup

Use a simple line-of-sight setup first:

1. Put one anchor and one tag on a table or tripod, not in your hand.
2. Keep the antennas facing the same way for every measurement.
3. Start with a known distance around `2 m` to `5 m`.
4. Measure the distance between the antenna reference points, not between the PCB edges.
5. Avoid calibrating next to large metal objects, walls, or your laptop.

Very short distances can be misleading because near-field effects and antenna
placement dominate the result. Start at a moderate distance, then validate again
at a second distance after tuning.

### Practical Procedure

The safest way to calibrate this example is to move both boards together first,
then split them only if needed.

1. Set `ANCHOR_ANTENNA_DELAY` and `TAG_ANTENNA_DELAY` to the same starting value.
2. Flash both boards and place them at a known distance.
3. Let the tag print multiple distance samples.
4. Record at least `20` to `50` readings and compute the average.
5. Compute the offset:
   `offset = measured_average - real_distance`
6. Change both constants by the same amount, rebuild, and reflash.
7. Repeat until the average error is small enough for your application.

Recommended step sizes:

- Start coarse with `128` or `256` ticks.
- When you are close, switch to `32` ticks.
- For final cleanup, use `8` or `16` ticks.

### Finding The Correct Direction

If you do not yet know whether the delay should go up or down on your hardware,
do one probe step:

1. Measure the average at the current value.
2. Increase both delays by `128` ticks.
3. Measure the new average.
4. Keep moving in whichever direction reduces the absolute error.

This avoids guessing the sign and is more reliable than relying on intuition
about the timestamp path.

### Equal Vs Per-Board Calibration

For one tag and one anchor:

- Keep both constants equal while doing the first calibration pass.
- If the result is good enough, stop there.

If you want tighter matching across multiple anchors:

1. Choose one tag as the reference.
2. Fix `TAG_ANTENNA_DELAY`.
3. Calibrate each anchor separately against the same tag.
4. Store a dedicated `ANCHOR_ANTENNA_DELAY` per anchor firmware image.

That is usually easier than trying to tune both sides independently every time.

### What To Look For

After calibration at one distance, verify at another distance:

1. Check again at roughly `1 m` to `2 m`.
2. Check again at roughly `5 m` or more if your space allows it.

Interpretation:

- If the error is roughly constant at all distances, antenna delay is the main issue.
- If the error changes a lot with distance or orientation, the problem is more likely multipath, antenna placement, or mechanical inconsistency.
- If readings are noisy, improve the environment before fine-tuning the constants.

### Suggested Workflow

For each calibration iteration:

1. Edit `src/bin/anchor.rs` and `src/bin/tag.rs`.
2. Rebuild:
   `cargo build --bin anchor --release`
   `cargo build --bin tag --release`
3. Reflash both boards.
4. Read the tag output and compute the average.
5. Repeat with a smaller step once close.

The example prints the active antenna delay during startup, so you can verify
that the flashed firmware matches the value you intended to test.
