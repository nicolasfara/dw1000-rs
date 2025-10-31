# DW1000 Rust Examples

This directory contains example applications demonstrating how to use the DW1000 Rust driver for UWB ranging.

## Examples

### Anchor Example (`anchor.rs`)

Demonstrates how to configure a DW1000 device as an anchor node. Anchors:
- Wait for blink messages from tags
- Respond to ranging requests
- Compute distances using asymmetric two-way ranging
- Report distance and signal quality metrics

**Usage:**
```bash
cargo build --example anchor --release
```

### Tag Example (`tag.rs`)

Demonstrates how to configure a DW1000 device as a tag node. Tags:
- Send blink messages to discover anchors
- Initiate ranging sequences with known anchors
- Receive distance measurements from anchors
- Display ranging results

**Usage:**
```bash
cargo build --example tag --release
```

## Hardware Requirements

Both examples assume an STM32F401RE microcontroller with the following connections:

### SPI Connections
- **MOSI**: PA7
- **MISO**: PA6
- **SCK**: PA5
- **CS**: PA4

### Other Connections
- **RST**: PA9 (DW1000 reset pin)
- **IRQ**: PA2 (DW1000 interrupt pin)

## Configuration

### Network Settings
- **Network ID**: 0xDECA (default PAN ID)
- **Operating Mode**: Long Data Range Low Power mode

### Addresses

**Anchor:**
- EUI-64: `82:17:5B:D5:A9:9A:E2:9C`
- Short Address: Derived from first 2 bytes

**Tag:**
- EUI-64: `7D:00:22:EA:82:60:3B:9C`
- Short Address: Derived from first 2 bytes

## Features

### Range Filtering
Both examples enable exponential moving average filtering with a factor of 15:
```rust
ranging.use_range_filter(true);
ranging.set_range_filter_value(15);
```

### Inactive Device Detection
Both examples periodically check for inactive devices and remove them from the network device list.

### Automatic Protocol Handling
The examples handle:
- Device discovery (blink messages)
- Ranging initialization
- Poll/Poll-ACK exchange
- Range computation
- Range reporting

## Customization

### Changing Addresses
Modify the address constants at the top of each example file:
```rust
const ANCHOR_ADDRESS: [u8; 8] = [0x82, 0x17, 0x5B, 0xD5, 0xA9, 0x9A, 0xE2, 0x9C];
const TAG_ADDRESS: [u8; 8] = [0x7D, 0x00, 0x22, 0xEA, 0x82, 0x60, 0x3B, 0x9C];
```

### Adjusting Timer Delays
The examples use an 80ms timer tick for periodic operations. Adjust in the main loop:
```rust
if timer_tick_counter >= ranging.timer_delay() {
    // Timer tick actions
}
```

### SPI Speed
The SPI clock is set to 2 MHz by default. Adjust in the SPI configuration:
```rust
spi_config.frequency = Hertz(2_000_000); // 2 MHz
```

## Expected Output

### Anchor
When running as an anchor, you should see:
```
DW1000 Anchor initialized
Address: 82:17:5B:D5:A9:9A:E2:9C
Short Address: 82:17
Waiting for tags...
New tag detected!
  Address: 7D:00:22:EA:82:60:3B:9C
  Short: 7D00
Range measurement:
  Device: 7D00
  Range: 2.45 m
  RX Power: -85.3 dBm
```

### Tag
When running as a tag, you should see:
```
DW1000 Tag initialized
Address: 7D:00:22:EA:82:60:3B:9C
Short Address: 7D:00
Looking for anchors...
Blink sent
New anchor detected!
  Short Address: 8217
Poll sent to 1 anchors
Range message sent
Range measurement:
  Anchor: 8217
  Range: 2.45 m
  RX Power: -85.3 dBm
```

## Troubleshooting

### No devices detected
- Verify SPI connections
- Check that both devices are on the same network ID
- Ensure DW1000 modules are powered correctly
- Verify interrupt pin is connected and configured

### Inaccurate ranging
- Calibrate antenna delays
- Enable range filtering
- Ensure clear line-of-sight between devices
- Check for interference

### Protocol failures
- Verify timing parameters (reply delays)
- Check message buffer sizes
- Enable debug logging with `defmt` feature

## Dependencies

The examples use Embassy for async/embedded operations:
- `embassy-stm32`: STM32 HAL
- `embassy-time`: Timing and delays
- `embassy-executor`: Async runtime
- `defmt`: Logging (optional)

## Building for Different Targets

To adapt these examples for different microcontrollers:

1. Change the HAL crate (e.g., `embassy-stm32` → `embassy-nrf`)
2. Update pin assignments
3. Adjust SPI configuration for your target
4. Update the linker script and target in `.cargo/config.toml`

## License

These examples are provided under the Apache-2.0 license, matching the main library.

