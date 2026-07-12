# WirePIO

> TwoWire-compatible I2C implementation using PIO + DMA for the RP2040.

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform: RP2040](https://img.shields.io/badge/Platform-RP2040-green.svg)](https://www.raspberrypi.com/products/rp2040/)

## Overview

The RP2040 has only two hardware I2C controllers (I2C0 and I2C1). In large
projects this quickly becomes a limitation — many I2C devices have only two
possible addresses, requiring multiplexers like the TCA9548A.

**WirePIO** eliminates this limitation using the RP2040's PIO (Programmable I/O)
to create as many I2C buses as needed, with an API fully compatible with
Arduino's `Wire.h` (TwoWire).

## Features

- **Unlimited I2C buses** — any GPIO pin pair, independent operation
- **Drop-in Wire replacement** — `WirePIO bus(2,3); bus.begin();`
- **PIO + DMA** — zero CPU overhead during transfers
- **GPIO bit-bang fallback** — works even without PIO resources
- **Master + Slave** modes
- **Async DMA transfers** with callback
- **Timeout + NACK detection**
- **Compatible** with Adafruit_BME280, SSD1306, U8g2, RTClib, and more

## Requirements

- **Raspberry Pi Pico** or any **RP2040-based board**
- **Earle Philhower's arduino-pico core** (install via Boards Manager)
  - In Arduino IDE: File → Preferences → Additional Boards Manager URLs:
    `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
  - Then: Tools → Board → Boards Manager → search "Raspberry Pi Pico/RP2040"
- Arduino IDE 1.8.10+ or PlatformIO

> **Note:** This library uses the RP2040 PIO hardware and is NOT
> compatible with the Arduino Mbed OS RP2040 core.

## Quick Start

```cpp
#include <WirePIO.h>

// Create a bus on any GPIO pins
WirePIO bus(2, 3);  // SDA=GPIO2, SCL=GPIO3

void setup() {
    Serial.begin(115200);
    bus.begin();

    // Use exactly like Wire:
    bus.beginTransmission(0x76);
    bus.write(0xD0);         // Chip ID register
    bus.endTransmission(false);

    bus.requestFrom(0x76, 1);
    if (bus.available()) {
        Serial.println(bus.read(), HEX);
    }
}
```

## Multiple Buses

```cpp
WirePIO bus1(2, 3);   // Sensors
WirePIO bus2(4, 5);   // Display
WirePIO bus3(6, 7);   // ADC
WirePIO bus4(8, 9);   // GPIO expander
WirePIO bus5(10, 11); // EEPROM

// Each bus operates independently
bus1.begin();
bus2.begin();
// ...
```

## API

WirePIO implements the full `TwoWire` / `HardwareI2C` interface:

| Method | Description |
|---|---|
| `begin()` | Initialize as I2C master |
| `begin(uint8_t addr)` | Initialize as I2C slave |
| `end()` | Shut down and release resources |
| `setClock(freqHz)` | Set bus frequency (100000 or 400000) |
| `setSDA(pin)` / `setSCL(pin)` | Change pins (before begin()) |
| `setBufferSize(size)` | Set TX/RX buffer size (default 256) |
| `beginTransmission(addr)` | Start a master write transaction |
| `write(byte)` / `write(buf, len)` | Append data to TX buffer |
| `endTransmission([stop])` | Send TX buffer, return error code |
| `requestFrom(addr, n, [stop])` | Read n bytes from slave |
| `available()` | Bytes available in RX buffer |
| `read()` | Read one byte, -1 if empty |
| `peek()` | Peek next byte without consuming |
| `onReceive(callback)` | Slave receive callback |
| `onRequest(callback)` | Slave request callback |
| `scan()` | Scan bus and print addresses |

### Async API

| Method | Description |
|---|---|
| `writeAsync(addr, buf, len)` | Non-blocking write |
| `readAsync(addr, buf, len)` | Non-blocking read |
| `writeReadAsync(addr, wbuf, wlen, rbuf, rlen)` | Combined non-blocking |
| `finishedAsync()` | Check if async transfer done |
| `onFinishedAsync(callback)` | Completion callback |

### Error Codes

| Code | Meaning |
|---|---|
| 0 | Success |
| 1 | Data too long for buffer |
| 2 | NACK on address |
| 3 | NACK on data |
| 4 | Other error |
| 5 | Timeout |

## Architecture

```
Application
    │
├─── Adafruit_BME280
├─── Adafruit_SSD1306
├─── INA219
├─── ADS1115
│
    ▼
WirePIO (TwoWire-compatible API)
    │
    ├── WirePIOTransport
    │   ├── GPIO bit-bang (any pin pair)
    │   └── PIO + DMA (zero-CPU burst)
    │
    └── WirePIOSlave
        └── Hardware I2C peripheral
```

## Pin Mapping

- **SDA**: Any GPIO pin. The PIO drives it LOW actively, releases to Hi-Z for
  HIGH (open-drain emulation). External pull-up resistor (2.2k–4.7kΩ) required.
- **SCL**: Any GPIO pin. The PIO drives it actively (push-pull via side-set)
  for clean clock edges. No external pull-up strictly required for master mode,
  but recommended for multi-master or slave compatibility.

## Performance

- SCL frequency: ~65–87 kHz (conservative, spec-compliant timing)
- PIO clock divider: `sys_clk / (freq * 13)` — 13 PIO cycles per I2C bit
- DMA burst: 2 DMA channels per bus (TX → PIO, PIO → RX)
- CPU usage during transfer: 0% (DMA handles everything)

## Limitations

- PIO program: 31 instructions (fits within RP2040's 32-instruction limit)
- Max 8 PIO state machines per RP2040 → up to 8 I2C buses simultaneously
  (or fewer if other PIO programs are loaded)
- Slave mode uses hardware I2C peripheral (requires I2C-capable pins)
- No clock stretching in PIO master (deferred to v1.1 — saves 3 instructions)

## Installation

### Arduino IDE

1. Open **Library Manager** (Tools → Manage Libraries...)
2. Search for **"WirePIO"**
3. Click **Install**

> **Manual install:** Download this repository as ZIP and use
> Sketch → Include Library → Add .ZIP Library...

### PlatformIO

```ini
lib_deps = WirePIO
```

Or with a specific version:

```ini
lib_deps = angeloINTJ/TwoWirePIO_RP2040 @ ^1.3.1
```

### Pico SDK (CMake)

```cmake
add_subdirectory(path/to/TwoWirePIO_RP2040)
target_link_libraries(your_target WirePIO)
```

## License

MIT — see [LICENSE](LICENSE)

## Author

Angelo Moises Alves — [@angeloINTJ](https://github.com/angeloINTJ)
