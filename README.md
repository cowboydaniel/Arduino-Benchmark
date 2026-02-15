# Arduino Universal Benchmark Suite

A comprehensive benchmarking framework for comparing performance across Arduino-compatible boards. Upload a sketch, open Serial Monitor, and get detailed metrics — on any board from a classic Uno to an ESP32-S3.

## Supported Boards (30+)

| Family | Boards |
|---|---|
| **ESP** | ESP32 (all variants: C3, S2, S3, C6, H2), ESP8266 |
| **Arduino Classic** | Uno, Nano, Mega, Leonardo, Micro, Pro, Mini |
| **Arduino Modern** | Uno R4 WiFi/Minima, Portenta H7/C33, Giga, Due, Zero |
| **Arduino IoT** | Nano 33 IoT, MKR WiFi1010, MKR FOX/WAN/GSM/NB, Vidor 4000 |
| **RP2040** | Raspberry Pi Pico, Pico W, Nano RP2040 Connect, XIAO RP2040 |
| **nRF52** | Nano 33 BLE |
| **STM32** | Blue Pill, Black Pill, Nucleo, Uno Q (STM32U585) |
| **Teensy** | 3.x, 4.0, 4.1 |
| **Other** | Adafruit Feather, Seeed XIAO, Arduino Yun, Multiduino, INK4u |

Board detection is automatic via compiler macros — no manual configuration needed.

## Repository Structure

```
├── Part1_CoreBenchmarks/
│   ├── Part1_CoreBenchmarks.ino    # Core performance tests
│   └── BenchmarkHelpers.h
├── Part2_PlatformBenchmarks/
│   ├── Part2_PlatformBenchmarks.ino # Platform-specific tests
│   └── BenchmarkHelpers.h
├── BenchmarkHelpers.h               # Shared timing utilities
├── Arduino_Benchmark_Comparison.xlsx # Results spreadsheet
└── CONTRIBUTING.md                   # Guide for contributors
```

## What Gets Benchmarked

### Part 1 — Core Benchmarks

Tests that run on every board:

- **CPU:** Integer ops, floating-point ops, string ops, 64-bit math, DSP-enhanced operations (if available)
- **Memory:** SRAM read/write speed, EEPROM read/write/erase, PSRAM (if present)
- **I/O:** Digital read/write (API and direct register), analog read/write
- **Hardware:** Flash info, PWM, interrupt latency, SPI communication

### Part 2 — Platform Benchmarks

Advanced and board-specific tests:

- **CPU stress testing** with temperature tracking
- **Communication:** Serial baud rate tests, I2C, SPI
- **Connectivity:** WiFi throughput, BLE/Bluetooth, LED matrix (Uno R4)
- **Cryptography:** SHA-1, AES, MD5 (hardware-accelerated where available), hardware RNG
- **Multi-core:** Dual-core performance (ESP32, Portenta H7)
- **Board-specific:** ESP32 DAC/touch/sleep, RP2040 DMA/PIO, Teensy parallel I/O, Yun Bridge, and more

## How to Use

1. Open **Arduino IDE**
2. Select your board and port
3. Open `Part1_CoreBenchmarks/Part1_CoreBenchmarks.ino`
4. Upload to your board
5. Open **Serial Monitor** at **115200 baud**
6. Wait for all tests to complete
7. Repeat with `Part2_PlatformBenchmarks/Part2_PlatformBenchmarks.ino`
8. Record results in `Arduino_Benchmark_Comparison.xlsx`

Results are printed as operations per millisecond (ops/ms), timing in microseconds, and other metrics depending on the test.

## Measurement Methodology

- **Volatile accumulators** prevent the compiler from optimizing away operations
- **Checksummed results** verify actual execution
- **Minimum-duration loops** ensure stable timing (`runForAtLeastUs`, `runTimedLoop`)
- **Register-level access** measured alongside API calls to quantify abstraction overhead
- **Jitter analysis** (5-trial runs) for timing precision benchmarks
- **yield() calls** on RTOS-based boards (ESP32, ESP8266, RP2040) to prevent watchdog resets

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on creating issues, submitting code, and adding new boards to the spreadsheet.

## License

See repository for license details.
