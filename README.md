# Arduino Universal Benchmark Suite

A comprehensive benchmarking framework for Arduino-compatible boards. The goal is to support every Arduino-compatible board — upload a sketch, open Serial Monitor, and get detailed performance metrics for any board you have on hand.

## What Is This?

This is a standardized benchmark suite that measures the real-world performance of Arduino-compatible boards. It runs a set of CPU, memory, I/O, and platform-specific tests, then prints the results over Serial so you can compare boards side by side.

The suite is split into two sketches. Part 1 covers core benchmarks that apply to any board (integer math, floating-point, memory, digital/analog I/O). Part 2 covers platform-specific features like WiFi, BLE, cryptography, multi-core, and other capabilities that vary from board to board. Results are collected in a shared spreadsheet for easy comparison.

Boards are detected automatically at compile time using preprocessor macros — no manual configuration needed. If your board isn't detected yet, it's straightforward to add (see [CONTRIBUTING.md](CONTRIBUTING.md)).

## Repository Structure

```
├── Part1_CoreBenchmarks/
│   ├── Part1_CoreBenchmarks.ino    # Core performance tests
│   └── BenchmarkHelpers.h
├── Part2_PlatformBenchmarks/
│   ├── Part2_PlatformBenchmarks.ino # Platform-specific tests
│   └── BenchmarkHelpers.h
├── BenchmarkHelpers.h               # Shared timing utilities
├── Part1_CoreBenchmarks_Results.xlsx # Core benchmark results spreadsheet
├── Part2_PlatformBenchmarks_Results.xlsx # Platform benchmark results spreadsheet
├── reports/                          # Raw Serial Monitor output per board
├── BOARD_STATUS.md                   # Board testing tracker (priority + status)
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
8. Save the full Serial Monitor output to `reports/<board>/<version>.txt`
9. Record Part 1 results in `Part1_CoreBenchmarks_Results.xlsx` and Part 2 results in `Part2_PlatformBenchmarks_Results.xlsx`

Results are printed as operations per millisecond (ops/ms), timing in microseconds, and other metrics depending on the test.

See [BOARD_STATUS.md](BOARD_STATUS.md) for which boards have been tested and which still need results.

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
