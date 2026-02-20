# Change Log Arduino-Benchmark

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/)
and this project adheres to [Semantic Versioning](https://semver.org/).

## [1.0.0] - 2026-02-16

First official release on the `v1.0.0` branch.

### Added
- Split benchmark into two parts: Part 1 (Core Benchmarks) and Part 2 (Platform Benchmarks)
- Benchmark versioning system (`BENCHMARK_VERSION` in BenchmarkHelpers.h)
- Version bump workflow documented in CONTRIBUTING.md
- `digitalWriteFast()` benchmark for direct register-write comparison
- `delay()` and `delayMicroseconds()` timing accuracy benchmarks
- Arduino Yun support with Bridge, WLAN, and WAN benchmarks
- Arduino Uno Q (Zephyr RTOS) support: flash info, die temperature, watchdog
- Comprehensive benchmarks for ESP32, RP2040, and Uno R4
- Versioned result spreadsheets (Part1 and Part2 `_Results_v1.0.0.xlsx`)
- BOARD_STATUS.md tracker for board compatibility
- CONTRIBUTING.md with guidelines and version bump workflow
- `reports/` directory with per-board serial output logs

### Fixed
- ESP32-C3 GPIO and touch benchmark issues
- ESP32 PWM performance (channel-based LEDC writes)
- Flash overflow on ATmega32U4 (`BOARD_SMALL_FLASH` gating)
- Serial output on Arduino Yun and other native USB boards
- Zephyr build issues (stack budget, sbrk, Kconfig guards)
- Stack depth test rewritten to discover actual limits before timing

### Changed
- Optimized flash usage for AVR boards (itoa over snprintf, trimmed strings)
- Scoped `BOARD_SMALL_FLASH` to Yun only, not all AVR boards

### Removed
- SD card functionality tests (present in v0.0.0, dropped during the Part 1/Part 2 split)

## [0.0.0] - 2025-01-29

Initial version on the `v0.0.0` branch.

### Added
- Single-file universal Arduino benchmark sketch
- Integer, floating-point, string, and 64-bit math benchmarks
- SRAM, EEPROM, and PSRAM speed tests
- Digital and analog I/O benchmarks
- Flash info, PWM, interrupt latency, and SPI tests
- WiFi throughput, BLE/Bluetooth benchmarks
- Cryptography benchmarks (SHA-1, AES, MD5)
- Multi-core, DAC, touch, and sleep benchmarks (ESP32)
- DMA and PIO benchmarks (RP2040)
- LED matrix benchmarks (Uno R4)
- SD card functionality tests
- Volatile accumulators and checksummed results for accuracy
- `runForAtLeastUs` / `runTimedLoop` timing helpers
- Jitter analysis (5-trial runs) for timing benchmarks
- `yield()` calls for RTOS boards (ESP32, ESP8266, RP2040)
