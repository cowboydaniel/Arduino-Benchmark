# Board Test Count Analysis

How many benchmarks does each board run? The benchmark suite uses conditional
compilation (`#ifdef` / `#ifndef`) to include or exclude tests based on board
features and architecture. This document summarises the resulting test counts.

## Base tests (all boards): 18

Every board always runs these 18 benchmarks. The 10 tests guarded by
`#ifndef BOARD_SMALL_FLASH` are included here because `BOARD_SMALL_FLASH` is
never defined in the current codebase.

| #  | Benchmark                  | Guard                        |
|----|----------------------------|------------------------------|
| 1  | `benchmarkIntegerOps`      | (none)                       |
| 2  | `benchmarkFloatOps`        | (none)                       |
| 3  | `benchmarkStringOps`       | `#ifndef BOARD_SMALL_FLASH`  |
| 4  | `benchmarkCPUStress`       | `#ifndef BOARD_SMALL_FLASH`  |
| 5  | `benchmarkSRAM`            | (none)                       |
| 6  | `benchmarkDigitalIO`       | (none)                       |
| 7  | `benchmarkAnalogIO`        | `#ifndef BOARD_SMALL_FLASH`  |
| 8  | `benchmarkSerial`          | `#ifndef BOARD_SMALL_FLASH`  |
| 9  | `benchmarkFlash`           | (none)                       |
| 10 | `benchmarkAdvancedMath`    | `#ifndef BOARD_SMALL_FLASH`  |
| 11 | `benchmarkTimingPrecision` | `#ifndef BOARD_SMALL_FLASH`  |
| 12 | `benchmarkDelayTiming`     | `#ifndef BOARD_SMALL_FLASH`  |
| 13 | `benchmarkStackDepth`      | `#ifndef BOARD_SMALL_FLASH`  |
| 14 | `benchmarkPWM`             | (none)                       |
| 15 | `benchmarkInterruptLatency`| (none)                       |
| 16 | `benchmarkSPI`             | (none)                       |
| 17 | `benchmarkWatchdog`        | `#ifndef BOARD_SMALL_FLASH`  |
| 18 | `benchmarkSleepModes`      | `#ifndef BOARD_SMALL_FLASH`  |

## Per-board totals

| Board                                  | Total | Additional tests beyond the 18 base |
|----------------------------------------|:-----:|--------------------------------------|
| ESP32 (original + PSRAM)               | **29** | EEPROM, WiFi, BLE, MultiCore, ESP32Crypto, HardwareRNG, ESP32DAC, ESP32Touch, ESP32Sleep, ESP32AES, PSRAM |
| ESP32 (original, no PSRAM)             | **28** | EEPROM, WiFi, BLE, MultiCore, ESP32Crypto, HardwareRNG, ESP32DAC, ESP32Touch, ESP32Sleep, ESP32AES |
| ESP32-S3 (+ PSRAM)                     | **28** | Same as original ESP32 minus ESP32DAC, plus PSRAM |
| ESP32-S3/S2/C3/C6/H2, Nano ESP32      | **27** | EEPROM, WiFi, BLE, MultiCore, ESP32Crypto, HardwareRNG, ESP32Touch, ESP32Sleep, ESP32AES |
| Raspberry Pi Pico W                    | **27** | EEPROM, WiFi, SHA1, MultiCore, RP2040DMA, RP2040PIO, RP2040Interpolator, RP2040PWM, PicoWBLE |
| Arduino Nano RP2040 Connect            | **27** | EEPROM, WiFi, BLE, SHA1, MultiCore, RP2040DMA, RP2040PIO, RP2040Interpolator, RP2040PWM |
| Raspberry Pi Pico                      | **25** | EEPROM, SHA1, MultiCore, RP2040DMA, RP2040PIO, RP2040Interpolator, RP2040PWM |
| Arduino Uno R4 WiFi                    | **25** | EEPROM, LEDMatrix, WiFi, SoftwareATSE, UnoR4DAC, UnoR4RTC, UnoR4BLE |
| Arduino Uno R4 Minima                  | **22** | EEPROM, LEDMatrix, UnoR4DAC, UnoR4RTC |
| ESP8266                                | **20** | EEPROM, WiFi |
| Arduino Uno WiFi Rev2                  | **20** | EEPROM, WiFi |
| Arduino Nano 33 IoT                    | **20** | WiFi, BLE |
| Arduino Portenta H7                    | **20** | WiFi, BLE |
| Arduino Portenta C33                   | **20** | WiFi, BLE |
| Arduino Giga R1 WiFi                   | **20** | WiFi, BLE |
| Seeed Wio Terminal                     | **20** | WiFi, BLE |
| Multiduino                             | **20** | EEPROM, RTC |
| Arduino Uno / Nano / Mega / Leonardo / Micro / Pro / Mini (AVR) | **19** | EEPROM |
| Teensy (all variants)                  | **19** | EEPROM |
| STM32 (Blue Pill, Black Pill, Nucleo)  | **19** | EEPROM |
| Arduino Uno Q (MCU)                    | **19** | HardwareRNG |
| Arduino Nano 33 BLE / BLE Rev2        | **19** | BLE |
| Seeed XIAO nRF52840                    | **19** | BLE |
| INK4u / MegaCoreX                      | **19** | EEPROM |
| Arduino MKR WiFi 1010 / MKR1000       | **19** | WiFi |
| Arduino Zero / MKR Zero / Due / Feather M0 / Feather M4 / Seeeduino XIAO / Unknown | **18** | (none — base tests only) |

## Conditional test catalogue

These 11 conditions gate the 23 extra tests:

| Condition | Tests enabled |
|-----------|---------------|
| `EEPROM_h \|\| ESP32 \|\| ESP8266` | EEPROM |
| `HAS_PSRAM` | PSRAM |
| `HAS_LED_MATRIX` | LEDMatrix |
| `HAS_WIFI` | WiFi |
| `HAS_BLE` | BLE |
| `HAS_RTC` | RTC |
| `ARDUINO_ARCH_RP2040` | SHA1, RP2040DMA, RP2040PIO, RP2040Interpolator, RP2040PWM, MultiCore |
| `ARDUINO_RASPBERRY_PI_PICO_W` | PicoWBLE |
| `ESP32` | MultiCore, ESP32Crypto, HardwareRNG, ESP32Touch, ESP32Sleep, ESP32AES |
| `ESP32 && CONFIG_IDF_TARGET_ESP32` | ESP32DAC |
| `ARDUINO_UNOR4_WIFI` | SoftwareATSE, UnoR4BLE |
| `ARDUINO_UNOR4_WIFI \|\| ARDUINO_UNOR4_MINIMA` | UnoR4DAC, UnoR4RTC |
| `BOARD_STM32U5` | HardwareRNG |

## Key observations

1. **Largest gap:** The original ESP32 with PSRAM runs **29** tests while
   boards like the Arduino Zero and Due run only **18** — a 61% difference.

2. **ESP32 dominates** because it has the most platform-specific benchmarks
   (crypto, AES, DAC, touch, deep-sleep) plus WiFi, BLE, and multi-core.

3. **RP2040 boards are second** (25–27) thanks to DMA, PIO, interpolator,
   and hardware-PWM benchmarks alongside SHA-1 and multi-core.

4. **Uno R4 WiFi vs Minima:** WiFi adds 3 extra tests (WiFi, SoftwareATSE,
   UnoR4BLE), giving the WiFi variant 25 vs the Minima's 22.

5. **SAMD boards without WiFi/BLE run the fewest** (18) — they lack EEPROM
   includes and have no architecture-specific benchmarks.

6. **`BOARD_SMALL_FLASH` is a dead code path** — it is checked 5 times via
   `#ifndef` but never `#define`d, so it has no effect. If enabled for
   constrained boards, it would reduce the base count from 18 to 8.
