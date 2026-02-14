/*
 * Universal Arduino Benchmark Suite
 * Compatible with: Arduino (AVR, ARM, SAMD), ESP32, ESP8266, RP2040 (Pico), STM32, and more!
 * 
 * This benchmark tests:
 * - CPU: Integer, Float, String operations
 * - Memory: SRAM, EEPROM, PSRAM (if available)
 * - I/O: Digital/Analog pins, Serial speed
 * - Board-specific: WiFi, BLE, Flash, etc.
 * 
 * METHODOLOGY:
 * - All computation benchmarks use volatile accumulators to prevent compiler optimization
 * - Results are checksummed and printed to ensure actual execution
 * - Integer operations use uint64_t to prevent silent overflow
 * - Checksums are consumed outside timed blocks for accuracy
 * - GPIO benchmarks include direct register writes alongside digitalWrite() to measure overhead
 * - Serial timing includes flush() to measure actual transmission time
 * - ESP32: CPU cycle counter used to cross-check micros() jitter
 * - EEPROM timing includes flash commit separately (ESP32/ESP8266)
 * - Memory benchmarks accumulate values to prevent dead code elimination
 * - All performance metrics reported as operations per millisecond (ops/ms)
 * - Temperature tracking (CPU stress test only) shows start temp and total gain
 * 
 * Upload to ANY Arduino-compatible board and view results in Serial Monitor (115200 baud)
 */

#include <limits.h>

#ifndef NOT_AN_INTERRUPT
#define NOT_AN_INTERRUPT -1
#endif

// ==================== BOARD DETECTION ====================

// ESP32 Family (all variants)
#if defined(ESP32)
#if defined(ARDUINO_NANO_ESP32)
#define BOARD_NAME "Arduino Nano ESP32"
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define BOARD_NAME "ESP32-S3"
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
#define BOARD_NAME "ESP32-S2"
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
#define BOARD_NAME "ESP32-C3"
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
#define BOARD_NAME "ESP32-C6"
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
#define BOARD_NAME "ESP32-H2"
#else
#define BOARD_NAME "ESP32"
#endif
#define HAS_WIFI
#define HAS_BLE
#ifdef BOARD_HAS_PSRAM
#define HAS_PSRAM
#endif
#include <WiFi.h>
#include <EEPROM.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <driver/gpio.h>
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"
#include "mbedtls/md5.h"
#include "mbedtls/sha1.h"
#include "mbedtls/sha256.h"
#include "mbedtls/sha512.h"
#include "mbedtls/md.h"
#include "mbedtls/pkcs5.h"
#include "mbedtls/version.h"
#if defined(MBEDTLS_SHA3_C)
#include "mbedtls/sha3.h"
#endif
#define EEPROM_SIZE 512

// ESP8266 Family
#elif defined(ESP8266)
#define BOARD_NAME "ESP8266"
#define HAS_WIFI
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#define EEPROM_SIZE 512

// RP2040 Family (Pico, Pico W, Arduino Nano RP2040 Connect)
#elif defined(ARDUINO_ARCH_RP2040)
#if defined(ARDUINO_NANO_RP2040_CONNECT)
#define BOARD_NAME "Arduino Nano RP2040 Connect"
#define HAS_WIFI
#define HAS_BLE
#include <WiFiNINA.h>
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
#define BOARD_NAME "Raspberry Pi Pico W"
#define HAS_WIFI
#include <WiFi.h>
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
#define BOARD_NAME "Raspberry Pi Pico"
#elif defined(ARDUINO_NANO_RP2040_CONNECT)
#define BOARD_NAME "Arduino Nano RP2040 Connect"
#define HAS_WIFI
#include <WiFiNINA.h>
#else
#ifndef BOARD_NAME
#define BOARD_NAME "RP2040"
#endif
#endif
#include <EEPROM.h>
// RP2040 temperature sensor - analogReadTemp already declared in Arduino.h
#define HAS_TEMPERATURE
// RP2040 hardware register access
#include "hardware/gpio.h"
#if defined(ARDUINO_ARCH_RP2040) && __has_include("pico/multicore.h")
#include "pico/multicore.h"
#define HAS_PICO_MULTICORE
#endif

// Renesas RA4M1 (Arduino Uno R4 WiFi & Minima)
#elif defined(ARDUINO_UNOR4_WIFI)
#define BOARD_NAME "Arduino Uno R4 WiFi"
#define HAS_WIFI
#define HAS_LED_MATRIX
#include <WiFiS3.h>
#include <EEPROM.h>
#include "Arduino_LED_Matrix.h"
#elif defined(ARDUINO_UNOR4_MINIMA)
#define BOARD_NAME "Arduino Uno R4 Minima"
#define HAS_LED_MATRIX
#include <EEPROM.h>
#include "Arduino_LED_Matrix.h"
#elif defined(ARDUINO_ARCH_RENESAS)
#define BOARD_NAME "Renesas (Uno R4 family)"
#include <EEPROM.h>

// Arduino SAMD Family (Zero, MKR, Nano 33 IoT)
#elif defined(ARDUINO_SAMD_ZERO)
#define BOARD_NAME "Arduino Zero"
#define BOARD_SAMD
#elif defined(ARDUINO_SAMD_MKR1000)
#define BOARD_NAME "Arduino MKR1000"
#define HAS_WIFI
#define BOARD_SAMD
#include <WiFi101.h>
#elif defined(ARDUINO_SAMD_MKRZERO)
#define BOARD_NAME "Arduino MKR Zero"
#define BOARD_SAMD
#elif defined(ARDUINO_SAMD_MKRWIFI1010)
#define BOARD_NAME "Arduino MKR WiFi 1010"
#define HAS_WIFI
#define BOARD_SAMD
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKRFOX1200)
#define BOARD_NAME "Arduino MKR FOX 1200"
#define BOARD_SAMD
#elif defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310)
#define BOARD_NAME "Arduino MKR WAN 1300/1310"
#define BOARD_SAMD
#elif defined(ARDUINO_SAMD_MKRGSM1400)
#define BOARD_NAME "Arduino MKR GSM 1400"
#define BOARD_SAMD
#elif defined(ARDUINO_SAMD_MKRNB1500)
#define BOARD_NAME "Arduino MKR NB 1500"
#define BOARD_SAMD
#elif defined(ARDUINO_SAMD_MKRVIDOR4000)
#define BOARD_NAME "Arduino MKR Vidor 4000"
#define HAS_FPGA
#define BOARD_SAMD
#elif defined(ARDUINO_SAMD_NANO_33_IOT)
#define BOARD_NAME "Arduino Nano 33 IoT"
#define HAS_WIFI
#define HAS_BLE
#define HAS_IMU
#define BOARD_SAMD
#include <WiFiNINA.h>
#elif defined(ARDUINO_ARCH_SAMD)
#define BOARD_NAME "SAMD (ARM Cortex-M0+)"
#define BOARD_SAMD

// Arduino Nano 33 BLE Family (nRF52840)
#elif defined(ARDUINO_ARDUINO_NANO33BLE)
#define BOARD_NAME "Arduino Nano 33 BLE"
#define HAS_BLE
#define HAS_IMU
#define BOARD_NRF52
#elif defined(ARDUINO_NANO33BLE)
#define BOARD_NAME "Arduino Nano 33 BLE (Rev2)"
#define HAS_BLE
#define HAS_IMU
#define BOARD_NRF52

// Arduino Portenta Family
#elif defined(ARDUINO_PORTENTA_H7_M7)
#define BOARD_NAME "Arduino Portenta H7"
#define HAS_WIFI
#define HAS_BLE
#define HAS_DUAL_CORE
#define BOARD_STM32H7
#elif defined(ARDUINO_PORTENTA_C33)
#define BOARD_NAME "Arduino Portenta C33"
#define HAS_WIFI
#define HAS_BLE

// Arduino Giga Family
#elif defined(ARDUINO_GIGA)
#define BOARD_NAME "Arduino Giga R1 WiFi"
#define HAS_WIFI
#define HAS_BLE
#define HAS_DUAL_CORE
#define BOARD_STM32H7

// Arduino Mega + WiFi
#elif defined(ARDUINO_AVR_MEGA2560)
#define BOARD_NAME "Arduino Mega 2560"
#include <EEPROM.h>
#define BOARD_AVR
#elif defined(ARDUINO_AVR_ADK)
#define BOARD_NAME "Arduino Mega ADK"
#include <EEPROM.h>
#define BOARD_AVR

// INK4u Board (AVR128DA28 - Next Gen Arduino Uno)
#elif defined(ARDUINO_AVR_INK4U) || defined(__AVR_AVR128DA28__)
#define BOARD_NAME "INK4u Board (AVR128DA28)"
#include <EEPROM.h>
#define BOARD_AVR
#define BOARD_MEGAAVR  // Modern AVR with advanced features
#define HAS_UPDI       // UPDI programming interface
#define BOARD_SRAM_KB 16
#define BOARD_FLASH_KB 128

// Multiduino (your custom Uno-derivative)
#elif defined(ARDUINO_AVR_MULTIDUINO)
#define BOARD_NAME "Multiduino"
#include <EEPROM.h>
#include <Wire.h>
#include <RTClib.h>  // Adafruit RTClib for DS1307
#define BOARD_AVR
#define HAS_RTC
#define RTC_I2C_ADDRESS 0x68  // Standard DS1307 I2C address

// Arduino Uno
#elif defined(ARDUINO_AVR_UNO)
#define BOARD_NAME "Arduino Uno"
#include <EEPROM.h>
#define BOARD_AVR
#elif defined(ARDUINO_AVR_UNO_WIFI_REV2)
#define BOARD_NAME "Arduino Uno WiFi Rev2"
#define HAS_WIFI
#include <EEPROM.h>
#include <WiFiNINA.h>
#define BOARD_AVR


// Arduino Nano Family (Classic)
#elif defined(ARDUINO_AVR_NANO)
#define BOARD_NAME "Arduino Nano"
#include <EEPROM.h>
#define BOARD_AVR
#elif defined(ARDUINO_AVR_NANO_EVERY)
#define BOARD_NAME "Arduino Nano Every"
#include <EEPROM.h>
#define BOARD_AVR

// MegaCoreX (ATmega x09 series - modern AVR)
#elif defined(__AVR_ATmegax09__)
#define BOARD_NAME "MegaCoreX (ATmega x09)"
#include <EEPROM.h>
#define BOARD_AVR
#define BOARD_MEGAAVR  // Modern AVR architecture like INK4u
#define BOARD_SRAM_KB 6  // Varies: 4809 has 6KB, 4809 has 6KB, etc.
#define BOARD_FLASH_KB 48  // Varies by variant

// Arduino Yun (ATmega32U4 + Atheros AR9331 Linux)
#elif defined(ARDUINO_AVR_YUN)
#define BOARD_NAME "Arduino Yun"
#include <EEPROM.h>
#define BOARD_AVR

// Arduino Leonardo/Micro
#elif defined(ARDUINO_AVR_LEONARDO)
#define BOARD_NAME "Arduino Leonardo"
#include <EEPROM.h>
#define BOARD_AVR
#elif defined(ARDUINO_AVR_MICRO)
#define BOARD_NAME "Arduino Micro"
#include <EEPROM.h>
#define BOARD_AVR

// Arduino Pro/Mini
#elif defined(ARDUINO_AVR_PRO)
#define BOARD_NAME "Arduino Pro"
#include <EEPROM.h>
#define BOARD_AVR
#elif defined(ARDUINO_AVR_MINI)
#define BOARD_NAME "Arduino Mini"
#include <EEPROM.h>
#define BOARD_AVR

// Arduino Due (SAM)
#elif defined(ARDUINO_SAM_DUE)
#define BOARD_NAME "Arduino Due"
#define BOARD_SAM

// Generic AVR catch-all
#elif defined(__AVR__)
#define BOARD_NAME "AVR-based Arduino"
#include <EEPROM.h>
#define BOARD_AVR

// Arduino Uno Q (Dual-processor: STM32U585 MCU + Qualcomm QRB2210 Linux MPU)
#elif defined(ARDUINO_UNO_Q)
#define BOARD_NAME "Arduino Uno Q (MCU)"
#define BOARD_STM32U5
#define BOARD_SRAM_KB 786
#define BOARD_FLASH_KB 2048
#define HAS_DUAL_PROCESSOR  // MCU + Linux MPU
#define HAS_FPU
#define HAS_DSP
#define HAS_RNG
#define USING_ZEPHYR
#define HAS_ROUTER_BRIDGE
#define HAS_RPC_BRIDGE
#include "Arduino_RouterBridge.h"
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/flash.h>

// STM32 Family (Blue Pill, Black Pill, Nucleo)
#elif defined(ARDUINO_ARCH_STM32)
#if defined(ARDUINO_BLUEPILL_F103C8) || defined(ARDUINO_BLUEPILL_F103CB)
#define BOARD_NAME "STM32 Blue Pill"
#elif defined(ARDUINO_BLACKPILL_F401CC) || defined(ARDUINO_BLACKPILL_F411CE)
#define BOARD_NAME "STM32 Black Pill"
#elif defined(ARDUINO_NUCLEO_F401RE)
#define BOARD_NAME "STM32 Nucleo-F401RE"
#elif defined(ARDUINO_NUCLEO_F411RE)
#define BOARD_NAME "STM32 Nucleo-F411RE"
#else
#define BOARD_NAME "STM32"
#endif
#include <EEPROM.h>
#define BOARD_STM32

// Teensy Family
#elif defined(TEENSYDUINO)
#if defined(__IMXRT1062__)
#if defined(ARDUINO_TEENSY41)
#define BOARD_NAME "Teensy 4.1"
#else
#define BOARD_NAME "Teensy 4.0"
#endif
#elif defined(__MK66FX1M0__)
#define BOARD_NAME "Teensy 3.6"
#elif defined(__MK64FX512__)
#define BOARD_NAME "Teensy 3.5"
#elif defined(__MK20DX256__)
#define BOARD_NAME "Teensy 3.2"
#elif defined(__MK20DX128__)
#define BOARD_NAME "Teensy 3.1/3.0"
#elif defined(__MKL26Z64__)
#define BOARD_NAME "Teensy LC"
#else
#define BOARD_NAME "Teensy"
#endif
#include <EEPROM.h>
#define BOARD_TEENSY

// Adafruit Feather Family
#elif defined(ADAFRUIT_FEATHER_M0)
#define BOARD_NAME "Adafruit Feather M0"
#define BOARD_SAMD
#elif defined(ADAFRUIT_FEATHER_M4)
#define BOARD_NAME "Adafruit Feather M4"
#define BOARD_SAMD

// Seeed Studio
#elif defined(SEEED_XIAO_M0)
#define BOARD_NAME "Seeeduino XIAO"
#define BOARD_SAMD
#elif defined(ARDUINO_SEEED_XIAO_NRF52840) || defined(ARDUINO_Seeed_XIAO_nRF52840)
#define BOARD_NAME "Seeed XIAO nRF52840"
#define HAS_BLE
#define BOARD_NRF52
#elif defined(ARDUINO_SEEED_XIAO_RP2040)
#define BOARD_NAME "Seeed XIAO RP2040"
#elif defined(SEEED_WIO_TERMINAL)
#define BOARD_NAME "Seeed Wio Terminal"
#define HAS_WIFI
#define HAS_BLE
#define HAS_LCD
#define BOARD_SAMD

// Unknown/Generic
#else
#define BOARD_NAME "Unknown Arduino-Compatible"
#endif

// ==================== CONFIGURATION ====================
#define BENCHMARK_ITERATIONS 10000
#define MEMORY_TEST_SIZE 1024


#if defined(ARDUINO_ARCH_RP2040)
#include <Hash.h>
#endif

#include "BenchmarkHelpers.h"
#include <SPI.h>
#if !defined(ARDUINO_UNO_Q)
#include <Wire.h>
#endif

#if defined(ESP32)
#include <HEXBuilder.h>
#include "mbedtls/aes.h"
#include "esp_sleep.h"
#endif

#if defined(ARDUINO_ARCH_RP2040)
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/interp.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#endif

#if defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
#include "RTC.h"
#endif

#if defined(__AVR__)
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#endif


// ==================== SERIAL OUTPUT ABSTRACTION ====================
// Uno Q uses Monitor instead of Serial for output
#if defined(ARDUINO_UNO_Q)
#define SERIAL_OUT Monitor
#define F_STR(x) x  // Monitor doesn't support F() macro
#else
#define SERIAL_OUT Serial
#define F_STR(x) F(x)  // Use F() macro for flash storage on other boards
#endif

// Serial baud rate
#define SERIAL_BAUD 115200

// ==================== GLOBAL VARIABLES ====================
uint8_t testBuffer[256];
unsigned long gMinBenchUs = 20000;
const uint8_t kJitterTrials = 5;

#ifdef HAS_RTC
RTC_DS1307 rtc;
#endif


// ==================== HELPER FUNCTIONS ====================

void printDivider() {
  SERIAL_OUT.println(F_STR("========================================"));
}

void printHeader(const char *title) {
  SERIAL_OUT.println();
  printDivider();
  SERIAL_OUT.println(title);
  printDivider();
}

unsigned long benchmarkStart;

void startBenchmark() {
  benchmarkStart = micros();
}

unsigned long endBenchmark() {
  return micros() - benchmarkStart;
}

template<typename T, size_t N>
struct MedianCollector {
  T values[N];
  uint8_t count;

  void add(T value) {
    if (count < N) {
      values[count++] = value;
    }
  }

  T median() const {
    if (count == 0) {
      return T();
    }
    T sorted[N];
    for (uint8_t i = 0; i < count; i++) {
      sorted[i] = values[i];
    }
    for (uint8_t i = 1; i < count; i++) {
      T key = sorted[i];
      int8_t j = i - 1;
      while (j >= 0 && sorted[j] > key) {
        sorted[j + 1] = sorted[j];
        j--;
      }
      sorted[j + 1] = key;
    }
    uint8_t mid = count / 2;
    if (count % 2 == 1) {
      return sorted[mid];
    }
    return (sorted[mid - 1] + sorted[mid]) / 2;
  }
};

void calibrateBenchmarkTime() {
  unsigned long start = micros();
  unsigned long next = start;
  uint32_t spins = 0;
  while (next == start && spins < 100000) {
    next = micros();
    spins++;
  }
  unsigned long microsResolution = next - start;

  unsigned long delayStart = micros();
  delayMicroseconds(1000);
  unsigned long delayElapsed = micros() - delayStart;
  long delayError = abs((long)delayElapsed - 1000);

  if (microsResolution <= 2 && delayError <= 5) {
    gMinBenchUs = 5000;
  } else if (microsResolution <= 8 && delayError <= 25) {
    gMinBenchUs = 20000;
  } else {
    gMinBenchUs = 50000;
  }
}

// ==================== CPU BENCHMARKS ====================


void benchmarkIntegerOps() {
  printHeader("CPU: INTEGER OPERATIONS");

  const uint32_t minDurationMs = 5;

  // Addition - use volatile uint64_t to prevent optimization and overflow
  volatile uint64_t acc = 0;
  startBenchmark();
  for (uint32_t i = 0; i < BENCHMARK_ITERATIONS; i++) {
    acc += i;
  }
  unsigned long addTime = endBenchmark();
  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println((uint32_t)(acc & 0xFFFFFFFF));

  // Multiplication - use LCG-style updates to prevent optimization
  acc = 1;
  TimedLoopResult mulResult = runTimedLoop(minDurationMs, 100, [&]() {
    for (uint32_t i = 1; i <= 100; i++) {
      acc = (acc * (i | 1)) & 0xFFFFFFFF;  // Ensure odd multiplier, prevent overflow
    }
  });
  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println((uint32_t)acc);

  // Division - vary both dividend and divisor
  acc = 0xFFFFFFFFULL;
  TimedLoopResult divResult = runTimedLoop(minDurationMs, 100, [&]() {
    for (uint32_t i = 1; i <= 100; i++) {
      uint32_t divisor = (i % 127) + 2;  // 2-128, avoid div-by-1
      acc = (acc / divisor) + i;         // Accumulate to prevent optimization
    }
  });
  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println((uint32_t)acc);

  SERIAL_OUT.print(F_STR("Addition ("));
  SERIAL_OUT.print(BENCHMARK_ITERATIONS);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(addTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print((float)BENCHMARK_ITERATIONS / addTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Multiplication ("));
  SERIAL_OUT.print(mulResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(mulResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(mulResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Division ("));
  SERIAL_OUT.print(divResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(divResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(divResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

#if defined(BOARD_STM32U5) || defined(HAS_DSP)
  // Enhanced tests for Cortex-M33 with DSP extensions
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("--- DSP-Enhanced Integer Tests (Cortex-M33) ---"));

  // 64-bit integer operations (emulated on 32-bit MCU)
  volatile uint64_t acc64 = 0x123456789ABCDEFULL;
  TimedLoopResult mul64Result = runTimedLoop(minDurationMs, 50, [&]() {
    for (uint32_t i = 1; i <= 50; i++) {
      acc64 = (acc64 * (i | 1));  // 64-bit multiply
    }
  });
  SERIAL_OUT.print(F_STR("64-bit Multiply ("));
  SERIAL_OUT.print(mul64Result.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(mul64Result.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms"));

  // 64-bit division (expensive on 32-bit MCU)
  acc64 = 0xFFFFFFFFFFFFFFFFULL;
  TimedLoopResult div64Result = runTimedLoop(minDurationMs, 50, [&]() {
    for (uint32_t i = 1; i <= 50; i++) {
      uint64_t divisor = ((uint64_t)i * 123456) + 2;
      acc64 = (acc64 / divisor) + i;
    }
  });
  SERIAL_OUT.print(F_STR("64-bit Divide ("));
  SERIAL_OUT.print(div64Result.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(div64Result.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms"));

  // MAC (Multiply-Accumulate) style operations - DSP strength
  volatile int32_t macAcc = 0;
  volatile int32_t macResult[4] = { 0, 0, 0, 0 };
  startBenchmark();
  for (uint32_t i = 0; i < 1000; i++) {
    // Simulate DSP-style MAC operations
    int32_t a = (int32_t)(i & 0xFFFF);
    int32_t b = (int32_t)((i >> 8) & 0xFFFF);
    macAcc += a * b;  // MAC operation
    macResult[i % 4] += macAcc;
  }
  unsigned long macTime = endBenchmark();
  SERIAL_OUT.print(F_STR("MAC Operations (1000 ops): "));
  SERIAL_OUT.print(macTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(1000000.0 / macTime);
  SERIAL_OUT.println(F_STR(" ops/ms)"));
  SERIAL_OUT.print(F_STR("MAC Checksum: "));
  SERIAL_OUT.println(macAcc);

  // Saturating arithmetic (DSP-style)
  volatile int32_t satAcc = 0;
  startBenchmark();
  for (uint32_t i = 0; i < 1000; i++) {
    int32_t val = (int32_t)i * 1000000;
    // Simulate saturating add (clamp to INT32_MAX)
    if (satAcc > INT32_MAX - val) {
      satAcc = INT32_MAX;
    } else {
      satAcc += val;
    }
  }
  unsigned long satTime = endBenchmark();
  SERIAL_OUT.print(F_STR("Saturating Add (1000 ops): "));
  SERIAL_OUT.print(satTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(1000000.0 / satTime);
  SERIAL_OUT.println(F_STR(" ops/ms)"));
#endif
}

void benchmarkFloatOps() {
  printHeader("CPU: FLOATING POINT OPERATIONS");

  const uint32_t minDurationMs = 5;

  // Float addition - volatile prevents optimization
  volatile float fresult = 0.0f;
  startBenchmark();
  for (uint32_t i = 0; i < BENCHMARK_ITERATIONS / 10; i++) {
    fresult += 3.14159f;
  }
  unsigned long faddTime = endBenchmark();
  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println(fresult);

  // Float multiplication
  fresult = 1.0f;
  startBenchmark();
  for (uint32_t i = 0; i < BENCHMARK_ITERATIONS / 10; i++) {
    fresult *= 1.0001f;
  }
  unsigned long fmulTime = endBenchmark();
  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println(fresult);

  // Float division
  fresult = 1000000.0f;
  startBenchmark();
  for (uint32_t i = 0; i < BENCHMARK_ITERATIONS / 10; i++) {
    fresult /= 1.0001f;
  }
  unsigned long fdivTime = endBenchmark();
  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println(fresult);

#if !defined(ARDUINO_UNO_Q) && !defined(ARDUINO_UNO_Q_MCU)
  // Sqrt - accumulate to prevent optimization (skipped on Uno Q - no libm)
  fresult = 0.0f;
  TimedLoopResult sqrtResult = runTimedLoop(minDurationMs, 100, [&]() {
    for (uint32_t i = 0; i < 100; i++) {
      fresult += sqrt((float)i);
    }
  });
  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println(fresult);

  // Sin/Cos - accumulate to prevent optimization (skipped on Uno Q - no libm)
  fresult = 0.0f;
  TimedLoopResult trigResult = runTimedLoop(minDurationMs, 100, [&]() {
    for (uint32_t i = 0; i < 100; i++) {
      fresult += sin((float)i / 100.0f) + cos((float)i / 100.0f);
    }
  });
  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println(fresult);
#endif

  SERIAL_OUT.print(F_STR("Float Addition ("));
  SERIAL_OUT.print(BENCHMARK_ITERATIONS / 10);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(faddTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print((float)(BENCHMARK_ITERATIONS / 10) / faddTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Float Multiply ("));
  SERIAL_OUT.print(BENCHMARK_ITERATIONS / 10);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(fmulTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print((float)(BENCHMARK_ITERATIONS / 10) / fmulTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Float Division ("));
  SERIAL_OUT.print(BENCHMARK_ITERATIONS / 10);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(fdivTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print((float)(BENCHMARK_ITERATIONS / 10) / fdivTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

#if !defined(ARDUINO_UNO_Q) && !defined(ARDUINO_UNO_Q_MCU)
  SERIAL_OUT.print(F_STR("Square Root ("));
  SERIAL_OUT.print(sqrtResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(sqrtResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(sqrtResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Sin/Cos ("));
  SERIAL_OUT.print(trigResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(trigResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(trigResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));
#else
  SERIAL_OUT.println(F_STR("Sqrt/Sin/Cos: Skipped (Uno Q - no libm)"));
#endif

  SERIAL_OUT.println();
  SERIAL_OUT.print(F_STR("sizeof(float):  "));
  SERIAL_OUT.println(sizeof(float));
  SERIAL_OUT.print(F_STR("sizeof(double): "));
  SERIAL_OUT.println(sizeof(double));

#if !defined(ARDUINO_UNO_Q) && !defined(ARDUINO_UNO_Q_MCU) && defined(__SIZEOF_DOUBLE__) && __SIZEOF_DOUBLE__ >= 8
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("8-byte double detected - double precision tests:"));

  // Double multiplication
  volatile double dresult = 1.0;
  startBenchmark();
  for (uint32_t i = 0; i < BENCHMARK_ITERATIONS / 10; i++) {
    dresult *= 1.0001;
  }
  unsigned long dmulTime = endBenchmark();
  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println((float)dresult);

  // Double division
  dresult = 1000000.0;
  startBenchmark();
  for (uint32_t i = 0; i < BENCHMARK_ITERATIONS / 10; i++) {
    dresult /= 1.0001;
  }
  unsigned long ddivTime = endBenchmark();
  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println((float)dresult);

  // Double sqrt
  dresult = 0.0;
  TimedLoopResult dsqrtResult = runTimedLoop(minDurationMs, 100, [&]() {
    for (uint32_t i = 0; i < 100; i++) {
      dresult += sqrt((double)i);
    }
  });
  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println((float)dresult);

  // Double sin/cos
  dresult = 0.0;
  TimedLoopResult dtrigResult = runTimedLoop(minDurationMs, 100, [&]() {
    for (uint32_t i = 0; i < 100; i++) {
      dresult += sin((double)i / 100.0) + cos((double)i / 100.0);
    }
  });
  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println((float)dresult);

  SERIAL_OUT.print(F_STR("Double Multiply ("));
  SERIAL_OUT.print(BENCHMARK_ITERATIONS / 10);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(dmulTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print((float)(BENCHMARK_ITERATIONS / 10) / dmulTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Double Division ("));
  SERIAL_OUT.print(BENCHMARK_ITERATIONS / 10);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(ddivTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print((float)(BENCHMARK_ITERATIONS / 10) / ddivTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Double Sqrt ("));
  SERIAL_OUT.print(dsqrtResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(dsqrtResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(dsqrtResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Double Sin/Cos ("));
  SERIAL_OUT.print(dtrigResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(dtrigResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(dtrigResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));
#elif !defined(ARDUINO_UNO_Q) && !defined(ARDUINO_UNO_Q_MCU)
  SERIAL_OUT.println(F_STR("4-byte double (same as float) - double tests skipped"));
#endif
}

#ifndef BOARD_SMALL_FLASH
void benchmarkStringOps() {
  printHeader("CPU: STRING OPERATIONS");

  const uint32_t minDurationMs = max(5UL, (gMinBenchUs + 999UL) / 1000UL);

  // String concatenation
  String testString = "";
  TimedLoopResult concatResult = runTimedLoop(minDurationMs, 100, [&]() {
    testString = "";
    for (int i = 0; i < 100; i++) {
      testString += "X";
    }
  });

  // String comparison
  String str1 = "TestString123";
  String str2 = "TestString123";
  volatile bool cmpResult;
  TimedLoopResult cmpResultData = runTimedLoop(minDurationMs, 1000, [&]() {
    for (int i = 0; i < 1000; i++) {
      cmpResult = (str1 == str2);
    }
  });

  // Integer to String
  String numStr;
  TimedLoopResult toStrResult = runTimedLoop(minDurationMs, 1000, [&]() {
    for (int i = 0; i < 1000; i++) {
      numStr = String(i);
    }
  });

  // itoa into fixed buffer (no heap allocation)
  char fixedBuffer[12];
  TimedLoopResult itoaResult = runTimedLoop(minDurationMs, 1000, [&]() {
    for (int i = 0; i < 1000; i++) {
      itoa(i, fixedBuffer, 10);
    }
  });

  SERIAL_OUT.print(F_STR("Concatenation ("));
  SERIAL_OUT.print(concatResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(concatResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(concatResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Comparison ("));
  SERIAL_OUT.print(cmpResultData.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(cmpResultData.elapsedMicros);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(cmpResultData.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Int to String ("));
  SERIAL_OUT.print(toStrResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(toStrResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(toStrResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("itoa fixed buf ("));
  SERIAL_OUT.print(itoaResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(itoaResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(itoaResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));
}
#endif  // BOARD_SMALL_FLASH
// ==================== MEMORY BENCHMARKS ====================

void benchmarkSRAM() {
  printHeader("MEMORY: SRAM READ/WRITE");

  const uint32_t minDurationMs = 5;

  // Sequential write
  TimedLoopResult writeResult = runTimedLoop(minDurationMs, 256, [&]() {
    for (uint16_t i = 0; i < 256; i++) {
      testBuffer[i] = (uint8_t)i;
    }
  });

  // Sequential read - accumulate to prevent optimization
  volatile uint32_t checksum = 0;
  TimedLoopResult readResult = runTimedLoop(minDurationMs, 256, [&]() {
    for (uint16_t i = 0; i < 256; i++) {
      checksum += testBuffer[i];
    }
  });
  SERIAL_OUT.print(F_STR("Read checksum: "));
  SERIAL_OUT.println((uint32_t)checksum);

  // Random access - accumulate to prevent optimization
  MedianCollector<float, kJitterTrials> randomOpsMedian = {};
  MedianCollector<unsigned long, kJitterTrials> randomElapsedMedian = {};
  MedianCollector<uint32_t, kJitterTrials> randomTotalOpsMedian = {};
  for (uint8_t trial = 0; trial < kJitterTrials; trial++) {
    checksum = 0;
    TimedLoopResult randomResult = runTimedLoop(minDurationMs, 256, [&]() {
      for (uint16_t i = 0; i < 256; i++) {
        uint8_t idx = (i * 7 + 13) % 256;  // Pseudo-random
        checksum += testBuffer[idx];
      }
    });
    randomOpsMedian.add(randomResult.opsPerMs);
    randomElapsedMedian.add(randomResult.elapsedMicros);
    randomTotalOpsMedian.add(randomResult.totalOps);
  }
  float randomOpsPerMs = randomOpsMedian.median();
  unsigned long randomElapsedMicros = randomElapsedMedian.median();
  uint32_t randomTotalOps = randomTotalOpsMedian.median();
  SERIAL_OUT.print(F_STR("Random checksum: "));
  SERIAL_OUT.println((uint32_t)checksum);

  SERIAL_OUT.print(F_STR("Sequential Write ("));
  SERIAL_OUT.print(writeResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(writeResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(writeResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Sequential Read ("));
  SERIAL_OUT.print(readResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(readResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(readResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Random Access ("));
  SERIAL_OUT.print(randomTotalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(randomElapsedMicros);
  SERIAL_OUT.print(F_STR(" μs (median "));
  SERIAL_OUT.print(randomOpsPerMs);
  SERIAL_OUT.print(F_STR(" ops/ms, "));
  SERIAL_OUT.print(kJitterTrials);
  SERIAL_OUT.println(F_STR(" trials)"));

  // Enhanced memory tests - scaled by available RAM
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("--- Memory Bandwidth Tests ---"));

  // Scale buffer size based on available RAM
  size_t freeHeapBytes = 0;
#if defined(ESP32) || defined(ESP8266)
  freeHeapBytes = ESP.getFreeHeap();
#elif defined(__AVR__)
  extern int __heap_start, *__brkval;
  int v;
  freeHeapBytes =
    (size_t)((int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval));
#elif defined(ARDUINO_SAM_DUE)
  extern "C" char *sbrk(int i);
  char *ramend = (char *)0x20088000;
  freeHeapBytes = (size_t)(ramend - sbrk(0));
#else
  // Probe heap by attempting allocations until failure.
  size_t low = 0;
  size_t high = 1024;
  while (true) {
    void *probe = malloc(high);
    if (probe) {
      free(probe);
      low = high;
      if (high > (SIZE_MAX / 2)) {
        break;
      }
      high *= 2;
    } else {
      break;
    }
  }
  size_t left = low;
  size_t right = high;
  while (left + 1 < right) {
    size_t mid = left + (right - left) / 2;
    void *probe = malloc(mid);
    if (probe) {
      free(probe);
      left = mid;
    } else {
      right = mid;
    }
  }
  freeHeapBytes = left;
#endif

  if (freeHeapBytes > 0) {
    SERIAL_OUT.print(F_STR("Detected free heap: "));
    SERIAL_OUT.print(freeHeapBytes);
    SERIAL_OUT.println(F_STR(" bytes"));
  }

  size_t bufSize = 0;
  if (freeHeapBytes > 0) {
    // Use ~75% of free heap across two buffers (leave headroom for stack/overhead).
    bufSize = (freeHeapBytes * 3) / 8;
    bufSize = (bufSize / 4) * 4;
    if (bufSize < 128) {
      bufSize = 128;
    }
    SERIAL_OUT.print(F_STR("Using "));
    SERIAL_OUT.print(bufSize);
    SERIAL_OUT.println(F_STR(" byte buffers (~75% free heap total)"));
  }

  // Allocate buffers on heap for safety
  if (bufSize == 0) {
    bufSize = 512;
    SERIAL_OUT.println(F_STR("Using 512 byte buffers (fallback)"));
  }

  uint8_t *largeSrc = NULL;
  uint8_t *largeDst = NULL;
  size_t attemptSize = bufSize;
  while (attemptSize >= 64 && (!largeSrc || !largeDst)) {
    largeSrc = (uint8_t *)malloc(attemptSize);
    largeDst = (uint8_t *)malloc(attemptSize);
    if (largeSrc && largeDst) {
      bufSize = attemptSize;
      break;
    }
    if (largeSrc) {
      free(largeSrc);
      largeSrc = NULL;
    }
    if (largeDst) {
      free(largeDst);
      largeDst = NULL;
    }
    attemptSize /= 2;
  }

  if (largeSrc == NULL || largeDst == NULL) {
    SERIAL_OUT.println(F_STR("ERROR: Could not allocate test buffers"));
    if (largeSrc) free(largeSrc);
    if (largeDst) free(largeDst);
  } else {
    // Initialize source buffer
    for (size_t i = 0; i < bufSize; i++) {
      largeSrc[i] = (uint8_t)(i & 0xFF);
    }

    // memcpy throughput test
    int iterations = (bufSize >= 1024) ? 100 : 200;  // More iterations for small buffers
    startBenchmark();
    for (int iter = 0; iter < iterations; iter++) {
      memcpy(largeDst, largeSrc, bufSize);
    }
    unsigned long memcpyTime = endBenchmark();

    SERIAL_OUT.print(F_STR("memcpy ("));
    SERIAL_OUT.print(bufSize * iterations);
    SERIAL_OUT.print(F_STR(" bytes): "));
    SERIAL_OUT.print(memcpyTime);
    SERIAL_OUT.print(F_STR(" μs ("));
    SERIAL_OUT.print((bufSize * iterations * 1.0) / memcpyTime);
    SERIAL_OUT.println(F_STR(" MB/s)"));

    // memset throughput test
    startBenchmark();
    for (int iter = 0; iter < iterations; iter++) {
      memset(largeDst, 0xAA, bufSize);
    }
    unsigned long memsetTime = endBenchmark();

    SERIAL_OUT.print(F_STR("memset ("));
    SERIAL_OUT.print(bufSize * iterations);
    SERIAL_OUT.print(F_STR(" bytes): "));
    SERIAL_OUT.print(memsetTime);
    SERIAL_OUT.print(F_STR(" μs ("));
    SERIAL_OUT.print((bufSize * iterations * 1.0) / memsetTime);
    SERIAL_OUT.println(F_STR(" MB/s)"));

    // Memory bandwidth test - tight loop
    volatile uint32_t *ramPtr = (volatile uint32_t *)largeDst;
    const size_t numWords = bufSize / 4;

    startBenchmark();
    for (int iter = 0; iter < iterations; iter++) {
      for (size_t i = 0; i < numWords; i++) {
        ramPtr[i] = i;  // Sequential write
      }
    }
    unsigned long bandwidthWriteTime = endBenchmark();

    startBenchmark();
    volatile uint32_t sum = 0;
    for (int iter = 0; iter < iterations; iter++) {
      for (size_t i = 0; i < numWords; i++) {
        sum += ramPtr[i];  // Sequential read
      }
    }
    unsigned long bandwidthReadTime = endBenchmark();

    SERIAL_OUT.print(F_STR("RAM Write Bandwidth: "));
    SERIAL_OUT.print((bufSize * iterations * 1.0) / bandwidthWriteTime);
    SERIAL_OUT.println(F_STR(" MB/s"));

    SERIAL_OUT.print(F_STR("RAM Read Bandwidth: "));
    SERIAL_OUT.print((bufSize * iterations * 1.0) / bandwidthReadTime);
    SERIAL_OUT.println(F_STR(" MB/s"));

    // Clean up
    free(largeSrc);
    free(largeDst);
  }
}

#if defined(EEPROM_h) || defined(ESP32) || defined(ESP8266)
void benchmarkEEPROM() {
  printHeader("MEMORY: EEPROM");

#if defined(ESP32) || defined(ESP8266)
  EEPROM.begin(EEPROM_SIZE);
#endif

  int eepromSize = 0;
#if defined(ESP32) || defined(ESP8266)
  eepromSize = EEPROM_SIZE;
#elif defined(EEPROM_h)
  eepromSize = EEPROM.length();
#endif

  if (eepromSize > 0) {
    SERIAL_OUT.print(F_STR("EEPROM Size: "));
    SERIAL_OUT.print(eepromSize);
    SERIAL_OUT.println(F_STR(" bytes"));

    // Write test (smaller sample)
    int testSize = min(64, eepromSize);

#if defined(ESP32) || defined(ESP8266)
    // ESP32/ESP8266: write to RAM buffer
    startBenchmark();
    for (int i = 0; i < testSize; i++) {
      EEPROM.write(i, (uint8_t)i);
    }
    unsigned long ramWriteTime = endBenchmark();

    SERIAL_OUT.print(F_STR("RAM Buffer Write ("));
    SERIAL_OUT.print(testSize);
    SERIAL_OUT.print(F_STR(" bytes): "));
    SERIAL_OUT.print(ramWriteTime);
    SERIAL_OUT.print(F_STR(" μs ("));
    SERIAL_OUT.print((float)testSize / ramWriteTime * 1000);
    SERIAL_OUT.println(F_STR(" ops/ms)"));

    // Commit multiple times to measure min/median/max (captures erase events)
    SERIAL_OUT.println(F_STR("Flash Commit Test (10 commits):"));
    unsigned long commitTimes[10];

    for (int trial = 0; trial < 10; trial++) {
      // Write alternating pattern to force bit transitions
      for (int i = 0; i < testSize; i++) {
        EEPROM.write(i, (trial % 2) ? 0xFF : 0x00);
      }

      // Time the commit
      startBenchmark();
      EEPROM.commit();
      commitTimes[trial] = endBenchmark();
    }

    // Sort to find min/median/max
    for (int i = 0; i < 9; i++) {
      for (int j = i + 1; j < 10; j++) {
        if (commitTimes[j] < commitTimes[i]) {
          unsigned long temp = commitTimes[i];
          commitTimes[i] = commitTimes[j];
          commitTimes[j] = temp;
        }
      }
    }

    unsigned long minCommit = commitTimes[0];
    unsigned long medianCommit = commitTimes[5];
    unsigned long maxCommit = commitTimes[9];

    SERIAL_OUT.print(F_STR("  Min: "));
    SERIAL_OUT.print(minCommit);
    SERIAL_OUT.print(F_STR(" μs ("));
    SERIAL_OUT.print(minCommit / 1000.0);
    SERIAL_OUT.println(F_STR(" ms)"));

    SERIAL_OUT.print(F_STR("  Median: "));
    SERIAL_OUT.print(medianCommit);
    SERIAL_OUT.print(F_STR(" μs ("));
    SERIAL_OUT.print(medianCommit / 1000.0);
    SERIAL_OUT.println(F_STR(" ms)"));

    SERIAL_OUT.print(F_STR("  Max: "));
    SERIAL_OUT.print(maxCommit);
    SERIAL_OUT.print(F_STR(" μs ("));
    SERIAL_OUT.print(maxCommit / 1000.0);
    SERIAL_OUT.println(F_STR(" ms) ← Includes erase"));
#else
    // AVR/native EEPROM: each write goes to hardware
    startBenchmark();
    for (int i = 0; i < testSize; i++) {
      EEPROM.write(i, (uint8_t)i);
    }
    unsigned long writeTime = endBenchmark();

    SERIAL_OUT.print(F_STR("Hardware Write ("));
    SERIAL_OUT.print(testSize);
    SERIAL_OUT.print(F_STR(" bytes): "));
    SERIAL_OUT.print(writeTime);
    SERIAL_OUT.print(F_STR(" μs ("));
    SERIAL_OUT.print((float)testSize / writeTime * 1000);
    SERIAL_OUT.println(F_STR(" ops/ms)"));
#endif

    // Read test - measure actual reads with checksum
    delay(10);  // Let EEPROM settle
    volatile uint32_t checksum = 0;
    startBenchmark();
    for (int i = 0; i < testSize; i++) {
      checksum += EEPROM.read(i);
    }
    unsigned long readTime = endBenchmark();

    SERIAL_OUT.print(F_STR("Read ("));
    SERIAL_OUT.print(testSize);
    SERIAL_OUT.print(F_STR(" bytes): "));
    SERIAL_OUT.print(readTime);
    SERIAL_OUT.print(F_STR(" μs ("));
    SERIAL_OUT.print((float)testSize / readTime * 1000);
    SERIAL_OUT.println(F_STR(" ops/ms)"));
    SERIAL_OUT.print(F_STR("Read checksum: "));
    SERIAL_OUT.println((uint32_t)checksum);
  } else {
    SERIAL_OUT.println(F_STR("EEPROM not available"));
  }

#if defined(ESP32) || defined(ESP8266)
  EEPROM.end();
#endif
}
#endif

#ifdef HAS_PSRAM
void benchmarkPSRAM() {
  printHeader("MEMORY: PSRAM");

  if (psramFound()) {
    SERIAL_OUT.print(F_STR("PSRAM Size: "));
    SERIAL_OUT.print(ESP.getPsramSize() / 1024);
    SERIAL_OUT.println(F_STR(" KB"));

    SERIAL_OUT.print(F_STR("Free PSRAM: "));
    SERIAL_OUT.print(ESP.getFreePsram() / 1024);
    SERIAL_OUT.println(F_STR(" KB"));

    // Allocate test buffer in PSRAM
    uint8_t *psramBuffer = (uint8_t *)ps_malloc(4096);
    if (psramBuffer != NULL) {
      // Write test
      startBenchmark();
      for (int i = 0; i < 4096; i++) {
        psramBuffer[i] = i & 0xFF;
      }
      unsigned long writeTime = endBenchmark();

      // Read test - accumulate to prevent optimization
      volatile uint32_t checksum = 0;
      startBenchmark();
      for (int i = 0; i < 4096; i++) {
        checksum += psramBuffer[i];
      }
      unsigned long readTime = endBenchmark();

      SERIAL_OUT.print(F_STR("Write (4096 bytes): "));
      SERIAL_OUT.print(writeTime);
      SERIAL_OUT.print(F_STR(" μs ("));
      SERIAL_OUT.print(4096.0 / writeTime * 1000);
      SERIAL_OUT.println(F_STR(" ops/ms)"));

      SERIAL_OUT.print(F_STR("Read (4096 bytes): "));
      SERIAL_OUT.print(readTime);
      SERIAL_OUT.print(F_STR(" μs ("));
      SERIAL_OUT.print(4096.0 / readTime * 1000);
      SERIAL_OUT.println(F_STR(" ops/ms)"));

      SERIAL_OUT.print(F_STR("Read checksum: "));
      SERIAL_OUT.println((uint32_t)checksum);

      free(psramBuffer);
    } else {
      SERIAL_OUT.println(F_STR("Failed to allocate PSRAM"));
    }
  } else {
    SERIAL_OUT.println(F_STR("PSRAM not found"));
  }
}
#endif
// ==================== I/O BENCHMARKS ====================

void benchmarkDigitalIO() {
  printHeader("I/O: DIGITAL PIN OPERATIONS");

  const uint32_t minDurationMs = 5;

  // Find a safe digital pin to test
  int testPin;
#if defined(LED_BUILTIN)
  testPin = LED_BUILTIN;
#elif defined(ESP32)
  testPin = 2;  // Most ESP32 boards have LED on GPIO2
#elif defined(ESP8266)
  testPin = 2;
#elif defined(ARDUINO_ARCH_RP2040)
  testPin = 25;
#else
  testPin = 13;           // Classic Arduino default
#endif

  pinMode(testPin, OUTPUT);

  // digitalWrite benchmark
  MedianCollector<float, kJitterTrials> writeOpsMedian = {};
  MedianCollector<unsigned long, kJitterTrials> writeElapsedMedian = {};
  uint32_t dwOpsPerTrial = 0;
  for (uint8_t trial = 0; trial < kJitterTrials; trial++) {
    volatile uint32_t dwOps = 0;
    startBenchmark();
    for (int i = 0; i < 1000; i++) {
      digitalWrite(testPin, HIGH);
      digitalWrite(testPin, LOW);
      dwOps += 2;
    }
    unsigned long writeTime = endBenchmark();
    if (trial == 0) {
      dwOpsPerTrial = dwOps;
    }
    writeElapsedMedian.add(writeTime);
    writeOpsMedian.add(dwOps * 1000.0f / writeTime);
  }
  unsigned long writeTime = writeElapsedMedian.median();
  float writeOpsPerMs = writeOpsMedian.median();

// Direct port manipulation (AVR only)
#ifdef __AVR__
  volatile uint8_t *out = portOutputRegister(digitalPinToPort(testPin));
  uint8_t mask = digitalPinToBitMask(testPin);
  MedianCollector<float, kJitterTrials> portOpsMedian = {};
  MedianCollector<unsigned long, kJitterTrials> portElapsedMedian = {};
  MedianCollector<uint32_t, kJitterTrials> portTotalOpsMedian = {};
  for (uint8_t trial = 0; trial < kJitterTrials; trial++) {
    TimedLoopResult portResult = runTimedLoop(minDurationMs, 2000, [&]() {
      for (int i = 0; i < 1000; i++) {
        *out |= mask;   // Set
        *out &= ~mask;  // Clear
      }
    });
    portOpsMedian.add(portResult.opsPerMs);
    portElapsedMedian.add(portResult.elapsedMicros);
    portTotalOpsMedian.add(portResult.totalOps);
  }
  float portOpsPerMs = portOpsMedian.median();
  unsigned long portElapsedMicros = portElapsedMedian.median();
  uint32_t portTotalOps = portTotalOpsMedian.median();
#endif

// Direct register write (ESP32)
#ifdef ESP32
  MedianCollector<float, kJitterTrials> regOpsMedian = {};
  MedianCollector<unsigned long, kJitterTrials> regElapsedMedian = {};
  MedianCollector<uint32_t, kJitterTrials> regTotalOpsMedian = {};
  for (uint8_t trial = 0; trial < kJitterTrials; trial++) {
    TimedLoopResult regResult = runTimedLoop(minDurationMs, 2000, [&]() {
      for (int i = 0; i < 1000; i++) {
#if defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32H2) || defined(CONFIG_IDF_TARGET_ESP32C3)
        // ESP32-C6/C5/H2/C3 use different register structure with .val field
        if (testPin < 32) {
          GPIO.out_w1ts.val = 1u << testPin;
          GPIO.out_w1tc.val = 1u << testPin;
        } else {
#if defined(CONFIG_IDF_TARGET_ESP32C3)
          digitalWrite(testPin, HIGH);
          digitalWrite(testPin, LOW);
#else
            GPIO.out1_w1ts.val = 1u << (testPin - 32);
            GPIO.out1_w1tc.val = 1u << (testPin - 32);
#endif
        }
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
          // Other ESP32 variants
          if (testPin < 32) {
            GPIO.out_w1ts = 1u << testPin;
            GPIO.out_w1tc = 1u << testPin;
          } else {
            GPIO.out1_w1ts.data = 1u << (testPin - 32);
            GPIO.out1_w1tc.data = 1u << (testPin - 32);
          }
#else
          digitalWrite(testPin, HIGH);
          digitalWrite(testPin, LOW);
#endif
      }
    });
    regOpsMedian.add(regResult.opsPerMs);
    regElapsedMedian.add(regResult.elapsedMicros);
    regTotalOpsMedian.add(regResult.totalOps);
  }
  float regOpsPerMs = regOpsMedian.median();
  unsigned long regElapsedMicros = regElapsedMedian.median();
  uint32_t regTotalOps = regTotalOpsMedian.median();
#endif

// Direct register write (RP2040)
#ifdef ARDUINO_ARCH_RP2040
  MedianCollector<float, kJitterTrials> regOpsMedian = {};
  MedianCollector<unsigned long, kJitterTrials> regElapsedMedian = {};
  MedianCollector<uint32_t, kJitterTrials> regTotalOpsMedian = {};
  for (uint8_t trial = 0; trial < kJitterTrials; trial++) {
    TimedLoopResult regResult = runTimedLoop(minDurationMs, 2000, [&]() {
      for (int i = 0; i < 1000; i++) {
        sio_hw->gpio_set = 1ul << testPin;  // Set
        sio_hw->gpio_clr = 1ul << testPin;  // Clear
      }
    });
    regOpsMedian.add(regResult.opsPerMs);
    regElapsedMedian.add(regResult.elapsedMicros);
    regTotalOpsMedian.add(regResult.totalOps);
  }
  float regOpsPerMs = regOpsMedian.median();
  unsigned long regElapsedMicros = regElapsedMedian.median();
  uint32_t regTotalOps = regTotalOpsMedian.median();
#endif

  SERIAL_OUT.print(F_STR("digitalWrite() ("));
  SERIAL_OUT.print(dwOpsPerTrial);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(writeTime);
  SERIAL_OUT.print(F_STR(" μs (median "));
  SERIAL_OUT.print(writeOpsPerMs);
  SERIAL_OUT.print(F_STR(" ops/ms, "));
  SERIAL_OUT.print(kJitterTrials);
  SERIAL_OUT.println(F_STR(" trials)"));

#ifdef __AVR__
  SERIAL_OUT.print(F_STR("Direct Port ("));
  SERIAL_OUT.print(portTotalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(portElapsedMicros);
  SERIAL_OUT.print(F_STR(" μs (median "));
  SERIAL_OUT.print(portOpsPerMs);
  SERIAL_OUT.print(F_STR(" ops/ms, "));
  SERIAL_OUT.print(kJitterTrials);
  SERIAL_OUT.println(F_STR(" trials)"));
  SERIAL_OUT.print(F_STR("Speedup: "));
  SERIAL_OUT.print(portOpsPerMs / writeOpsPerMs);
  SERIAL_OUT.println(F_STR("x faster"));
#endif

#if defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
  SERIAL_OUT.print(F_STR("Direct Register ("));
  SERIAL_OUT.print(regTotalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(regElapsedMicros);
  SERIAL_OUT.print(F_STR(" μs (median "));
  SERIAL_OUT.print(regOpsPerMs);
  SERIAL_OUT.print(F_STR(" ops/ms, "));
  SERIAL_OUT.print(kJitterTrials);
  SERIAL_OUT.println(F_STR(" trials)"));
  SERIAL_OUT.print(F_STR("Speedup: "));
  SERIAL_OUT.print(regOpsPerMs / writeOpsPerMs);
  SERIAL_OUT.println(F_STR("x faster"));
#endif
}

#ifndef BOARD_SMALL_FLASH
void benchmarkAnalogIO() {
  printHeader("I/O: ANALOG OPERATIONS");

  const uint32_t minDurationMs = max(5UL, (gMinBenchUs + 999UL) / 1000UL);

  // Find analog pins
#if defined(ESP32)
#if defined(ARDUINO_NANO_ESP32)
  int analogInPin = A0;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  int analogInPin = 1;
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  int analogInPin = 1;
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  int analogInPin = 0;
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
  int analogInPin = 0;
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
  int analogInPin = 0;
#else
  int analogInPin = 36;
#endif
#if defined(CONFIG_IDF_TARGET_ESP32)
  int analogOutPin = 25;  // DAC1
#else
  int analogOutPin = -1;
#endif

#elif defined(ESP8266)
  int analogInPin = A0;
  int analogOutPin = -1;

#elif defined(ARDUINO_UNO_Q) || defined(ARDUINO_UNO_Q_MCU)
  // UNO Q does have A0..A5 on the header. Keep output format identical to other boards:
  // single analogRead benchmark on a representative pin.
  int analogInPin = A0;
  int analogOutPin = -1;  // no DAC; treat PWM separately elsewhere

// Make ADC resolution deterministic if the core supports it.
#if defined(analogReadResolution)
  analogReadResolution(12);
#endif

#elif defined(__AVR__)
  int analogInPin = A0;
  int analogOutPin = 9;  // PWM

#elif defined(ARDUINO_ARCH_RP2040)
  int analogInPin = 26;   // A0
  int analogOutPin = 15;  // PWM

#else
  int analogInPin = A0;
  int analogOutPin = 3;
#endif

  // ------------------------------
  // analogRead benchmark
  // ------------------------------
  if (analogInPin >= 0) {
    // Some cores behave better with explicit INPUT and a short warm-up.
    pinMode(analogInPin, INPUT);
    for (int i = 0; i < 8; i++) (void)analogRead(analogInPin);

    volatile uint32_t sum = 0;
    bool allZero = true;

    TimedLoopResult readResult = runTimedLoop(minDurationMs, 1, [&]() {
      int value = analogRead(analogInPin);
      sum += (uint32_t)((value < 0) ? 0 : value);
      if (value != 0) allZero = false;
    });

    SERIAL_OUT.print(F_STR("analogRead() ("));
    SERIAL_OUT.print(readResult.totalOps);
    SERIAL_OUT.print(F_STR(" ops): "));
    SERIAL_OUT.print(readResult.elapsedMicros);
    SERIAL_OUT.print(F_STR(" μs ("));
    SERIAL_OUT.print(readResult.opsPerMs);
    SERIAL_OUT.println(F_STR(" ops/ms)"));

    SERIAL_OUT.print(F_STR("ADC average: "));
    SERIAL_OUT.println((uint32_t)(sum / readResult.totalOps));

    if (allZero) {
      SERIAL_OUT.println(F_STR("Warning: ADC all zero"));
    }
  }

  // ------------------------------
  // analogWrite/PWM benchmark
  // ------------------------------
  if (analogOutPin >= 0) {
#if defined(ESP32)
    const int pwmFreq = 5000;
    const int pwmResolution = 8;
    const int pwmChannel = 0;

    startBenchmark();
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)ledcAttachChannel(analogOutPin, pwmFreq, pwmResolution, pwmChannel);
#else
    ledcSetup(pwmChannel, pwmFreq, pwmResolution);
    ledcAttachPin(analogOutPin, pwmChannel);
#endif
    unsigned long setupTime = endBenchmark();

    uint32_t pwmValue = 0;
    TimedLoopResult updateResult = runTimedLoop(minDurationMs, 1, [&]() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
      ledcWrite(pwmChannel, pwmValue % 256);
#else
        ledcWrite(pwmChannel, pwmValue % 256);
#endif
      pwmValue++;
    });

    SERIAL_OUT.print(F_STR("PWM setup: "));
    SERIAL_OUT.print(setupTime);
    SERIAL_OUT.print(F_STR(" μs ("));
    SERIAL_OUT.print(1000.0 / (double)setupTime);
    SERIAL_OUT.println(F_STR(" ops/ms)"));

    SERIAL_OUT.print(F_STR("PWM duty update ("));
    SERIAL_OUT.print(updateResult.totalOps);
    SERIAL_OUT.print(F_STR(" ops): "));
    SERIAL_OUT.print(updateResult.elapsedMicros);
    SERIAL_OUT.print(F_STR(" μs ("));
    SERIAL_OUT.print(updateResult.opsPerMs);
    SERIAL_OUT.println(F_STR(" ops/ms)"));
#else
    pinMode(analogOutPin, OUTPUT);

    uint32_t pwmValue = 0;
    TimedLoopResult writeResult = runTimedLoop(minDurationMs, 1, [&]() {
      analogWrite(analogOutPin, pwmValue % 256);
      pwmValue++;
    });

    SERIAL_OUT.print(F_STR("analogWrite() ("));
    SERIAL_OUT.print(writeResult.totalOps);
    SERIAL_OUT.print(F_STR(" ops): "));
    SERIAL_OUT.print(writeResult.elapsedMicros);
    SERIAL_OUT.print(F_STR(" μs ("));
    SERIAL_OUT.print(writeResult.opsPerMs);
    SERIAL_OUT.println(F_STR(" ops/ms)"));
#endif
  }
}
#endif  // BOARD_SMALL_FLASH

void benchmarkFlash() {
  printHeader("STORAGE: Flash Information");

#if defined(ESP32)
  SERIAL_OUT.print(F_STR("Flash Size: "));
  SERIAL_OUT.print(ESP.getFlashChipSize() / 1024);
  SERIAL_OUT.println(F_STR(" KB"));

  SERIAL_OUT.print(F_STR("Flash Speed: "));
  SERIAL_OUT.print(ESP.getFlashChipSpeed() / 1000000);
  SERIAL_OUT.println(F_STR(" MHz"));

  SERIAL_OUT.print(F_STR("Sketch Size: "));
  SERIAL_OUT.print(ESP.getSketchSize() / 1024);
  SERIAL_OUT.println(F_STR(" KB"));

  SERIAL_OUT.print(F_STR("Free Sketch Space: "));
  SERIAL_OUT.print(ESP.getFreeSketchSpace() / 1024);
  SERIAL_OUT.println(F_STR(" KB"));
#elif defined(ESP8266)
  SERIAL_OUT.print(F_STR("Flash Size: "));
  SERIAL_OUT.print(ESP.getFlashChipSize() / 1024);
  SERIAL_OUT.println(F_STR(" KB"));

  SERIAL_OUT.print(F_STR("Flash Speed: "));
  SERIAL_OUT.print(ESP.getFlashChipSpeed() / 1000000);
  SERIAL_OUT.println(F_STR(" MHz"));

  SERIAL_OUT.print(F_STR("Sketch Size: "));
  SERIAL_OUT.print(ESP.getSketchSize() / 1024);
  SERIAL_OUT.println(F_STR(" KB"));

  SERIAL_OUT.print(F_STR("Free Sketch Space: "));
  SERIAL_OUT.print(ESP.getFreeSketchSpace() / 1024);
  SERIAL_OUT.println(F_STR(" KB"));
#elif defined(ARDUINO_UNO_Q)
  // STM32U585 flash size register at 0x0BFA07A0 (value in KB)
  uint16_t flashSizeKB = *((volatile uint16_t *)0x0BFA07A0);
  SERIAL_OUT.print(F_STR("Flash Size (register): "));
  SERIAL_OUT.print(flashSizeKB);
  SERIAL_OUT.println(F_STR(" KB"));

#if defined(CONFIG_FLASH)
  // Also try Zephyr flash API for page geometry
  const struct device *flashDev = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
  if (device_is_ready(flashDev)) {
    const struct flash_parameters *fp = flash_get_parameters(flashDev);
    if (fp) {
      SERIAL_OUT.print(F_STR("Write block size: "));
      SERIAL_OUT.print((unsigned long)fp->write_block_size);
      SERIAL_OUT.println(F_STR(" bytes"));
      SERIAL_OUT.print(F_STR("Erase value: 0x"));
      SERIAL_OUT.println(fp->erase_value, HEX);
    }
    size_t totalSize = flash_get_write_block_size(flashDev);
    SERIAL_OUT.print(F_STR("Flash write-block: "));
    SERIAL_OUT.print((unsigned long)totalSize);
    SERIAL_OUT.println(F_STR(" bytes"));
  } else {
    SERIAL_OUT.println(F_STR("Zephyr flash device not ready"));
  }
#endif

#else
  SERIAL_OUT.println(F_STR("Flash info not available on this platform"));
#endif
}

// ==================== GENERAL BENCHMARKS (ALL BOARDS) ====================

// PWM Benchmark (all boards with analogWrite)
void benchmarkPWM() {
  printHeader("I/O: PWM PERFORMANCE");

  // Find a PWM-capable pin
  int pwmPin;
#if defined(ESP32)
  pwmPin = 2;
#elif defined(ARDUINO_ARCH_RP2040)
  pwmPin = 25;
#elif defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  pwmPin = 3;
#elif defined(__AVR__)
  pwmPin = 9;
#else
  pwmPin = 3;
#endif

  pinMode(pwmPin, OUTPUT);

  SERIAL_OUT.print(F_STR("Testing PWM on pin "));
  SERIAL_OUT.println(pwmPin);

  // PWM update benchmark
#if defined(ESP32)
  const int pwmFreq = 5000;
  const int pwmResolution = 8;
  const int pwmChannel = 0;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pwmPin, pwmFreq, pwmResolution, pwmChannel);
#else
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);
#endif
#endif

  volatile uint8_t duty = 0;
  startBenchmark();
  for (int i = 0; i < 10000; i++) {
#if defined(ESP32)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pwmChannel, duty);
#else
    ledcWrite(pwmChannel, duty);
#endif
#else
    analogWrite(pwmPin, duty);
#endif
    duty++;
  }
  unsigned long pwmTime = endBenchmark();

#if defined(ESP32)
  SERIAL_OUT.print(F_STR("LEDC duty update (10000 ops): "));
#else
  SERIAL_OUT.print(F_STR("analogWrite() (10000 ops): "));
#endif
  SERIAL_OUT.print(pwmTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(10000.0 / pwmTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  // PWM ramp timing
  startBenchmark();
  for (int cycle = 0; cycle < 100; cycle++) {
    for (int val = 0; val < 256; val++) {
#if defined(ESP32)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
      ledcWrite(pwmChannel, val);
#else
      ledcWrite(pwmChannel, val);
#endif
#else
      analogWrite(pwmPin, val);
#endif
    }
  }
  unsigned long rampTime = endBenchmark();

  SERIAL_OUT.print(F_STR("PWM ramp (100 cycles): "));
  SERIAL_OUT.print(rampTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(25600.0 / rampTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  // Time per update
  SERIAL_OUT.print(F_STR("Time per update: "));
  SERIAL_OUT.print(rampTime / 25600.0, 3);
  SERIAL_OUT.println(F_STR(" us"));

#if defined(ESP32)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pwmChannel, 0);
#else
  ledcWrite(pwmChannel, 0);
#endif
#else
  analogWrite(pwmPin, 0);
#endif
}

// Interrupt Latency Benchmark
volatile unsigned long isrStartTime = 0;
volatile unsigned long isrEndTime = 0;
volatile bool isrFired = false;

void latencyISR() {
  isrEndTime = micros();
  isrFired = true;
}

void benchmarkInterruptLatency() {
  printHeader("TIMING: INTERRUPT LATENCY");

  // Find suitable pins for interrupt test
  int triggerPin, interruptPin;

#if defined(ESP32)
  triggerPin = 5;
  interruptPin = 4;
#elif defined(ARDUINO_ARCH_RP2040)
  triggerPin = 3;
  interruptPin = 2;
#elif defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  triggerPin = 3;
  interruptPin = 2;
#elif defined(__AVR__)
  triggerPin = 3;
  interruptPin = 2;  // INT0
#else
  triggerPin = 3;
  interruptPin = 2;
#endif

  SERIAL_OUT.print(F_STR("Jumper "));
  SERIAL_OUT.print(triggerPin);
  SERIAL_OUT.print(F_STR(" -> "));
  SERIAL_OUT.println(interruptPin);

  SERIAL_OUT.print(F_STR("Using pin "));
  SERIAL_OUT.println(interruptPin);

  int interruptNumber = digitalPinToInterrupt(interruptPin);
  if (interruptNumber == NOT_AN_INTERRUPT) {
    SERIAL_OUT.println(F_STR("Interrupt measurement skipped"));
    SERIAL_OUT.println(F_STR("(digitalPinToInterrupt returned NOT_AN_INTERRUPT)"));
    SERIAL_OUT.println(F_STR("Check board core/variant selection or pin choice logic."));
    return;
  }

  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);

  // INPUT_PULLDOWN not available on AVR - use INPUT_PULLUP with FALLING edge instead
#if defined(__AVR__)
  pinMode(interruptPin, INPUT_PULLUP);
  digitalWrite(triggerPin, HIGH);  // Start high for FALLING edge test
  attachInterrupt(interruptNumber, latencyISR, FALLING);
#else
  pinMode(interruptPin, INPUT_PULLDOWN);
  attachInterrupt(interruptNumber, latencyISR, RISING);
#endif

  // Measure interrupt latency
  unsigned long totalLatency = 0;
  int successfulMeasurements = 0;
  const int iterations = 100;

  for (int i = 0; i < iterations; i++) {
    isrFired = false;

    // Trigger the interrupt
    isrStartTime = micros();
#if defined(__AVR__)
    digitalWrite(triggerPin, LOW);  // FALLING edge for AVR
#else
    digitalWrite(triggerPin, HIGH);  // RISING edge for others
#endif

    // Wait for ISR (with timeout)
    unsigned long timeout = micros() + 1000;  // 1ms timeout
    while (!isrFired && micros() < timeout) {
      // Spin
    }

#if defined(__AVR__)
    digitalWrite(triggerPin, HIGH);  // Reset for next FALLING edge
#else
    digitalWrite(triggerPin, LOW);   // Reset for next RISING edge
#endif

    if (isrFired) {
      unsigned long latency = isrEndTime - isrStartTime;
      totalLatency += latency;
      successfulMeasurements++;
    }

    delayMicroseconds(100);  // Small delay between tests
  }

  detachInterrupt(interruptNumber);

  if (successfulMeasurements > 0) {
    float avgLatency = (float)totalLatency / successfulMeasurements;

    SERIAL_OUT.print(F_STR("Successful measurements: "));
    SERIAL_OUT.print(successfulMeasurements);
    SERIAL_OUT.print(F_STR("/"));
    SERIAL_OUT.println(iterations);

    SERIAL_OUT.print(F_STR("Average interrupt latency: "));
    SERIAL_OUT.print(avgLatency, 2);
    SERIAL_OUT.println(F_STR(" us"));

    SERIAL_OUT.println();
    SERIAL_OUT.println(F_STR("Note: Includes digitalWrite + ISR entry overhead"));
  } else {
    SERIAL_OUT.println(F_STR("Interrupt measurement failed"));
    SERIAL_OUT.println(F_STR("(Pin may not support interrupts)"));
  }
}

// SPI Loopback Benchmark
void benchmarkSPI() {
  printHeader("I/O: SPI PERFORMANCE");

  SERIAL_OUT.println(F_STR("Testing SPI transaction speed"));
  SERIAL_OUT.println(F_STR("(No loopback - measuring CPU overhead)"));
  SERIAL_OUT.println();

  SPI.begin();

  // Test different SPI speeds
#if defined(ESP32)
  uint32_t speeds[] = { 1000000, 4000000, 10000000, 20000000, 40000000 };
  const char *speedNames[] = { "1 MHz", "4 MHz", "10 MHz", "20 MHz", "40 MHz" };
  int numSpeeds = 5;
#elif defined(ARDUINO_ARCH_RP2040)
  uint32_t speeds[] = { 1000000, 4000000, 10000000, 20000000, 62500000 };
  const char *speedNames[] = { "1 MHz", "4 MHz", "10 MHz", "20 MHz", "62.5 MHz" };
  int numSpeeds = 5;
#elif defined(__AVR__)
  uint32_t speeds[] = { 1000000, 2000000, 4000000, 8000000 };
  const char *speedNames[] = { "1 MHz", "2 MHz", "4 MHz", "8 MHz" };
  int numSpeeds = 4;
#else
  uint32_t speeds[] = { 1000000, 4000000, 8000000, 16000000 };
  const char *speedNames[] = { "1 MHz", "4 MHz", "8 MHz", "16 MHz" };
  int numSpeeds = 4;
#endif

  for (int s = 0; s < numSpeeds; s++) {
    SPISettings settings(speeds[s], MSBFIRST, SPI_MODE0);

    volatile uint8_t result = 0;
    startBenchmark();
    for (int i = 0; i < 1000; i++) {
      SPI.beginTransaction(settings);
      result += SPI.transfer(i & 0xFF);
      SPI.endTransaction();
    }
    unsigned long xferTime = endBenchmark();

    SERIAL_OUT.print(speedNames[s]);
    SERIAL_OUT.print(F_STR(": "));
    SERIAL_OUT.print(xferTime);
    SERIAL_OUT.print(F_STR(" us for 1000 bytes ("));
    SERIAL_OUT.print(1000.0 / xferTime * 1000);
    SERIAL_OUT.println(F_STR(" KB/s effective)"));
  }

  // Bulk transfer benchmark
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("Bulk transfer (256 bytes):"));

  uint8_t txBuffer[256];
  for (int i = 0; i < 256; i++) txBuffer[i] = i;

#if defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
  uint32_t bulkSpeed = 10000000;
#else
  uint32_t bulkSpeed = 4000000;
#endif

  SPISettings bulkSettings(bulkSpeed, MSBFIRST, SPI_MODE0);

  startBenchmark();
  for (int i = 0; i < 100; i++) {
    SPI.beginTransaction(bulkSettings);
    SPI.transfer(txBuffer, 256);
    SPI.endTransaction();
  }
  unsigned long bulkTime = endBenchmark();

  float throughput = (256.0 * 100) / (bulkTime / 1000000.0) / 1024;
  SERIAL_OUT.print(F_STR("Throughput: "));
  SERIAL_OUT.print(throughput, 1);
  SERIAL_OUT.println(F_STR(" KB/s"));

  SPI.end();
}
// ==================== SYSTEM INFO ====================

void printSystemInfo() {
  printHeader("SYSTEM INFORMATION");

  SERIAL_OUT.print(F_STR("Board: "));
  SERIAL_OUT.println(BOARD_NAME);

#if defined(ESP32)
  SERIAL_OUT.print(F_STR("Chip Model: "));
  SERIAL_OUT.println(ESP.getChipModel());
  SERIAL_OUT.print(F_STR("Chip Revision: "));
  SERIAL_OUT.println(ESP.getChipRevision());
  SERIAL_OUT.print(F_STR("CPU Frequency: "));
  SERIAL_OUT.print(ESP.getCpuFreqMHz());
  SERIAL_OUT.println(F_STR(" MHz"));
  SERIAL_OUT.print(F_STR("Cores: "));
  SERIAL_OUT.println(ESP.getChipCores());
  SERIAL_OUT.print(F_STR("SDK Version: "));
  SERIAL_OUT.println(ESP.getSdkVersion());
#elif defined(ESP8266)
  SERIAL_OUT.print(F_STR("CPU Frequency: "));
  SERIAL_OUT.print(ESP.getCpuFreqMHz());
  SERIAL_OUT.println(F_STR(" MHz"));
  SERIAL_OUT.print(F_STR("Chip ID: "));
  SERIAL_OUT.println(ESP.getChipId());
  SERIAL_OUT.print(F_STR("SDK Version: "));
  SERIAL_OUT.println(ESP.getSdkVersion());
#elif defined(ARDUINO_ARCH_RP2040)
  SERIAL_OUT.print(F_STR("CPU Frequency: "));
  SERIAL_OUT.print(F_CPU / 1000000);
  SERIAL_OUT.println(F_STR(" MHz"));
#if defined(ARDUINO_NANO_RP2040_CONNECT)
  SERIAL_OUT.println(F_STR("Features: WiFi (Nina W102), BLE, IMU (LSM6DSOX), Mic"));
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
  SERIAL_OUT.println(F_STR("Features: WiFi (CYW43439)"));
#endif
#elif defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  SERIAL_OUT.print(F_STR("MCU: Renesas RA4M1 (ARM Cortex-M4)"));
  SERIAL_OUT.println();
  SERIAL_OUT.print(F_STR("CPU Frequency: "));
  SERIAL_OUT.print(F_CPU / 1000000);
  SERIAL_OUT.println(F_STR(" MHz"));
#ifdef ARDUINO_UNOR4_WIFI
  SERIAL_OUT.println(F_STR("Features: WiFi (ESP32-S3), 12x8 LED Matrix"));
#else
  SERIAL_OUT.println(F_STR("Features: 12x8 LED Matrix"));
#endif
#elif defined(BOARD_SAMD)
  SERIAL_OUT.print(F_STR("MCU: SAMD (ARM Cortex-M0+)"));
  SERIAL_OUT.println();
  SERIAL_OUT.print(F_STR("CPU Frequency: "));
  SERIAL_OUT.print(F_CPU / 1000000);
  SERIAL_OUT.println(F_STR(" MHz"));
#if defined(ARDUINO_SAMD_NANO_33_IOT)
  SERIAL_OUT.println(F_STR("Features: WiFi, BLE, IMU (LSM6DS3)"));
#elif defined(ARDUINO_SAMD_MKRWIFI1010)
  SERIAL_OUT.println(F_STR("Features: WiFi (Nina W102), Crypto (ECC508)"));
#endif
#elif defined(BOARD_NRF52)
  SERIAL_OUT.print(F_STR("MCU: nRF52840 (ARM Cortex-M4F)"));
  SERIAL_OUT.println();
  SERIAL_OUT.print(F_STR("CPU Frequency: 64 MHz"));
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("Features: BLE 5.0, IMU (LSM9DS1)"));
#elif defined(BOARD_STM32H7)
  SERIAL_OUT.print(F_STR("MCU: STM32H7 (ARM Cortex-M7)"));
  SERIAL_OUT.println();
  SERIAL_OUT.print(F_STR("CPU Frequency: "));
  SERIAL_OUT.print(F_CPU / 1000000);
  SERIAL_OUT.println(F_STR(" MHz"));
#ifdef HAS_DUAL_CORE
  SERIAL_OUT.println(F_STR("Cores: Dual Core (M7 + M4)"));
#endif
#elif defined(BOARD_TEENSY)
  SERIAL_OUT.print(F_STR("CPU Frequency: "));
  SERIAL_OUT.print(F_CPU / 1000000);
  SERIAL_OUT.println(F_STR(" MHz"));
#if defined(__IMXRT1062__)
  SERIAL_OUT.println(F_STR("MCU: i.MX RT1062 (ARM Cortex-M7)"));
#endif
#elif defined(BOARD_STM32U5)
  SERIAL_OUT.println(F_STR("MCU: STM32U585 (ARM Cortex-M33)"));
  SERIAL_OUT.print(F_STR("MCU Frequency: "));

  // Try to get actual clock frequency
#if defined(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC)
  SERIAL_OUT.print(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000000);
#elif defined(F_CPU)
  SERIAL_OUT.print(F_CPU / 1000000);
#else
  SERIAL_OUT.print(F_STR("160"));  // STM32U585 default max frequency
#endif

  SERIAL_OUT.println(F_STR(" MHz"));
  SERIAL_OUT.println(F_STR("MCU Features: FPU, DSP, TrustZone"));
  SERIAL_OUT.println(F_STR("MCU RAM: 786 KB SRAM"));
  SERIAL_OUT.println(F_STR("MCU Flash: 2 MB"));
#ifdef USING_ZEPHYR
  SERIAL_OUT.println(F_STR("MCU RTOS: Zephyr"));
#endif
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("Linux MPU: Qualcomm QRB2210"));
  SERIAL_OUT.println(F_STR("MPU: Quad Cortex-A53 @ up to 2.0 GHz"));
  SERIAL_OUT.println(F_STR("MPU RAM: 2-4 GB"));
  SERIAL_OUT.println(F_STR("MPU Storage: 16 GB eMMC"));
  SERIAL_OUT.println(F_STR("MPU OS: Debian-based Linux"));
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("Communication: Arduino Bridge RPC (MCU<->Linux)"));
#ifdef HAS_ROUTER_BRIDGE
  SERIAL_OUT.println(F_STR("RPC Library: Arduino_RouterBridge (MessagePack)"));
#elif defined(HAS_RPCLITE)
  SERIAL_OUT.println(F_STR("RPC Library: Arduino_RPClite (MessagePack)"));
#else
  SERIAL_OUT.println(F_STR("RPC Library: Not detected"));
#endif
#else
  SERIAL_OUT.print(F_STR("CPU Frequency: "));
#if defined(F_CPU)
  SERIAL_OUT.print(F_CPU / 1000000);
  SERIAL_OUT.println(F_STR(" MHz"));
#else
  SERIAL_OUT.println(F_STR("Unknown"));
#endif
#endif

#if defined(ARDUINO_AVR_MULTIDUINO)
  SERIAL_OUT.println(F_STR("Features: RTC (DS1307)"));
#endif

#if defined(ARDUINO_AVR_YUN)
  SERIAL_OUT.println(F_STR("Linux: AR9331 400MHz, 64MB RAM, 16MB Flash"));
  SERIAL_OUT.println(F_STR("WiFi + USB Host (OpenWrt)"));
#endif

  // RAM Info
  SERIAL_OUT.print(F_STR("Free RAM: "));
#if defined(ESP32)
  SERIAL_OUT.print(ESP.getFreeHeap() / 1024);
  SERIAL_OUT.println(F_STR(" KB"));
  SERIAL_OUT.print(F_STR("Total Heap: "));
  SERIAL_OUT.print(ESP.getHeapSize() / 1024);
  SERIAL_OUT.println(F_STR(" KB"));
  SERIAL_OUT.print(F_STR("Min Free Heap: "));
  SERIAL_OUT.print(ESP.getMinFreeHeap() / 1024);
  SERIAL_OUT.println(F_STR(" KB"));
#elif defined(ESP8266)
  SERIAL_OUT.print(ESP.getFreeHeap() / 1024);
  SERIAL_OUT.println(F_STR(" KB"));
#elif defined(__AVR__)
  extern int __heap_start, *__brkval;
  int v;
  int freeRam = (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
  SERIAL_OUT.print(freeRam);
  SERIAL_OUT.println(F_STR(" bytes"));
// Show total RAM for common AVR boards
#if defined(__AVR_ATmega328P__)
  SERIAL_OUT.println(F_STR("Total RAM: 2 KB"));
#elif defined(__AVR_ATmega2560__)
  SERIAL_OUT.println(F_STR("Total RAM: 8 KB"));
#elif defined(__AVR_ATmega32U4__)
  SERIAL_OUT.println(F_STR("Total RAM: 2.5 KB"));
#endif
#elif defined(ARDUINO_SAM_DUE)
  extern char _end;
  extern "C" char *sbrk(int i);
  char *ramstart = (char *)0x20070000;
  char *ramend = (char *)0x20088000;
  int freeRam = ramend - sbrk(0);
  SERIAL_OUT.print(freeRam / 1024);
  SERIAL_OUT.println(F_STR(" KB"));
  SERIAL_OUT.println(F_STR("Total RAM: 96 KB"));
#elif defined(ARDUINO_ARCH_RP2040)
  // RP2040 has 264KB RAM
  SERIAL_OUT.println(F_STR("~264 KB (RP2040)"));
#elif defined(BOARD_NRF52)
  SERIAL_OUT.println(F_STR("~256 KB (nRF52840)"));
#elif defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  SERIAL_OUT.println(F_STR("32 KB (RA4M1)"));
#elif defined(BOARD_SRAM_KB)
  SERIAL_OUT.print(F_STR("Total RAM: "));
  SERIAL_OUT.print(BOARD_SRAM_KB);
  SERIAL_OUT.println(F_STR(" KB"));
#else
  SERIAL_OUT.println(F_STR("Unknown"));
#endif

  SERIAL_OUT.print(F_STR("Compile Date: "));
  SERIAL_OUT.print(__DATE__);
  SERIAL_OUT.print(F_STR(" "));
  SERIAL_OUT.println(__TIME__);
}

// ==================== MAIN FUNCTIONS ====================

void setup() {
#if defined(ARDUINO_UNO_Q)
  SERIAL_OUT.begin();  // Uno Q Monitor doesn't use baud rate
  delay(3000);         // Wait longer for Monitor connection
#elif defined(ESP32)
  SERIAL_OUT.begin(SERIAL_BAUD);
  delay(3000);  // ESP32 needs extra time for serial initialization
#else
  SERIAL_OUT.begin(SERIAL_BAUD);
#if defined(USBCON)
  {
    unsigned long waitStart = millis();
    while (!SERIAL_OUT && (millis() - waitStart < 10000)) {
      delay(10);
    }
  }
#endif
  delay(2000);  // Wait for serial connection
#endif

  calibrateBenchmarkTime();

  SERIAL_OUT.println();
  SERIAL_OUT.println();
  printDivider();
  SERIAL_OUT.println(F_STR("  UNIVERSAL ARDUINO BENCHMARK SUITE"));
  SERIAL_OUT.println(F_STR("  Part 1: Core Performance"));
  printDivider();
  SERIAL_OUT.println();

  // System info
  printSystemInfo();

  // CPU benchmarks
  benchmarkIntegerOps();
  benchmarkFloatOps();
#ifndef BOARD_SMALL_FLASH
  benchmarkStringOps();
#endif

  // Memory benchmarks
  benchmarkSRAM();
#if defined(EEPROM_h) || defined(ESP32) || defined(ESP8266)
  benchmarkEEPROM();
#endif
#ifdef HAS_PSRAM
  benchmarkPSRAM();
#endif

  // I/O benchmarks
  benchmarkDigitalIO();
#ifndef BOARD_SMALL_FLASH
  benchmarkAnalogIO();
#endif

  // Storage info
  benchmarkFlash();

  // General hardware I/O
  benchmarkPWM();
  benchmarkInterruptLatency();
  benchmarkSPI();

  // Final summary
  printHeader("PART 1 BENCHMARK COMPLETE!");
  SERIAL_OUT.println(F_STR("Core benchmarks finished."));
  SERIAL_OUT.println(F_STR("Upload Part 2 for remaining benchmarks."));
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("To run again, press the RESET button or"));
  SERIAL_OUT.println(F_STR("re-upload the sketch."));
  printDivider();
}

void loop() {
  // Nothing to do after benchmarks complete
}
