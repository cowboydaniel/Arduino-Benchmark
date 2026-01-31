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
#define BOARD_NAME "INK4u Board"
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

void benchmarkCPUStress() {
  printHeader("CPU: STRESS TEST with Temperature");

  // Check if temperature sensor is available
  bool hasTempSensor = false;
#if defined(ESP32)
  hasTempSensor = true;
#elif defined(ARDUINO_ARCH_RP2040)
  hasTempSensor = true;
#elif defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  hasTempSensor = false;  // RA4M1 has sensor but not easily accessible
#elif defined(BOARD_TEENSY) && defined(__IMXRT1062__)
  hasTempSensor = true;  // Teensy 4.x has temp sensor
#endif

  if (!hasTempSensor) {
    SERIAL_OUT.println(F_STR("Temperature sensor not available on this board"));
    SERIAL_OUT.println(F_STR("Running stress test without temperature monitoring..."));
    SERIAL_OUT.println();
  }

  // Initial temperature reading
  float startTemp = 0;
#if defined(ESP32)
  startTemp = temperatureRead();
  SERIAL_OUT.print(F_STR("Start Temperature: "));
  SERIAL_OUT.print(startTemp);
  SERIAL_OUT.println(F_STR(" °C"));
#elif defined(ARDUINO_ARCH_RP2040)
  startTemp = analogReadTemp(3.3f);
  SERIAL_OUT.print(F_STR("Start Temperature: "));
  SERIAL_OUT.print(startTemp);
  SERIAL_OUT.println(F_STR(" °C"));
#elif defined(BOARD_TEENSY) && defined(__IMXRT1062__)
  startTemp = tempmonGetTemp();
  SERIAL_OUT.print(F_STR("Start Temperature: "));
  SERIAL_OUT.print(startTemp);
  SERIAL_OUT.println(F_STR(" °C"));
#endif

  SERIAL_OUT.print(F_STR("Running intensive computation for 10 seconds..."));
  SERIAL_OUT.println();

  // CPU stress test - run all cores
  unsigned long stressStart = millis();
  unsigned long lastService = millis();
#if defined(ARDUINO_ARCH_RP2040)
  unsigned long lastIo = millis();
#endif
  unsigned long iterations = 0;
  volatile float result = 1.0f;
  const float twoPi = 6.2831853f;
#if defined(ARDUINO_ARCH_RP2040)
  uint32_t lcg = 0x12345678u;
#endif

  while (millis() - stressStart < 10000) {
    // Mix of integer and float operations
    for (int i = 0; i < 100; i++) {
#if defined(ARDUINO_ARCH_RP2040)
      lcg = lcg * 1664525u + 1013904223u;
      uint32_t mixed = lcg ^ (lcg >> 16);
      float f1 = (mixed & 0xFFFF) * 0.0001f;
      float f2 = (mixed & 0xFF) * 0.00001f;
      result = result * 1.0001f + f1;
      result = result * 0.9999f + f2;
#else
#if defined(ARDUINO_UNO_Q) || defined(ARDUINO_UNO_Q_MCU)
      // Keep it deterministic and heavy without libm (no sin/cos/fmod/sqrt)
      // Mix float + integer-ish noise
      uint32_t x = (uint32_t)iterations * 1664525u + 1013904223u;
      float f1 = (float)(x & 0xFFFFu) * 0.0001f;
      float f2 = (float)((x >> 16) & 0xFFFFu) * 0.00005f;
      result = result * 1.00013f + f1;
      result = result * 0.99991f + f2;
#else
      result = result * 1.0001f + sqrtf((float)i);
#if defined(__ZEPHYR__) || defined(BOARD_STM32U5)
      // Manual fmod implementation for Zephyr (avoids libm linking issues)
      float divisor = twoPi;
      result = result - ((int)(result / divisor)) * divisor;
#else
      result = fmodf(result, twoPi);
#endif
      result = sinf(result) + cosf(result);
#endif
#endif
      iterations++;
    }

    // Periodic yield to prevent watchdog timeout (but don't print)
    if (millis() - lastService >= 5) {
#if defined(ARDUINO_ARCH_RP2040) || defined(ESP32) || defined(ESP8266)
      yield();
#if defined(ESP32) || defined(ESP8266)
      delay(0);
#endif
#endif
      lastService = millis();
    }

#if defined(ARDUINO_ARCH_RP2040)
    if (millis() - lastIo >= 250) {
      SERIAL_OUT.write('.');
      lastIo = millis();
    }
#endif
  }

  unsigned long stressDuration = millis() - stressStart;

  SERIAL_OUT.println();
  SERIAL_OUT.print(F_STR("Stress test complete: "));
  SERIAL_OUT.print(iterations);
  SERIAL_OUT.print(F_STR(" iterations in "));
  SERIAL_OUT.print(stressDuration);
  SERIAL_OUT.println(F_STR(" ms"));

  SERIAL_OUT.print(F_STR("Performance: "));
  SERIAL_OUT.print((float)iterations / stressDuration);
  SERIAL_OUT.println(F_STR(" iterations/ms"));

  // Final temperature reading
  if (hasTempSensor) {
    delay(100);  // Let sensor stabilize
    float endTemp = 0;
#if defined(ESP32)
    endTemp = temperatureRead();
#elif defined(ARDUINO_ARCH_RP2040)
    endTemp = analogReadTemp(3.3f);
#elif defined(BOARD_TEENSY) && defined(__IMXRT1062__)
    endTemp = tempmonGetTemp();
#endif

    SERIAL_OUT.println();
    SERIAL_OUT.print(F_STR("Final Temperature: "));
    SERIAL_OUT.print(endTemp);
    SERIAL_OUT.println(F_STR(" °C"));

    SERIAL_OUT.print(F_STR("Total Temperature Gain: +"));
    SERIAL_OUT.print(endTemp - startTemp);
    SERIAL_OUT.println(F_STR(" °C"));

    if (endTemp - startTemp > 10) {
      SERIAL_OUT.println(F_STR("⚠️  Significant heating detected - ensure adequate cooling"));
    } else if (endTemp - startTemp > 5) {
      SERIAL_OUT.println(F_STR("ℹ️  Normal temperature increase under load"));
    } else {
      SERIAL_OUT.println(F_STR("✓ Minimal temperature increase - good thermal performance"));
    }
  }
}

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
}

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

  // snprintf into fixed buffer (no heap allocation)
  char fixedBuffer[32];
  volatile int snprintfTotal = 0;
  TimedLoopResult snprintfResult = runTimedLoop(minDurationMs, 1000, [&]() {
    for (int i = 0; i < 1000; i++) {
      snprintfTotal += snprintf(fixedBuffer, sizeof(fixedBuffer), "%d", i);
    }
  });

  SERIAL_OUT.print(F_STR("Arduino String (heap stress) - Concatenation ("));
  SERIAL_OUT.print(concatResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(concatResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(concatResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Arduino String (heap stress) - Comparison ("));
  SERIAL_OUT.print(cmpResultData.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(cmpResultData.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(cmpResultData.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Arduino String (heap stress) - Int to String ("));
  SERIAL_OUT.print(toStrResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(toStrResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(toStrResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("snprintf (fixed buffer, "));
  SERIAL_OUT.print(snprintfResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(snprintfResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(snprintfResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));
}

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
      SERIAL_OUT.println(F_STR("Warning: ADC reads were all zero; ADC pin may be invalid or tied low."));
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

void benchmarkSerial() {
  printHeader("I/O: SERIAL COMMUNICATION");

  // Calculate expected bytes
  int expectedBytes = 0;
  for (int i = 0; i < 100; i++) {
    expectedBytes += (i < 10) ? 1 : (i < 100) ? 2
                                              : 3;  // digit count
  }

  // Print benchmark - measure enqueue time (CPU overhead)
  MedianCollector<unsigned long, kJitterTrials> enqueueTimeMedian = {};
  MedianCollector<float, kJitterTrials> enqueueRateMedian = {};
  for (uint8_t trial = 0; trial < kJitterTrials; trial++) {
    SERIAL_OUT.flush();
    startBenchmark();
    for (int i = 0; i < 100; i++) {
      SERIAL_OUT.print(i);
    }
    unsigned long enqueueTime = endBenchmark();
    enqueueTimeMedian.add(enqueueTime);
    enqueueRateMedian.add(expectedBytes * 1000.0f / enqueueTime);
  }
  unsigned long enqueueTime = enqueueTimeMedian.median();
  float enqueueRate = enqueueRateMedian.median();

  // Measure what flush() actually does
  unsigned long flushStart = micros();
  SERIAL_OUT.flush();  // Wait for TX buffer to drain to UART FIFO
  unsigned long flushTime = micros() - flushStart;

  // Calculate theoretical wire time (10 bits per byte: start + 8 data + stop)
  // At 115200 baud: each bit = 1/115200 sec = 8.68 μs
  // Each byte = 10 bits = 86.8 μs
  unsigned long theoreticalWireTime = (unsigned long)(expectedBytes * 10 * 1000000.0 / SERIAL_BAUD);

  SERIAL_OUT.println();
  SERIAL_OUT.print(F_STR("Serial Enqueue ("));
  SERIAL_OUT.print(expectedBytes);
  SERIAL_OUT.print(F_STR(" bytes): "));
  SERIAL_OUT.print(enqueueTime);
  SERIAL_OUT.print(F_STR(" μs (median "));
  SERIAL_OUT.print(enqueueRate);
  SERIAL_OUT.print(F_STR(" bytes/ms CPU, "));
  SERIAL_OUT.print(kJitterTrials);
  SERIAL_OUT.println(F_STR(" trials)"));

  SERIAL_OUT.print(F_STR("flush() time (implementation-dependent): "));
  SERIAL_OUT.print(flushTime);
  SERIAL_OUT.print(F_STR(" μs"));
  SERIAL_OUT.println();

  SERIAL_OUT.print(F_STR("Theoretical Wire Time: "));
  SERIAL_OUT.print(theoreticalWireTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(theoreticalWireTime / 1000.0);
  SERIAL_OUT.println(F_STR(" ms)"));

  SERIAL_OUT.print(F_STR("Wire Throughput: "));
  SERIAL_OUT.print(expectedBytes * 1000.0 / theoreticalWireTime);
  SERIAL_OUT.print(F_STR(" bytes/ms ("));
  SERIAL_OUT.print(SERIAL_BAUD / 10);  // 8N1 = 10 bits per byte
  SERIAL_OUT.println(F_STR(" bytes/sec theoretical)"));

  // Compare enqueue vs wire
  SERIAL_OUT.print(F_STR("Enqueue/Wire Ratio: "));
  SERIAL_OUT.print((float)enqueueTime / theoreticalWireTime * 100);
  SERIAL_OUT.print(F_STR("% ("));
  if (enqueueTime < theoreticalWireTime) {
    SERIAL_OUT.println(F_STR("buffered, won't block)"));
  } else {
    SERIAL_OUT.println(F_STR("CPU-bound)"));
  }
}

// ==================== BOARD-SPECIFIC BENCHMARKS ====================

#ifdef HAS_LED_MATRIX
void benchmarkLEDMatrix() {
  printHeader("DISPLAY: LED Matrix (Uno R4)");

  SERIAL_OUT.println(F_STR("12x8 LED Matrix Available: YES"));
  SERIAL_OUT.println(F_STR("Running LED animation test..."));

  ArduinoLEDMatrix matrix;
  matrix.begin();

  // Test 1: Frame loading speed - all pixels on (with delays for visual effect)
  uint32_t frameOn[3] = {
    0xFFFFFFFF,
    0xFFFFFFFF,
    0xFFFFFFFF
  };
  uint32_t frameOff[3] = {
    0x00000000,
    0x00000000,
    0x00000000
  };

  unsigned long blinkStart = millis();
  for (int i = 0; i < 20; i++) {
    matrix.loadFrame(frameOn);
    delay(50);
    matrix.loadFrame(frameOff);
    delay(50);
  }
  unsigned long blinkTime = millis() - blinkStart;

  // Test 2: Pattern switching speed (no delays)
  uint32_t pattern1[3] = {
    0x55555555,
    0xAAAAAAAA,
    0x55555555
  };
  uint32_t pattern2[3] = {
    0xAAAAAAAA,
    0x55555555,
    0xAAAAAAAA
  };

  startBenchmark();
  for (int i = 0; i < 100; i++) {
    matrix.loadFrame(i % 2 == 0 ? pattern1 : pattern2);
  }
  unsigned long patternTime = endBenchmark();

  // Test 3: Bitmap rendering - must pass array name directly
  uint8_t bitmap[8][12] = {
    { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 },
    { 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 },
    { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 },
    { 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 },
    { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 },
    { 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 },
    { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 },
    { 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 }
  };

  startBenchmark();
  for (int i = 0; i < 50; i++) {
    matrix.renderBitmap(bitmap, 8, 12);
  }
  unsigned long bitmapTime = endBenchmark();

  matrix.loadFrame(frameOff);

  SERIAL_OUT.print(F_STR("Blink animation (20 cycles): "));
  SERIAL_OUT.print(blinkTime);
  SERIAL_OUT.println(F_STR(" ms"));

  SERIAL_OUT.print(F_STR("Pattern switching (100 frames): "));
  SERIAL_OUT.print(patternTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(100.0 / patternTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Bitmap rendering (50 frames): "));
  SERIAL_OUT.print(bitmapTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(50.0 / bitmapTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));
}
#endif

#ifdef HAS_WIFI
void benchmarkWiFi() {
  printHeader("WIRELESS: WiFi CAPABILITIES");

  SERIAL_OUT.print(F_STR("WiFi Available: YES"));

// Identify WiFi chip/library
#if defined(ARDUINO_UNOR4_WIFI)
  SERIAL_OUT.println(F_STR(" (ESP32-S3 via WiFiS3)"));
#elif defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_NANO_RP2040_CONNECT)
  SERIAL_OUT.println(F_STR(" (Nina W102 via WiFiNINA)"));
#elif defined(ARDUINO_SAMD_MKR1000)
  SERIAL_OUT.println(F_STR(" (WINC1500 via WiFi101)"));
#elif defined(ESP32) || defined(ESP8266)
  SERIAL_OUT.println(F_STR(" (Native)"));
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
  SERIAL_OUT.println(F_STR(" (CYW43439)"));
#else
  SERIAL_OUT.println();
#endif

// MAC Address - different methods for different libraries
#if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
  // ESP and Pico W need mode set first
  WiFi.mode(WIFI_STA);
  delay(100);
  SERIAL_OUT.print(F_STR("MAC Address: "));
  SERIAL_OUT.println(WiFi.macAddress());
#elif defined(ARDUINO_UNOR4_WIFI)
  // Uno R4 WiFi - no mode setting needed
  SERIAL_OUT.print(F_STR("MAC Address: "));
  byte mac[6];
  WiFi.macAddress(mac);
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) SERIAL_OUT.print("0");
    SERIAL_OUT.print(mac[i], HEX);
    if (i > 0) SERIAL_OUT.print(":");
  }
  SERIAL_OUT.println();
#else
  // WiFiNINA/WiFi101 - need to get MAC as byte array
  byte mac[6];
  WiFi.macAddress(mac);
  SERIAL_OUT.print(F_STR("MAC Address: "));
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) SERIAL_OUT.print("0");
    SERIAL_OUT.print(mac[i], HEX);
    if (i > 0) SERIAL_OUT.print(":");
  }
  SERIAL_OUT.println();
#endif

  // Scan networks
  SERIAL_OUT.print(F_STR("Scanning networks... "));
  int n = WiFi.scanNetworks();
  SERIAL_OUT.print(n);
  SERIAL_OUT.println(F_STR(" networks found"));

  if (n > 0) {
    SERIAL_OUT.println(F_STR("Strongest 3 networks:"));
    for (int i = 0; i < min(3, n); i++) {
      SERIAL_OUT.print(F_STR("  "));
      SERIAL_OUT.print(i + 1);
      SERIAL_OUT.print(F_STR(": "));
      SERIAL_OUT.print(WiFi.SSID(i));
      SERIAL_OUT.print(F_STR(" ("));
      SERIAL_OUT.print(WiFi.RSSI(i));
      SERIAL_OUT.println(F_STR(" dBm)"));
    }
  }

// Clean up - different methods for different libraries
#if defined(ESP32) || defined(ESP8266)
  WiFi.scanDelete();
  WiFi.mode(WIFI_OFF);
#elif defined(ARDUINO_UNOR4_WIFI)
  // WiFiS3 doesn't have scanDelete or mode
  // Just let it finish naturally
#else
  // WiFiNINA/WiFi101 - end() to clean up
  WiFi.end();
#endif
}
#endif

#ifdef HAS_BLE
void benchmarkBLE() {
  printHeader("WIRELESS: Bluetooth LE");
  SERIAL_OUT.println(F_STR("BLE Available: YES"));

#if defined(ESP32)
  SERIAL_OUT.println(F_STR("Initializing BLE..."));
  BLEDevice::init("");

  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  SERIAL_OUT.print(F_STR("Scanning for BLE devices (5 sec)... "));
  BLEScanResults *foundDevices = pBLEScan->start(5, false);
  int deviceCount = foundDevices->getCount();
  SERIAL_OUT.print(deviceCount);
  SERIAL_OUT.println(F_STR(" devices found"));

  if (deviceCount > 0) {
    SERIAL_OUT.println(F_STR("Discovered devices:"));
    for (int i = 0; i < min(5, deviceCount); i++) {
      BLEAdvertisedDevice device = foundDevices->getDevice(i);
      SERIAL_OUT.print(F_STR("  "));
      SERIAL_OUT.print(i + 1);
      SERIAL_OUT.print(F_STR(": "));
      if (device.haveName()) {
        SERIAL_OUT.print(device.getName().c_str());
      } else {
        SERIAL_OUT.print(F_STR("Unknown"));
      }
      SERIAL_OUT.print(F_STR(" ["));
      SERIAL_OUT.print(device.getAddress().toString().c_str());
      SERIAL_OUT.print(F_STR("] RSSI: "));
      SERIAL_OUT.print(device.getRSSI());
      SERIAL_OUT.println(F_STR(" dBm"));
    }
  }

  pBLEScan->clearResults();
  BLEDevice::deinit(false);

#elif defined(BOARD_NRF52) || defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_SAMD_NANO_33_IOT)
  // For boards using ArduinoBLE library
  SERIAL_OUT.println(F_STR("BLE scan requires ArduinoBLE library"));
  SERIAL_OUT.println(F_STR("Install via Library Manager: 'ArduinoBLE'"));
  SERIAL_OUT.println(F_STR("Scan example:"));
  SERIAL_OUT.println(F_STR("  BLE.begin() → BLE.scan() → check BLE.available()"));
#else
  SERIAL_OUT.println(F_STR("BLE hardware detected but scan not implemented"));
#endif
}
#endif

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
#else
  SERIAL_OUT.println(F_STR("Flash info not available on this platform"));
#endif
}

// ==================== ADVANCED MATH BENCHMARKS ====================

void benchmarkAdvancedMath() {
  printHeader("ADVANCED MATH: Transcendental Functions");

#if defined(ARDUINO_UNO_Q) || defined(ARDUINO_UNO_Q_MCU)
  SERIAL_OUT.println(F_STR("Uno Q: libm not available (sin/cos/log/exp/pow/atan2)"));
  SERIAL_OUT.println(F_STR("Skipping advanced math benchmark"));
  return;
#endif

  volatile float checksum = 0;

  // atan2 test
  startBenchmark();
  for (int i = 0; i < 100; i++) {
    checksum += atan2((float)i, (float)(100 - i));
  }
  unsigned long atan2Time = endBenchmark();

  // log test
  startBenchmark();
  for (int i = 1; i <= 100; i++) {
    checksum += log((float)i);
  }
  unsigned long logTime = endBenchmark();

  // exp test
  startBenchmark();
  for (int i = 0; i < 100; i++) {
    checksum += exp((float)i / 10.0f);
  }
  unsigned long expTime = endBenchmark();

  // pow test
  startBenchmark();
  for (int i = 1; i <= 100; i++) {
    checksum += pow((float)i, 1.5f);
  }
  unsigned long powTime = endBenchmark();

  // fmod test
  startBenchmark();
  for (int i = 0; i < 1000; i++) {
#if defined(__ZEPHYR__) || defined(BOARD_STM32U5)
    // Manual fmod for Zephyr (avoids libm linking issues)
    float val = (float)i;
    float divisor = 7.3f;
    checksum += val - ((int)(val / divisor)) * divisor;
#else
    checksum += fmod((float)i, 7.3f);
#endif
  }
  unsigned long fmodTime = endBenchmark();

  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println(checksum);

  SERIAL_OUT.print(F_STR("atan2() (100 ops): "));
  SERIAL_OUT.print(atan2Time);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(100.0 / atan2Time * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("log() (100 ops): "));
  SERIAL_OUT.print(logTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(100.0 / logTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("exp() (100 ops): "));
  SERIAL_OUT.print(expTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(100.0 / expTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("pow() (100 ops): "));
  SERIAL_OUT.print(powTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(100.0 / powTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("fmod() (1000 ops): "));
  SERIAL_OUT.print(fmodTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(1000.0 / fmodTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));
}

#if defined(ARDUINO_ARCH_RP2040)
void benchmarkSHA1() {
  printHeader("CRYPTO: SHA1");

  SERIAL_OUT.println(F_STR("Example digests:"));
  SERIAL_OUT.print(F_STR("SHA1:"));
  SERIAL_OUT.println(sha1("abc"));

  uint8_t hash[20];
  sha1("test", &hash[0]);
  SERIAL_OUT.print(F_STR("SHA1:"));
  for (uint16_t i = 0; i < 20; i++) {
    SERIAL_OUT.printf("%02x", hash[i]);
  }
  SERIAL_OUT.println();

  const uint32_t minDurationMs = max(5UL, (gMinBenchUs + 999UL) / 1000UL);
  volatile uint32_t checksum = 0;

  TimedLoopResult stringResult = runTimedLoop(minDurationMs, 10, [&]() {
    for (uint8_t i = 0; i < 10; i++) {
      String digest = sha1("abc");
      checksum += digest.length();
    }
  });

  TimedLoopResult bufferResult = runTimedLoop(minDurationMs, 10, [&]() {
    for (uint8_t i = 0; i < 10; i++) {
      sha1("test", &hash[0]);
      checksum += hash[0];
    }
  });

  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println(checksum);

  SERIAL_OUT.print(F_STR("SHA1 String ("));
  SERIAL_OUT.print(stringResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(stringResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(stringResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("SHA1 Buffer ("));
  SERIAL_OUT.print(bufferResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(bufferResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(bufferResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));
}
#endif

#if defined(ESP32)
static inline int benchmarkMd5(const unsigned char *input, size_t size, unsigned char *output) {
#if defined(MBEDTLS_VERSION_MAJOR) && MBEDTLS_VERSION_MAJOR >= 3
  return mbedtls_md5(input, size, output);
#else
  return mbedtls_md5_ret(input, size, output);
#endif
}

static inline int benchmarkSha1(const unsigned char *input, size_t size, unsigned char *output) {
#if defined(MBEDTLS_VERSION_MAJOR) && MBEDTLS_VERSION_MAJOR >= 3
  return mbedtls_sha1(input, size, output);
#else
  return mbedtls_sha1_ret(input, size, output);
#endif
}

static inline int benchmarkSha256(const unsigned char *input, size_t size, unsigned char *output) {
#if defined(MBEDTLS_VERSION_MAJOR) && MBEDTLS_VERSION_MAJOR >= 3
  return mbedtls_sha256(input, size, output, 0);
#else
  return mbedtls_sha256_ret(input, size, output, 0);
#endif
}

static inline int benchmarkSha512(const unsigned char *input, size_t size, unsigned char *output) {
#if defined(MBEDTLS_VERSION_MAJOR) && MBEDTLS_VERSION_MAJOR >= 3
  return mbedtls_sha512(input, size, output, 0);
#else
  return mbedtls_sha512_ret(input, size, output, 0);
#endif
}

void benchmarkESP32Crypto() {
  printHeader("CRYPTO: Hashing (ESP32)");

  constexpr uint8_t inputSize = 128;
  uint8_t input[inputSize];
  for (uint8_t i = 0; i < inputSize; i++) {
    input[i] = (uint8_t)(i * 3 + 7);
  }

  uint8_t digest[64];
  char hexDigest[129];
  char hexInput[(inputSize * 2) + 1];
  volatile uint32_t checksum = 0;
  uint32_t lastProgressMs = millis();
  const uint32_t maxCryptoMs = 15000;
  const uint32_t cryptoStartMs = lastProgressMs;
  bool cryptoTimedOut = false;
  bool cryptoAbortPrinted = false;
  auto benchYield = []() {
#if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_RP2040)
    yield();
#endif
  };
  auto checkCryptoTimeout = [&]() -> bool {
    if (!cryptoTimedOut && (millis() - cryptoStartMs > maxCryptoMs)) {
      SERIAL_OUT.println(F_STR("Warning: crypto benchmark timed out."));
      cryptoTimedOut = true;
    }
    return cryptoTimedOut;
  };
  auto abortCryptoIfTimedOut = [&]() -> bool {
    if (cryptoTimedOut && !cryptoAbortPrinted) {
      SERIAL_OUT.println(F_STR("Crypto benchmark aborted due to timeout."));
      cryptoAbortPrinted = true;
    }
    return cryptoTimedOut;
  };

  auto toHex = [&](const uint8_t *data, size_t len, char *out, size_t outCapacity) -> bool {
    const size_t requiredSize = (len * 2) + 1;
    if (outCapacity < requiredSize) {
      return false;
    }
#if defined(ESP32)
    size_t written = HEXBuilder::bytes2hex(out, outCapacity, data, len);
    return written >= requiredSize;
#else
    static const char kHexChars[] = "0123456789abcdef";
    for (size_t i = 0; i < len; ++i) {
      const uint8_t byteValue = data[i];
      out[i * 2] = kHexChars[(byteValue >> 4) & 0x0F];
      out[(i * 2) + 1] = kHexChars[byteValue & 0x0F];
    }
    out[len * 2] = '\0';
    return true;
#endif
  };

  const uint32_t minDurationMs = max(5UL, (gMinBenchUs + 999UL) / 1000UL);

  SERIAL_OUT.println(F_STR("Example digests:"));
  benchmarkMd5(input, inputSize, digest);
  if (!toHex(digest, 16, hexDigest, sizeof(hexDigest))) {
    SERIAL_OUT.println(F_STR("MD5 hex buffer too small."));
    return;
  }
  SERIAL_OUT.print(F_STR("MD5: "));
  SERIAL_OUT.println(hexDigest);

  benchmarkSha1(input, inputSize, digest);
  if (!toHex(digest, 20, hexDigest, sizeof(hexDigest))) {
    SERIAL_OUT.println(F_STR("SHA1 hex buffer too small."));
    return;
  }
  SERIAL_OUT.print(F_STR("SHA1: "));
  SERIAL_OUT.println(hexDigest);

  benchmarkSha256(input, inputSize, digest);
  if (!toHex(digest, 32, hexDigest, sizeof(hexDigest))) {
    SERIAL_OUT.println(F_STR("SHA256 hex buffer too small."));
    return;
  }
  SERIAL_OUT.print(F_STR("SHA256: "));
  SERIAL_OUT.println(hexDigest);

  benchmarkSha512(input, inputSize, digest);
  if (!toHex(digest, 64, hexDigest, sizeof(hexDigest))) {
    SERIAL_OUT.println(F_STR("SHA512 hex buffer too small."));
    return;
  }
  SERIAL_OUT.print(F_STR("SHA512: "));
  SERIAL_OUT.println(hexDigest);

#if defined(MBEDTLS_SHA3_C)
  {
    mbedtls_sha3_context sha3;
    mbedtls_sha3_init(&sha3);
    mbedtls_sha3_starts(&sha3, 256);
    mbedtls_sha3_update(&sha3, input, inputSize);
    mbedtls_sha3_finish(&sha3, digest);
    mbedtls_sha3_free(&sha3);
    if (!toHex(digest, 32, hexDigest, sizeof(hexDigest))) {
      SERIAL_OUT.println(F_STR("SHA3-256 hex buffer too small."));
      return;
    }
    SERIAL_OUT.print(F_STR("SHA3-256: "));
    SERIAL_OUT.println(hexDigest);
  }
#else
  SERIAL_OUT.println(F_STR("SHA3-256: not available in this build"));
#endif

  SERIAL_OUT.println(F_STR("...running HEX encode"));
  TimedLoopResult hexResult = runTimedLoop(minDurationMs, 20, [&]() -> bool {
    for (uint8_t i = 0; i < 20; i++) {
      if (checkCryptoTimeout()) {
        return false;
      }
      if (!toHex(input, inputSize, hexInput, sizeof(hexInput))) {
        SERIAL_OUT.println(F_STR("HEX encode buffer too small."));
        return false;
      }
      checksum += hexInput[0];
      if ((i % 5) == 0) {
        benchYield();
      }
    }
    return true;
  });
  if (abortCryptoIfTimedOut()) {
    return;
  }
  SERIAL_OUT.println(F_STR("...running MD5"));
  TimedLoopResult md5Result = runTimedLoop(minDurationMs, 10, [&]() -> bool {
    for (uint8_t i = 0; i < 10; i++) {
      if (checkCryptoTimeout()) {
        return false;
      }
      benchmarkMd5(input, inputSize, digest);
      checksum += digest[0];
      if ((i % 3) == 0) {
        benchYield();
      }
    }
    return true;
  });
  if (abortCryptoIfTimedOut()) {
    return;
  }
  SERIAL_OUT.println(F_STR("...running SHA1"));
  TimedLoopResult sha1Result = runTimedLoop(minDurationMs, 10, [&]() -> bool {
    for (uint8_t i = 0; i < 10; i++) {
      if (checkCryptoTimeout()) {
        return false;
      }
      benchmarkSha1(input, inputSize, digest);
      checksum += digest[0];
      if ((i % 3) == 0) {
        benchYield();
      }
    }
    return true;
  });
  if (abortCryptoIfTimedOut()) {
    return;
  }
  SERIAL_OUT.println(F_STR("...running SHA256"));
  TimedLoopResult sha256Result = runTimedLoop(minDurationMs, 10, [&]() -> bool {
    for (uint8_t i = 0; i < 10; i++) {
      if (checkCryptoTimeout()) {
        return false;
      }
      benchmarkSha256(input, inputSize, digest);
      checksum += digest[0];
      if ((i % 3) == 0) {
        benchYield();
      }
    }
    return true;
  });
  if (abortCryptoIfTimedOut()) {
    return;
  }
  SERIAL_OUT.println(F_STR("...running SHA512"));
  TimedLoopResult sha512Result = runTimedLoop(minDurationMs, 10, [&]() -> bool {
    for (uint8_t i = 0; i < 10; i++) {
      if (checkCryptoTimeout()) {
        return false;
      }
      benchmarkSha512(input, inputSize, digest);
      checksum += digest[0];
      if ((i % 2) == 0) {
        benchYield();
      }
    }
    return true;
  });
  if (abortCryptoIfTimedOut()) {
    return;
  }
#if defined(MBEDTLS_SHA3_C)
  SERIAL_OUT.println(F_STR("...running SHA3-256"));
  TimedLoopResult sha3Result = runTimedLoop(minDurationMs, 5, [&]() -> bool {
    for (uint8_t i = 0; i < 5; i++) {
      if (checkCryptoTimeout()) {
        return false;
      }
      mbedtls_sha3_context sha3;
      mbedtls_sha3_init(&sha3);
      mbedtls_sha3_starts(&sha3, 256);
      mbedtls_sha3_update(&sha3, input, inputSize);
      mbedtls_sha3_finish(&sha3, digest);
      mbedtls_sha3_free(&sha3);
      checksum += digest[0];
      benchYield();
    }
    return true;
  });
  if (abortCryptoIfTimedOut()) {
    return;
  }
#endif

  const uint8_t saltSize = 16;
  uint8_t salt[saltSize];
  for (uint8_t i = 0; i < saltSize; i++) {
    salt[i] = (uint8_t)(0xA5 ^ i);
  }
  const char *password = "esp32-benchmark";
  const uint32_t pbkdf2Iterations = 500;

  SERIAL_OUT.println(F_STR("...running PBKDF2-HMAC-SHA256"));
  TimedLoopResult pbkdf2Result = runTimedLoop(minDurationMs, 1, [&]() -> bool {
    if (checkCryptoTimeout()) {
      return false;
    }
#if defined(MBEDTLS_VERSION_NUMBER) && MBEDTLS_VERSION_NUMBER >= 0x03000000
    if (mbedtls_pkcs5_pbkdf2_hmac_ext(MBEDTLS_MD_SHA256,
                                      reinterpret_cast<const unsigned char *>(password),
                                      strlen(password),
                                      salt,
                                      saltSize,
                                      pbkdf2Iterations,
                                      32,
                                      digest)
        == 0) {
      checksum += digest[0];
    }
#else
      mbedtls_md_context_t ctx;
      mbedtls_md_init(&ctx);
      const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
      if (info != nullptr && mbedtls_md_setup(&ctx, info, 1) == 0) {
        if (mbedtls_pkcs5_pbkdf2_hmac(&ctx,
                                      reinterpret_cast<const unsigned char *>(password),
                                      strlen(password),
                                      salt,
                                      saltSize,
                                      pbkdf2Iterations,
                                      32,
                                      digest)
            == 0) {
          checksum += digest[0];
        }
      }
      mbedtls_md_free(&ctx);
#endif
    if (millis() - lastProgressMs > 1000) {
      SERIAL_OUT.print('.');
      lastProgressMs = millis();
    }
    benchYield();
    return !checkCryptoTimeout();
  });
  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println(checksum);

  SERIAL_OUT.print(F_STR("HEX encode ("));
  SERIAL_OUT.print(hexResult.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(hexResult.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(hexResult.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("MD5 ("));
  SERIAL_OUT.print(md5Result.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(md5Result.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(md5Result.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("SHA1 ("));
  SERIAL_OUT.print(sha1Result.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(sha1Result.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(sha1Result.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("SHA256 ("));
  SERIAL_OUT.print(sha256Result.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(sha256Result.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(sha256Result.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("SHA512 ("));
  SERIAL_OUT.print(sha512Result.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(sha512Result.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(sha512Result.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

#if defined(MBEDTLS_SHA3_C)
  SERIAL_OUT.print(F_STR("SHA3-256 ("));
  SERIAL_OUT.print(sha3Result.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(sha3Result.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(sha3Result.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));
#endif

  SERIAL_OUT.print(F_STR("PBKDF2-HMAC-SHA256 ("));
  SERIAL_OUT.print(pbkdf2Result.totalOps);
  SERIAL_OUT.print(F_STR(" ops): "));
  SERIAL_OUT.print(pbkdf2Result.elapsedMicros);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(pbkdf2Result.opsPerMs);
  SERIAL_OUT.println(F_STR(" ops/ms)"));
  SERIAL_OUT.print(F_STR("  Iterations per op: "));
  SERIAL_OUT.println(pbkdf2Iterations);
}
#endif

// ==================== TIMING PRECISION BENCHMARKS ====================

void benchmarkTimingPrecision() {
  printHeader("TIMING: millis() and micros() Precision");

  // Test millis() accuracy
  unsigned long startMillis = millis();
  delay(1000);
  unsigned long endMillis = millis();
  long millisError = abs((long)(endMillis - startMillis) - 1000);

  SERIAL_OUT.print(F_STR("millis() 1-second test: "));
  SERIAL_OUT.print(endMillis - startMillis);
  SERIAL_OUT.print(F_STR(" ms (±"));
  SERIAL_OUT.print(millisError);
  SERIAL_OUT.println(F_STR(" ms error)"));

  // Test micros() resolution - wait for value to change to measure granularity
  unsigned long micro1 = micros();
  unsigned long micro2 = micro1;
  uint16_t iterations = 0;
  const uint16_t maxIterations = 10000;

  // Wait for micros() to increment
  while (micro2 == micro1 && iterations < maxIterations) {
    micro2 = micros();
    iterations++;
  }

  unsigned long microResolution = micro2 - micro1;

  SERIAL_OUT.print(F_STR("micros() resolution: "));
  if (iterations >= maxIterations) {
    SERIAL_OUT.println(F_STR("TIMEOUT (timer may not be running)"));
  } else {
    SERIAL_OUT.print(microResolution);
    SERIAL_OUT.print(F_STR(" μs (detected after "));
    SERIAL_OUT.print(iterations);
    SERIAL_OUT.println(F_STR(" reads)"));
  }

  // Additional resolution test - measure minimum measurable difference
  unsigned long minDiff = 0xFFFFFFFF;
  for (int i = 0; i < 100; i++) {
    unsigned long t1 = micros();
    unsigned long t2 = micros();
    while (t2 == t1) {
      t2 = micros();
    }
    unsigned long diff = t2 - t1;
    if (diff > 0 && diff < minDiff) {
      minDiff = diff;
    }
  }

  SERIAL_OUT.print(F_STR("Minimum step size (100 samples): "));
  SERIAL_OUT.print(minDiff);
  SERIAL_OUT.println(F_STR(" μs"));

#if defined(__AVR__)
  SERIAL_OUT.println(F_STR("Note: AVR micros() typically advances in 4 μs steps"));
#endif

  // Test micros() consistency over short period
  unsigned long startMicros = micros();
  delayMicroseconds(1000);
  unsigned long endMicros = micros();
  long microsError = abs((long)(endMicros - startMicros) - 1000);

  SERIAL_OUT.print(F_STR("micros() 1ms test: "));
  SERIAL_OUT.print(endMicros - startMicros);
  SERIAL_OUT.print(F_STR(" μs (±"));
  SERIAL_OUT.print(microsError);
  SERIAL_OUT.println(F_STR(" μs error)"));

// ESP32: CPU cycle counter cross-check
#ifdef ESP32
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("ESP32 CPU Cycle Counter:"));

  // Get CPU frequency
  uint32_t cpuFreqMHz = ESP.getCpuFreqMHz();
  SERIAL_OUT.print(F_STR("CPU Frequency: "));
  SERIAL_OUT.print(cpuFreqMHz);
  SERIAL_OUT.println(F_STR(" MHz"));

  // Test cycle counter vs micros()
  uint32_t cycleStart = ESP.getCycleCount();
  unsigned long microsStart = micros();

  // Busy wait for ~1000 us
  delayMicroseconds(1000);

  uint32_t cycleEnd = ESP.getCycleCount();
  unsigned long microsEnd = micros();

  uint32_t cycleDelta = cycleEnd - cycleStart;
  unsigned long microsDelta = microsEnd - microsStart;

  SERIAL_OUT.print(F_STR("Cycles elapsed: "));
  SERIAL_OUT.println(cycleDelta);
  SERIAL_OUT.print(F_STR("micros() elapsed: "));
  SERIAL_OUT.print(microsDelta);
  SERIAL_OUT.println(F_STR(" μs"));

  // Calculate expected cycles
  float expectedCycles = microsDelta * cpuFreqMHz;
  SERIAL_OUT.print(F_STR("Expected cycles: "));
  SERIAL_OUT.println((uint32_t)expectedCycles);

  // Calculate jitter
  float jitterPercent = abs((float)cycleDelta - expectedCycles) / expectedCycles * 100.0f;
  SERIAL_OUT.print(F_STR("Timing jitter: "));
  SERIAL_OUT.print(jitterPercent, 2);
  SERIAL_OUT.println(F_STR("%"));
#endif

  // Clock frequency estimate
  SERIAL_OUT.println();
  SERIAL_OUT.print(F_STR("Clock accuracy: "));
  float accuracy = 100.0f - ((float)millisError / 10.0f);
  SERIAL_OUT.print(accuracy);
  SERIAL_OUT.println(F_STR("%"));
}

// ==================== STACK DEPTH BENCHMARK ====================

volatile int recursionCounter = 0;

int testRecursion(int depth) {
  recursionCounter++;
  volatile char buffer[32];  // Consume stack space
  buffer[0] = (char)depth;
  buffer[1] = (char)(depth >> 1);

  if (depth > 0) {
    return testRecursion(depth - 1) + (buffer[0] & 1);
  }
  return 1;
}

void benchmarkStackDepth() {
  printHeader("MEMORY: Stack Depth Test");

  // Scale test depth based on available RAM to avoid overflow
  int testDepth;

#if defined(BOARD_STM32U5)
  testDepth = 500;  // 786 KB RAM - can go deep
  SERIAL_OUT.println(F_STR("Testing deep recursion (786 KB SRAM)"));
#elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  testDepth = 50;  // C3/C6 have very limited task stack
  SERIAL_OUT.println(F_STR("Testing shallow recursion (ESP32-C3/C6)"));
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  testDepth = 80;  // S3 has more stack
  SERIAL_OUT.println(F_STR("Testing moderate recursion (ESP32-S3)"));
#elif defined(ESP32)
  testDepth = 60;  // Classic ESP32 - limited task stack
  SERIAL_OUT.println(F_STR("Testing moderate recursion (ESP32)"));
#elif defined(ARDUINO_ARCH_RP2040)
  testDepth = 300;  // 264 KB RAM
  SERIAL_OUT.println(F_STR("Testing deep recursion (264 KB RAM)"));
#elif defined(ARDUINO_SAM_DUE)
  testDepth = 200;       // 96 KB RAM
  SERIAL_OUT.println(F_STR("Testing moderate recursion (96 KB RAM)"));
#elif defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  testDepth = 100;        // 32 KB RAM - be conservative
  SERIAL_OUT.println(F_STR("Testing moderate recursion (32 KB RAM)"));
#elif defined(BOARD_SAMD) || defined(BOARD_NRF52)
  testDepth = 100;  // Typically 32-256 KB
  SERIAL_OUT.println(F_STR("Testing moderate recursion (ARM)"));
#elif defined(__AVR_ATmega2560__)
  testDepth = 40;  // 8 KB RAM - deeper than Uno
  SERIAL_OUT.println(F_STR("Testing shallow recursion (8 KB RAM)"));
#elif defined(__AVR__)
  testDepth = 20;  // 2-2.5 KB RAM - very conservative
  SERIAL_OUT.println(F_STR("Testing shallow recursion (2 KB RAM)"));
#else
  testDepth = 50;  // Conservative default
  SERIAL_OUT.println(F_STR("Testing moderate recursion (unknown RAM)"));
#endif

  // Test safe recursion depth
  recursionCounter = 0;
  int result = testRecursion(testDepth);
  (void)result;

  SERIAL_OUT.print(F_STR("Recursion test ("));
  SERIAL_OUT.print(testDepth);
  SERIAL_OUT.print(F_STR(" deep): "));
  if (recursionCounter == testDepth + 1) {
    SERIAL_OUT.println(F_STR("PASS"));
    SERIAL_OUT.print(F_STR("Successfully executed "));
    SERIAL_OUT.print(testDepth);
    SERIAL_OUT.println(F_STR(" nested function calls"));
  } else {
    SERIAL_OUT.print(F_STR("FAIL - reached depth "));
    SERIAL_OUT.println(recursionCounter);
  }

  // Estimate stack usage per call (very rough)
  // Each recursive call typically uses 20-50 bytes on ARM, more on some platforms
#if defined(BOARD_STM32U5) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ARCH_RP2040)
  SERIAL_OUT.print(F_STR("Estimated stack usage: ~"));
  SERIAL_OUT.print(testDepth * 40);  // Rough estimate
  SERIAL_OUT.println(F_STR(" bytes"));
#endif

  SERIAL_OUT.println(F_STR("Note: Actual stack usage varies by compiler and optimization."));
}

// ==================== MULTI-CORE BENCHMARKS ====================

#if defined(ESP32) || defined(ARDUINO_ARCH_RP2040)

#ifdef ESP32
TaskHandle_t Task1;
TaskHandle_t Task2;
volatile unsigned long core0Count = 0;
volatile unsigned long core1Count = 0;
volatile bool testRunning = false;

void core0Task(void *parameter) {
  volatile uint32_t accumulator = 0;
  while (true) {
    if (testRunning) {
      core0Count++;
      accumulator += 3;
      accumulator ^= (accumulator << 1);
    }
  }
}

void core1Task(void *parameter) {
  volatile uint32_t accumulator = 0;
  while (true) {
    if (testRunning) {
      core1Count++;
      accumulator += 3;
      accumulator ^= (accumulator << 1);
    }
  }
}
#endif

#if defined(ARDUINO_ARCH_RP2040) && defined(HAS_PICO_MULTICORE)
volatile unsigned long rp2040Core0Count = 0;
volatile unsigned long rp2040Core1Count = 0;
volatile bool rp2040TestRunning = false;
volatile bool rp2040Core1Ready = false;

void rp2040Core1Task() {
  volatile uint32_t accumulator = 0;
  rp2040Core1Ready = true;
  while (true) {
    if (rp2040TestRunning) {
      rp2040Core1Count++;
      accumulator += 3;
      accumulator ^= (accumulator << 1);
    }
  }
}
#endif

void benchmarkMultiCore() {
  printHeader("MULTI-CORE: Parallel Performance");

#ifdef ESP32
  // Check number of cores available
  uint8_t numCores = 1;
#if CONFIG_FREERTOS_UNICORE
  numCores = 1;
#else
// For multi-core ESP32 variants, we can check portNUM_PROCESSORS or assume 2
#if defined(portNUM_PROCESSORS)
  numCores = portNUM_PROCESSORS;
#elif !defined(CONFIG_IDF_TARGET_ESP32C6) && !defined(CONFIG_IDF_TARGET_ESP32C5) && !defined(CONFIG_IDF_TARGET_ESP32H2) && !defined(CONFIG_IDF_TARGET_ESP32C2) && !defined(CONFIG_IDF_TARGET_ESP32C3)
  numCores = 2;  // Classic ESP32, S2, S3
#endif
#endif

  if (numCores < 2) {
    SERIAL_OUT.println(F_STR("Single-Core ESP32 Variant"));
    SERIAL_OUT.println(F_STR("Running single-core workload for 1 second..."));

    volatile unsigned long count = 0;
    volatile uint32_t accumulator = 0;
    unsigned long start = millis();
    while (millis() - start < 1000) {
      count++;
      accumulator += 3;
      accumulator ^= (accumulator << 1);
    }

    SERIAL_OUT.print(F_STR("Iterations in 1 second: "));
    SERIAL_OUT.println(count);
    SERIAL_OUT.println(F_STR("Note: This variant has only one core"));
  } else {
    SERIAL_OUT.println(F_STR("ESP32 Dual-Core Test"));

    // Create tasks on both cores
    xTaskCreatePinnedToCore(
      core0Task,
      "Task0",
      10000,
      NULL,
      1,
      &Task1,
      0);

    xTaskCreatePinnedToCore(
      core1Task,
      "Task1",
      10000,
      NULL,
      1,
      &Task2,
      1);

    delay(100);  // Let tasks start

    // Run test
    core0Count = 0;
    core1Count = 0;
    testRunning = true;
    delay(1000);
    testRunning = false;

    SERIAL_OUT.print(F_STR("Core 0 iterations: "));
    SERIAL_OUT.println(core0Count);
    SERIAL_OUT.print(F_STR("Core 1 iterations: "));
    SERIAL_OUT.println(core1Count);
    SERIAL_OUT.print(F_STR("Total iterations: "));
    SERIAL_OUT.println(core0Count + core1Count);
    SERIAL_OUT.print(F_STR("Core balance: "));
    float balance = (float)min(core0Count, core1Count) / (float)max(core0Count, core1Count) * 100.0f;
    SERIAL_OUT.print(balance);
    SERIAL_OUT.println(F_STR("%"));

    // Clean up
    vTaskDelete(Task1);
    vTaskDelete(Task2);
  }  // End of multi-core else block

#elif defined(ARDUINO_ARCH_RP2040)
  SERIAL_OUT.println(F_STR("RP2040 Dual-Core Test"));
#if defined(HAS_PICO_MULTICORE)
  SERIAL_OUT.println(F_STR("Running dual-core workload for 1 second..."));

  rp2040Core0Count = 0;
  rp2040Core1Count = 0;
  rp2040TestRunning = false;
  rp2040Core1Ready = false;

  multicore_reset_core1();
  multicore_launch_core1(rp2040Core1Task);

  unsigned long readyStart = millis();
  while (!rp2040Core1Ready && millis() - readyStart < 200) {
    delay(1);
  }

  rp2040TestRunning = true;
  unsigned long start = millis();
  volatile uint32_t accumulator = 0;
  while (millis() - start < 1000) {
    rp2040Core0Count++;
    accumulator += 3;
    accumulator ^= (accumulator << 1);
  }
  rp2040TestRunning = false;

  unsigned long total = rp2040Core0Count + rp2040Core1Count;
  unsigned long dominant = max(rp2040Core0Count, rp2040Core1Count);
  float scaling = dominant > 0 ? (float)total / (float)dominant : 0.0f;

  SERIAL_OUT.print(F_STR("Core 0 iterations: "));
  SERIAL_OUT.println(rp2040Core0Count);
  SERIAL_OUT.print(F_STR("Core 1 iterations: "));
  SERIAL_OUT.println(rp2040Core1Count);
  SERIAL_OUT.print(F_STR("Total iterations: "));
  SERIAL_OUT.println(total);
  SERIAL_OUT.print(F_STR("Scaling efficiency: "));
  SERIAL_OUT.print(scaling, 2);
  SERIAL_OUT.println(F_STR("x"));
#else
  SERIAL_OUT.println(F_STR("Single core performance:"));

  volatile unsigned long count = 0;
  unsigned long start = millis();
  while (millis() - start < 1000) {
    count++;
  }

  SERIAL_OUT.print(F_STR("Iterations in 1 second: "));
  SERIAL_OUT.println(count);
  SERIAL_OUT.println(F_STR("Note: Full dual-core requires multicore library"));
#endif
#endif
}
#endif

// ==================== SERIAL BAUD RATE BENCHMARK ====================
// NOTE: Baud rate sweeping causes boards to disconnect and require replug
// Disabled for stability

/*
void benchmarkSerialBaudRates() {
  printHeader("SERIAL: Baud Rate Sweep");
  
  const long baudRates[] = {9600, 57600, 115200, 230400, 460800, 921600};
  const int numRates = 6;
  
  SERIAL_OUT.println(F_STR("Testing throughput at different baud rates..."));
  delay(100);
  
  for (int i = 0; i < numRates; i++) {
    SERIAL_OUT.end();
    delay(100);
    SERIAL_OUT.begin(baudRates[i]);
    delay(100);
    
    // Test throughput at this baud rate
    const char testData[] = "0123456789";
    startBenchmark();
    for (int j = 0; j < 19; j++) {
      SERIAL_OUT.print(testData);
    }
    SERIAL_OUT.flush();
    unsigned long baudTime = endBenchmark();
    
    SERIAL_OUT.print(F_STR("Baud "));
    SERIAL_OUT.print(baudRates[i]);
    SERIAL_OUT.print(F_STR(": "));
    SERIAL_OUT.print(baudTime);
    SERIAL_OUT.print(F_STR(" μs ("));
    float bytesPerSec = 190000000.0f / baudTime;
    SERIAL_OUT.print(bytesPerSec, 0);
    SERIAL_OUT.println(F_STR(" bytes/sec)"));
    
    delay(50);
  }
  
  // Return to standard baud
  SERIAL_OUT.end();
  delay(100);
  SERIAL_OUT.begin(115200);
  delay(500);
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("Returned to 115200 baud"));
}
*/

// ==================== HARDWARE RNG BENCHMARK ====================

// Include Zephyr RNG for STM32U5 boards
#if defined(BOARD_STM32U5) && defined(__ZEPHYR__)
#include <zephyr/random/random.h>
#endif

#if defined(ESP32) || defined(BOARD_STM32U5)
void benchmarkHardwareRNG() {
  printHeader("CRYPTO: Hardware RNG");

#if defined(ESP32)
  SERIAL_OUT.println(F_STR("Using ESP32 hardware RNG"));
#elif defined(BOARD_STM32U5)
#if defined(__ZEPHYR__)
  SERIAL_OUT.println(F_STR("Using STM32U585 hardware RNG (Zephyr)"));
#else
  SERIAL_OUT.println(F_STR("Using STM32U585 RNG"));
#endif
#endif

  // Test RNG speed
  startBenchmark();
  volatile uint32_t rngSum = 0;
  for (int i = 0; i < 1000; i++) {
#if defined(ESP32)
    rngSum += esp_random();
#elif defined(BOARD_STM32U5) && defined(__ZEPHYR__)
    rngSum += sys_rand32_get();  // True hardware RNG via Zephyr
#elif defined(BOARD_STM32U5)
    rngSum += random(0, 0xFFFFFFFF);  // Fallback to Arduino random
#endif
  }
  unsigned long rngTime = endBenchmark();

  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println(rngSum);

  SERIAL_OUT.print(F_STR("Hardware RNG (1000 values): "));
  SERIAL_OUT.print(rngTime);
  SERIAL_OUT.print(F_STR(" μs ("));
  SERIAL_OUT.print(1000.0f / rngTime * 1000, 2);
  SERIAL_OUT.println(F_STR(" values/ms)"));

  // Test randomness distribution
  uint32_t bins[4] = { 0, 0, 0, 0 };
  for (int i = 0; i < 10000; i++) {
#if defined(ESP32)
    uint32_t val = esp_random();
#elif defined(BOARD_STM32U5) && defined(__ZEPHYR__)
    uint32_t val = sys_rand32_get();
#elif defined(BOARD_STM32U5)
    uint32_t val = random(0, 0xFFFFFFFF);
#endif
    bins[val % 4]++;
  }

  SERIAL_OUT.println(F_STR("Distribution (10000 samples):"));
  for (int i = 0; i < 4; i++) {
    SERIAL_OUT.print(F_STR("  Bin "));
    SERIAL_OUT.print(i);
    SERIAL_OUT.print(F_STR(": "));
    SERIAL_OUT.print(bins[i]);
    SERIAL_OUT.print(F_STR(" ("));
    SERIAL_OUT.print(bins[i] / 100.0f, 1);
    SERIAL_OUT.println(F_STR("%)"));
  }
  SERIAL_OUT.println(F_STR("(Ideal: 25% each bin)"));
}
#endif

// ==================== SOFTWARE ATSE BENCHMARK ====================

// Arduino Uno R4 WiFi has an ESP32-S3 WiFi module with secure element capabilities
// SoftwareATSE library is included with the WiFiS3 library on Uno R4 WiFi
#if defined(ARDUINO_UNOR4_WIFI)

#include <SoftwareATSE.h>

constexpr int kAtseKeyId = 999;  // Reserved for benchmark use to avoid clobbering other keys.

void benchmarkSoftwareATSE() {
  printHeader("CRYPTO: SoftwareATSE (Uno R4 WiFi)");

  SERIAL_OUT.println(F_STR("Initializing SoftwareATSE..."));

  // Initialize SoftwareATSE
  if (!SATSE.begin()) {
    SERIAL_OUT.println(F_STR("SoftwareATSE initialization failed"));
    SERIAL_OUT.println(F_STR("The WiFi module may not be responding or"));
    SERIAL_OUT.println(F_STR("secure element features may not be available."));
    return;
  }

  SERIAL_OUT.println(F_STR("SoftwareATSE initialized successfully!"));
  SERIAL_OUT.println();

  // ===== Public Key Generation Benchmark =====
  SERIAL_OUT.println(F_STR("--- Public Key Generation ---"));

  bool keyGenSupported = false;
  const uint16_t keySlot = kAtseKeyId;
  unsigned long keyGenStart = millis();
  byte publicKey[64];   // Buffer for public key
  byte privateKey[32];  // Buffer for private key

  if (SATSE.generatePrivateKey(keySlot, privateKey) == 1) {
    // Try to generate public key in the same slot
    if (SATSE.generatePublicKey(kAtseKeyId, publicKey) == 1) {
      unsigned long keyGenTime = millis() - keyGenStart;
      keyGenSupported = true;

      SERIAL_OUT.print(F_STR("SoftwareATSE: Public key generation (ms): "));
      SERIAL_OUT.println(keyGenTime);

      SERIAL_OUT.print(F_STR("  Rate: "));
      if (keyGenTime > 0) {
        SERIAL_OUT.print(1000.0f / keyGenTime, 3);
        SERIAL_OUT.println(F_STR(" keys/sec"));
      } else {
        SERIAL_OUT.println(F_STR("N/A (too fast)"));
      }
    } else {
      SERIAL_OUT.println(F_STR("SoftwareATSE: Public key generation - not supported"));
    }
  } else {
    SERIAL_OUT.println(F_STR("SoftwareATSE: Private key generation failed; skipping public key test"));
  }

  SERIAL_OUT.println();

  // ===== Configuration Write Benchmark =====
  SERIAL_OUT.println(F_STR("--- Secure Storage Operations ---"));

  uint8_t testConfig[256];
  for (int i = 0; i < 256; i++) {
    testConfig[i] = i & 0xFF;
  }

  unsigned long writeStart = micros();
  bool writeSuccess = false;

  if (SATSE.writeConfiguration(testConfig) == 1) {
    unsigned long writeTime = micros() - writeStart;
    writeSuccess = true;

    SERIAL_OUT.print(F_STR("Config write (\xC2\xB5s): "));
    SERIAL_OUT.println(writeTime);
    SERIAL_OUT.print(F_STR("  (ms): "));
    SERIAL_OUT.println(writeTime / 1000.0f, 3);

    SERIAL_OUT.print(F_STR("  Write speed: "));
    if (writeTime > 0) {
      SERIAL_OUT.print(256.0f * 1000000.0f / writeTime, 2);
      SERIAL_OUT.println(F_STR(" bytes/sec"));
    } else {
      SERIAL_OUT.println(F_STR("N/A (too fast)"));
    }
  } else {
    SERIAL_OUT.println(F_STR("Config write - not supported"));
  }

  SERIAL_OUT.println();

  // ===== Signing Benchmark =====
  SERIAL_OUT.println(F_STR("--- EC Signing ---"));

  if (keyGenSupported) {
    uint8_t testData[32];
    for (int i = 0; i < 32; i++) {
      testData[i] = (uint8_t)(i * 7 + 13);
    }

    uint8_t signature[64];

    // Signing benchmark
    const int NUM_SIGN_OPS = 10;
    unsigned long signStart = micros();
    int successfulSigns = 0;

    for (int i = 0; i < NUM_SIGN_OPS; i++) {
      if (SATSE.ecSign(kAtseKeyId, testData, signature)) {
        successfulSigns++;
      }
      yield();
    }

    unsigned long signTime = micros() - signStart;

    if (successfulSigns > 0) {
      float avgSignTimeMs = (signTime / 1000.0f) / successfulSigns;
      SERIAL_OUT.print(F_STR("EC signing (avg ms/op): "));
      SERIAL_OUT.println(avgSignTimeMs, 3);

      SERIAL_OUT.print(F_STR("  Rate: "));
      SERIAL_OUT.print(1000.0f / avgSignTimeMs, 2);
      SERIAL_OUT.println(F_STR(" ops/sec"));

      SERIAL_OUT.print(F_STR("  Successful operations: "));
      SERIAL_OUT.print(successfulSigns);
      SERIAL_OUT.print(F_STR("/"));
      SERIAL_OUT.println(NUM_SIGN_OPS);
    } else {
      SERIAL_OUT.println(F_STR("EC signing - not supported"));
    }
  } else {
    SERIAL_OUT.println(F_STR("Signing tests skipped (key generation not supported)"));
  }

  SERIAL_OUT.println();

  // ===== Secure RNG Benchmark =====
  SERIAL_OUT.println(F_STR("--- Secure Random Number Generator ---"));

  uint8_t rngTest[16];
  if (SATSE.random(rngTest, 16)) {

    // Benchmark RNG throughput
    const int RNG_BUFFER_SIZE = 256;
    const int RNG_ITERATIONS = 20;
    uint8_t rngBuffer[RNG_BUFFER_SIZE];

    unsigned long rngStart = micros();
    int successfulReads = 0;
    uint32_t rngChecksum = 0;

    for (int i = 0; i < RNG_ITERATIONS; i++) {
      if (SATSE.random(rngBuffer, RNG_BUFFER_SIZE)) {
        successfulReads++;
        for (int j = 0; j < RNG_BUFFER_SIZE; j++) {
          rngChecksum += rngBuffer[j];
        }
      }
      yield();
    }

    unsigned long rngTime = micros() - rngStart;

    SERIAL_OUT.print(F_STR("RNG checksum: "));
    SERIAL_OUT.println(rngChecksum);

    if (successfulReads > 0) {
      unsigned long totalBytes = successfulReads * RNG_BUFFER_SIZE;
      float bytesPerSec = (totalBytes * 1000000.0f) / rngTime;

      SERIAL_OUT.print(F_STR("Secure RNG throughput: "));
      SERIAL_OUT.print(bytesPerSec, 2);
      SERIAL_OUT.println(F_STR(" bytes/sec"));

      SERIAL_OUT.print(F_STR("  Total bytes generated: "));
      SERIAL_OUT.println(totalBytes);

      SERIAL_OUT.print(F_STR("  Time: "));
      SERIAL_OUT.print(rngTime / 1000.0f, 2);
      SERIAL_OUT.println(F_STR(" ms"));

      SERIAL_OUT.print(F_STR("  Successful reads: "));
      SERIAL_OUT.print(successfulReads);
      SERIAL_OUT.print(F_STR("/"));
      SERIAL_OUT.println(RNG_ITERATIONS);

      // Distribution test
      SERIAL_OUT.println(F_STR("Distribution test (first 1000 bytes):"));
      uint32_t byteBins[4] = { 0, 0, 0, 0 };

      for (int i = 0; i < 1000; i++) {
        uint8_t randomByte;
        if (SATSE.random(&randomByte, 1)) {
          byteBins[randomByte % 4]++;
        }
      }

      for (int i = 0; i < 4; i++) {
        SERIAL_OUT.print(F_STR("  Bin "));
        SERIAL_OUT.print(i);
        SERIAL_OUT.print(F_STR(": "));
        SERIAL_OUT.print(byteBins[i]);
        SERIAL_OUT.print(F_STR(" ("));
        SERIAL_OUT.print(byteBins[i] / 10.0f, 1);
        SERIAL_OUT.println(F_STR("%)"));
      }
      SERIAL_OUT.println(F_STR("  (Ideal: 25% each bin)"));
    } else {
      SERIAL_OUT.println(F_STR("Secure RNG - read operations failed"));
    }
  } else {
    SERIAL_OUT.println(F_STR("Secure RNG - not supported"));
  }

  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("SoftwareATSE benchmarks complete"));
}

#else  // !ARDUINO_UNOR4_WIFI

// Stub function for non-Uno R4 WiFi boards
void benchmarkSoftwareATSE() {
  // This function intentionally left empty for non-R4 boards
  // It won't be called due to compile-time guards in setup()
}

#endif  // ARDUINO_UNOR4_WIFI

// ==================== ARDUINO BRIDGE (UNO Q MCU<->LINUX) ====================

#if defined(BOARD_STM32U5) && defined(HAS_DUAL_PROCESSOR)

void benchmarkArduinoBridge() {
  printHeader("ARDUINO BRIDGE (MCU<->LINUX RPC)");

#ifdef HAS_RPC_BRIDGE
  SERIAL_OUT.println(F_STR("Testing MCU<->Linux RPC communication..."));
  SERIAL_OUT.println(F_STR("Using MessagePack RPC over Serial1"));
  SERIAL_OUT.println();

  // Note: For RouterBridge, Bridge object is pre-initialized
  // For RPClite, we need to create transport and client

#ifdef HAS_ROUTER_BRIDGE
  // Using Arduino_RouterBridge high-level API
  SERIAL_OUT.println(F_STR("Bridge Type: RouterBridge (high-level API)"));
  SERIAL_OUT.println();

  // Test 1: Simple RPC call latency
  SERIAL_OUT.println(F_STR("1. RPC Call Latency Test"));
  const int PING_COUNT = 50;
  unsigned long totalLatency = 0;
  int successfulCalls = 0;

  for (int i = 0; i < PING_COUNT; i++) {
    unsigned long start = micros();

    // Call a simple echo/ping function on Linux side
    // Assumes Linux has registered an "echo" or "ping" method
    String result;
    bool ok = Bridge.call("echo", result, "ping").result(result);

    unsigned long elapsed = micros() - start;

    if (ok) {
      totalLatency += elapsed;
      successfulCalls++;
    }

    if (i % 10 == 0) {
      yield();
    }
  }

  if (successfulCalls > 0) {
    SERIAL_OUT.print(F_STR("  Successful calls: "));
    SERIAL_OUT.print(successfulCalls);
    SERIAL_OUT.print(F_STR("/"));
    SERIAL_OUT.println(PING_COUNT);
    SERIAL_OUT.print(F_STR("  Average latency: "));
    SERIAL_OUT.print(totalLatency / successfulCalls);
    SERIAL_OUT.println(F_STR(" µs"));
    SERIAL_OUT.print(F_STR("  Calls per second: "));
    SERIAL_OUT.println((successfulCalls * 1000000.0) / totalLatency, 0);
  } else {
    SERIAL_OUT.println(F_STR("  ERROR: No successful RPC calls"));
    SERIAL_OUT.println(F_STR("  Make sure Linux side has 'echo' method registered"));
  }
  SERIAL_OUT.println();

  // Test 2: Integer arithmetic RPC
  SERIAL_OUT.println(F_STR("2. Integer Arithmetic RPC Test"));
  int mathTests = 0;
  int mathSuccess = 0;
  unsigned long mathTime = micros();

  for (int i = 0; i < 20; i++) {
    int sum;
    if (Bridge.call("add", sum, i, i + 1).result(sum)) {
      if (sum == (i + i + 1)) {
        mathSuccess++;
      }
      mathTests++;
    }
  }

  mathTime = micros() - mathTime;
  SERIAL_OUT.print(F_STR("  Tests: "));
  SERIAL_OUT.print(mathSuccess);
  SERIAL_OUT.print(F_STR("/"));
  SERIAL_OUT.println(mathTests);
  SERIAL_OUT.print(F_STR("  Average time: "));
  SERIAL_OUT.print(mathTime / mathTests);
  SERIAL_OUT.println(F_STR(" µs"));
  SERIAL_OUT.println();

  // Test 3: String operations
  SERIAL_OUT.println(F_STR("3. String RPC Test"));
  int strTests = 0;
  int strSuccess = 0;
  unsigned long strTime = micros();

  for (int i = 0; i < 10; i++) {
    String message = "Test_" + String(i);
    String response;
    if (Bridge.call("loopback", response, message).result(response)) {
      if (response == message) {
        strSuccess++;
      }
      strTests++;
    }
  }

  strTime = micros() - strTime;
  SERIAL_OUT.print(F_STR("  Tests: "));
  SERIAL_OUT.print(strSuccess);
  SERIAL_OUT.print(F_STR("/"));
  SERIAL_OUT.println(strTests);
  SERIAL_OUT.print(F_STR("  Average time: "));
  SERIAL_OUT.print(strTime / strTests);
  SERIAL_OUT.println(F_STR(" µs"));
  SERIAL_OUT.println();

  // Test 4: Async call test
  SERIAL_OUT.println(F_STR("4. Async RPC Call Test"));
  unsigned long asyncStart = micros();

  // Make multiple async calls
  RpcCall call1 = Bridge.call("add", 10, 20);
  RpcCall call2 = Bridge.call("add", 30, 40);
  RpcCall call3 = Bridge.call("add", 50, 60);

  // Now wait for results
  int result1, result2, result3;
  bool ok1 = call1.result(result1);
  bool ok2 = call2.result(result2);
  bool ok3 = call3.result(result3);

  unsigned long asyncTime = micros() - asyncStart;

  SERIAL_OUT.print(F_STR("  3 async calls completed in: "));
  SERIAL_OUT.print(asyncTime);
  SERIAL_OUT.println(F_STR(" µs"));
  if (ok1 && ok2 && ok3) {
    SERIAL_OUT.print(F_STR("  Results: "));
    SERIAL_OUT.print(result1);
    SERIAL_OUT.print(F_STR(", "));
    SERIAL_OUT.print(result2);
    SERIAL_OUT.print(F_STR(", "));
    SERIAL_OUT.println(result3);
  }
  SERIAL_OUT.println();

  SERIAL_OUT.println(F_STR("NOTE: These tests require corresponding RPC methods"));
  SERIAL_OUT.println(F_STR("      registered on the Linux side (echo, add, loopback)."));
  SERIAL_OUT.println(F_STR("      See Arduino_RouterBridge examples for Linux setup."));

#elif defined(HAS_RPCLITE)
  // Using Arduino_RPClite low-level API
  SERIAL_OUT.println(F_STR("Bridge Type: RPClite (low-level API)"));
  SERIAL_OUT.println();

  SerialTransport transport(Serial1);
  RPCClient client(transport);

  SERIAL_OUT.println(F_STR("1. RPC Call Latency Test"));
  const int PING_COUNT = 50;
  unsigned long totalLatency = 0;
  int successfulCalls = 0;

  for (int i = 0; i < PING_COUNT; i++) {
    unsigned long start = micros();

    String result;
    bool ok = client.call("echo", result, "ping");

    unsigned long elapsed = micros() - start;

    if (ok) {
      totalLatency += elapsed;
      successfulCalls++;
    }

    if (i % 10 == 0) {
      yield();
    }
  }

  if (successfulCalls > 0) {
    SERIAL_OUT.print(F_STR("  Successful calls: "));
    SERIAL_OUT.print(successfulCalls);
    SERIAL_OUT.print(F_STR("/"));
    SERIAL_OUT.println(PING_COUNT);
    SERIAL_OUT.print(F_STR("  Average latency: "));
    SERIAL_OUT.print(totalLatency / successfulCalls);
    SERIAL_OUT.println(F_STR(" µs"));
  }
  SERIAL_OUT.println();

  SERIAL_OUT.println(F_STR("NOTE: RPClite requires manual transport setup."));
  SERIAL_OUT.println(F_STR("      Ensure Serial1 is properly initialized."));
#endif

#else
  SERIAL_OUT.println(F_STR("Arduino RPC Bridge library not detected."));
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("To enable Bridge/RPC benchmarks:"));
  SERIAL_OUT.println(F_STR("  1. Install Arduino_RouterBridge or Arduino_RPClite"));
  SERIAL_OUT.println(F_STR("  2. Ensure Linux side has RPC server running"));
  SERIAL_OUT.println(F_STR("  3. Register test methods (echo, add, loopback)"));
  SERIAL_OUT.println(F_STR("  4. Re-compile and upload sketch"));
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("Library: github.com/arduino-libraries/Arduino_RouterBridge"));
  SERIAL_OUT.println(F_STR("Library: github.com/arduino-libraries/Arduino_RPClite"));
#endif

  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("Arduino Bridge benchmarks complete"));
}

#else  // !BOARD_STM32U5 || !HAS_DUAL_PROCESSOR

// Stub function for non-Uno Q boards
void benchmarkArduinoBridge() {
  // This function intentionally left empty for non-Uno Q boards
  // It won't be called due to compile-time guards in setup()
}

#endif  // BOARD_STM32U5 && HAS_DUAL_PROCESSOR

// ==================== MULTIDUINO-SPECIFIC BENCHMARKS ====================

#ifdef HAS_RTC
void benchmarkRTC() {
  printHeader("RTC BENCHMARK (DS1307)");

  // Initialize RTC
  if (!rtc.begin()) {
    SERIAL_OUT.println(F_STR("ERROR: RTC not found!"));
    SERIAL_OUT.println(F_STR("Check wiring: SDA->A4, SCL->A5"));
    SERIAL_OUT.println();
    return;
  }

  SERIAL_OUT.println(F_STR("RTC initialized successfully"));

  // Check if RTC is running
  if (!rtc.isrunning()) {
    SERIAL_OUT.println(F_STR("WARNING: RTC is not running!"));
    SERIAL_OUT.println(F_STR("Starting RTC and setting time..."));
    // Set to compile time as default
    rtc.adjust(DateTime(F_STR(__DATE__), F_STR(__TIME__)));
  }

  // Display current time
  DateTime now = rtc.now();
  SERIAL_OUT.print(F_STR("Current Time: "));
  SERIAL_OUT.print(now.year(), DEC);
  SERIAL_OUT.print('/');
  if (now.month() < 10) SERIAL_OUT.print('0');
  SERIAL_OUT.print(now.month(), DEC);
  SERIAL_OUT.print('/');
  if (now.day() < 10) SERIAL_OUT.print('0');
  SERIAL_OUT.print(now.day(), DEC);
  SERIAL_OUT.print(" ");
  if (now.hour() < 10) SERIAL_OUT.print('0');
  SERIAL_OUT.print(now.hour(), DEC);
  SERIAL_OUT.print(':');
  if (now.minute() < 10) SERIAL_OUT.print('0');
  SERIAL_OUT.print(now.minute(), DEC);
  SERIAL_OUT.print(':');
  if (now.second() < 10) SERIAL_OUT.print('0');
  SERIAL_OUT.println(now.second(), DEC);
  SERIAL_OUT.println();

  // Benchmark 1: RTC Read Speed
  SERIAL_OUT.println(F_STR("Test: RTC Read Speed"));
  volatile uint32_t readChecksum = 0;
  unsigned long startTime = micros();
  uint32_t reads = 0;

  unsigned long testDuration = 1000000UL;  // 1 second
  while (micros() - startTime < testDuration) {
    DateTime reading = rtc.now();
    readChecksum += reading.unixtime();
    reads++;
  }
  unsigned long elapsed = micros() - startTime;

  float readsPerMs = (reads * 1000.0f) / elapsed;
  SERIAL_OUT.print(F_STR("  Reads: "));
  SERIAL_OUT.print(reads);
  SERIAL_OUT.print(F_STR(" in "));
  SERIAL_OUT.print(elapsed / 1000.0f, 2);
  SERIAL_OUT.println(F_STR(" ms"));
  SERIAL_OUT.print(F_STR("  Speed: "));
  SERIAL_OUT.print(readsPerMs, 2);
  SERIAL_OUT.println(F_STR(" reads/ms"));
  SERIAL_OUT.print(F_STR("  Time per read: "));
  SERIAL_OUT.print((float)elapsed / reads, 2);
  SERIAL_OUT.println(F_STR(" μs"));
  SERIAL_OUT.print(F_STR("  Checksum: 0x"));
  SERIAL_OUT.println((unsigned long)readChecksum, HEX);
  SERIAL_OUT.println();

  // Benchmark 2: RTC Write Speed (Using adjust - time setting)
  SERIAL_OUT.println(F_STR("Test: RTC Write Speed (Time Adjust)"));
  startTime = micros();
  uint32_t writes = 0;

  testDuration = 1000000UL;  // 1 second
  DateTime testTime = DateTime(2025, 1, 1, 12, 0, 0);
  while (micros() - startTime < testDuration) {
    rtc.adjust(testTime);
    writes++;
  }
  elapsed = micros() - startTime;

  // Restore current time
  rtc.adjust(now);

  float writesPerMs = (writes * 1000.0f) / elapsed;
  SERIAL_OUT.print(F_STR("  Writes: "));
  SERIAL_OUT.print(writes);
  SERIAL_OUT.print(F_STR(" in "));
  SERIAL_OUT.print(elapsed / 1000.0f, 2);
  SERIAL_OUT.println(F_STR(" ms"));
  SERIAL_OUT.print(F_STR("  Speed: "));
  SERIAL_OUT.print(writesPerMs, 2);
  SERIAL_OUT.println(F_STR(" writes/ms"));
  SERIAL_OUT.print(F_STR("  Time per write: "));
  SERIAL_OUT.print((float)elapsed / writes, 2);
  SERIAL_OUT.println(F_STR(" μs"));
  SERIAL_OUT.println();

  // Benchmark 3: NVRAM Read/Write Speed (DS1307 has 56 bytes of NVRAM)
  SERIAL_OUT.println(F_STR("Test: NVRAM Read/Write (56 bytes)"));
  const uint8_t nvramSize = 56;
  uint8_t nvramData[nvramSize];

  // Write test
  for (uint8_t i = 0; i < nvramSize; i++) {
    nvramData[i] = i;
  }

  startTime = micros();
  uint32_t nvramWrites = 0;
  testDuration = 1000000UL;  // 1 second

  while (micros() - startTime < testDuration) {
    for (uint8_t addr = 0; addr < nvramSize; addr++) {
      rtc.writenvram(addr, nvramData[addr]);
    }
    nvramWrites++;
  }
  elapsed = micros() - startTime;

  uint32_t totalBytesWritten = nvramWrites * nvramSize;
  float nvramWriteSpeed = (totalBytesWritten * 1000.0f) / elapsed;

  SERIAL_OUT.print(F_STR("  NVRAM Writes: "));
  SERIAL_OUT.print(totalBytesWritten);
  SERIAL_OUT.print(F_STR(" bytes in "));
  SERIAL_OUT.print(elapsed / 1000.0f, 2);
  SERIAL_OUT.println(F_STR(" ms"));
  SERIAL_OUT.print(F_STR("  Speed: "));
  SERIAL_OUT.print(nvramWriteSpeed, 2);
  SERIAL_OUT.println(F_STR(" bytes/ms"));

  // Read test
  volatile uint32_t nvramChecksum = 0;
  startTime = micros();
  uint32_t nvramReads = 0;

  while (micros() - startTime < testDuration) {
    for (uint8_t addr = 0; addr < nvramSize; addr++) {
      nvramChecksum += rtc.readnvram(addr);
    }
    nvramReads++;
  }
  elapsed = micros() - startTime;

  uint32_t totalBytesRead = nvramReads * nvramSize;
  float nvramReadSpeed = (totalBytesRead * 1000.0f) / elapsed;

  SERIAL_OUT.print(F_STR("  NVRAM Reads: "));
  SERIAL_OUT.print(totalBytesRead);
  SERIAL_OUT.print(F_STR(" bytes in "));
  SERIAL_OUT.print(elapsed / 1000.0f, 2);
  SERIAL_OUT.println(F_STR(" ms"));
  SERIAL_OUT.print(F_STR("  Speed: "));
  SERIAL_OUT.print(nvramReadSpeed, 2);
  SERIAL_OUT.println(F_STR(" bytes/ms"));
  SERIAL_OUT.print(F_STR("  Checksum: 0x"));
  SERIAL_OUT.println((unsigned long)nvramChecksum, HEX);
  SERIAL_OUT.println();

  SERIAL_OUT.println(F_STR("RTC benchmarks complete"));
}
#endif  // HAS_RTC

// ==================== ESP32 ADDITIONAL BENCHMARKS ====================

#if defined(ESP32)

// ESP32 DAC Benchmark (GPIO25/26 on classic ESP32)
#if defined(CONFIG_IDF_TARGET_ESP32)
void benchmarkESP32DAC() {
  printHeader("I/O: DAC OUTPUT (ESP32)");

  SERIAL_OUT.println(F_STR("ESP32 has 2x 8-bit DACs on GPIO25 and GPIO26"));

  const int dacPin = 25;  // DAC1

  // DAC write speed benchmark
  volatile uint32_t dacValue = 0;
  startBenchmark();
  for (int i = 0; i < 10000; i++) {
    dacWrite(dacPin, dacValue & 0xFF);
    dacValue++;
  }
  unsigned long dacTime = endBenchmark();

  SERIAL_OUT.print(F_STR("dacWrite() (10000 ops): "));
  SERIAL_OUT.print(dacTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(10000.0 / dacTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  // DAC ramp test - measure update rate
  startBenchmark();
  for (int cycle = 0; cycle < 100; cycle++) {
    for (int val = 0; val < 256; val++) {
      dacWrite(dacPin, val);
    }
  }
  unsigned long rampTime = endBenchmark();

  SERIAL_OUT.print(F_STR("DAC ramp (100 cycles, 25600 writes): "));
  SERIAL_OUT.print(rampTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(25600.0 / rampTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  // Calculate effective sample rate
  float sampleRate = 25600.0 / (rampTime / 1000000.0);
  SERIAL_OUT.print(F_STR("Effective sample rate: "));
  SERIAL_OUT.print(sampleRate / 1000, 1);
  SERIAL_OUT.println(F_STR(" kHz"));

  dacWrite(dacPin, 0);  // Reset to 0
}
#endif  // CONFIG_IDF_TARGET_ESP32

// ESP32 Touch Sensor Benchmark
void benchmarkESP32Touch() {
  printHeader("I/O: CAPACITIVE TOUCH (ESP32)");

#if defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32H2) || defined(CONFIG_IDF_TARGET_ESP32C3)
  SERIAL_OUT.println(F_STR("Touch sensors not available on this ESP32 variant"));
  return;
#else
  // Touch pins vary by ESP32 variant
  int touchPin;
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
  touchPin = 1;  // T1 = GPIO1
#else
  touchPin = 4;  // T0 = GPIO4 on classic ESP32
#endif

  SERIAL_OUT.print(F_STR("Testing touch pin: GPIO"));
  SERIAL_OUT.println(touchPin);

  // Warm up reads
  for (int i = 0; i < 10; i++) {
    touchRead(touchPin);
  }

  // Touch read speed benchmark
  volatile uint32_t touchSum = 0;
  startBenchmark();
  for (int i = 0; i < 1000; i++) {
    touchSum += touchRead(touchPin);
  }
  unsigned long touchTime = endBenchmark();

  SERIAL_OUT.print(F_STR("touchRead() (1000 ops): "));
  SERIAL_OUT.print(touchTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(1000.0 / touchTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Average touch value: "));
  SERIAL_OUT.println(touchSum / 1000);

  // Touch threshold detection timing
  uint16_t baseline = touchRead(touchPin);
  SERIAL_OUT.print(F_STR("Baseline (no touch): "));
  SERIAL_OUT.println(baseline);

  SERIAL_OUT.print(F_STR("Time per read: "));
  SERIAL_OUT.print(touchTime / 1000.0, 2);
  SERIAL_OUT.println(F_STR(" us"));
#endif
}

// ESP32 Light Sleep Benchmark
void benchmarkESP32Sleep() {
  printHeader("POWER: LIGHT SLEEP TIMING (ESP32)");

  SERIAL_OUT.println(F_STR("Testing light sleep wake latency..."));
  SERIAL_OUT.println(F_STR("(Deep sleep would reset the MCU)"));
  SERIAL_OUT.flush();

  // Configure timer wake up for light sleep
  const uint64_t sleepTimeUs = 1000;  // 1ms sleep

  // Measure wake latency over multiple iterations
  unsigned long totalWakeTime = 0;
  const int iterations = 20;

  for (int i = 0; i < iterations; i++) {
    esp_sleep_enable_timer_wakeup(sleepTimeUs);

    unsigned long beforeSleep = micros();
    esp_light_sleep_start();
    unsigned long afterWake = micros();

    unsigned long actualSleep = afterWake - beforeSleep;
    totalWakeTime += actualSleep;

    yield();
  }

  float avgSleepTime = (float)totalWakeTime / iterations;
  float wakeOverhead = avgSleepTime - sleepTimeUs;

  SERIAL_OUT.print(F_STR("Target sleep time: "));
  SERIAL_OUT.print(sleepTimeUs);
  SERIAL_OUT.println(F_STR(" us"));

  SERIAL_OUT.print(F_STR("Average actual time: "));
  SERIAL_OUT.print(avgSleepTime, 1);
  SERIAL_OUT.println(F_STR(" us"));

  SERIAL_OUT.print(F_STR("Wake overhead: "));
  SERIAL_OUT.print(wakeOverhead, 1);
  SERIAL_OUT.println(F_STR(" us"));

  SERIAL_OUT.print(F_STR("Iterations: "));
  SERIAL_OUT.println(iterations);

  // Test different sleep durations (with timeout protection)
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("Sleep duration accuracy:"));

  uint64_t testDurations[] = { 100, 500, 1000, 5000, 10000 };
  for (int d = 0; d < 5; d++) {
    // Add timeout check to prevent freezing on some variants
    unsigned long testStart = millis();
    
    esp_sleep_enable_timer_wakeup(testDurations[d]);
    unsigned long before = micros();
    
    // Use timeout - if sleep doesn't wake in 50ms, something is wrong
    unsigned long timeout = 50000;  // 50ms timeout
    while ((micros() - before) < timeout) {
      esp_light_sleep_start();
      unsigned long actual = micros() - before;
      
      if (actual > 0) {
        float error = ((float)actual - testDurations[d]) / testDurations[d] * 100;

        SERIAL_OUT.print(F_STR("  Target "));
        SERIAL_OUT.print((uint32_t)testDurations[d]);
        SERIAL_OUT.print(F_STR(" us -> Actual "));
        SERIAL_OUT.print(actual);
        SERIAL_OUT.print(F_STR(" us ("));
        SERIAL_OUT.print(error, 1);
        SERIAL_OUT.println(F_STR("% error)"));
        break;
      }
    }
    
    // Check if timeout occurred
    if ((millis() - testStart) > 100) {
      SERIAL_OUT.println(F_STR("  [Sleep test timeout - skipping remaining tests]"));
      break;
    }
    
    yield();
  }
}

// ESP32 Hardware AES Benchmark
void benchmarkESP32AES() {
  printHeader("CRYPTO: HARDWARE AES (ESP32)");

  SERIAL_OUT.println(F_STR("Using ESP32 hardware AES accelerator"));

  // Test data
  uint8_t key[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                      0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
  uint8_t plaintext[16] = { 0 };
  uint8_t ciphertext[16] = { 0 };
  uint8_t decrypted[16] = { 0 };

  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);

  // AES-128 Encryption benchmark
  mbedtls_aes_setkey_enc(&aes, key, 128);

  volatile uint32_t checksum = 0;
  startBenchmark();
  for (int i = 0; i < 1000; i++) {
    plaintext[0] = i & 0xFF;
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, plaintext, ciphertext);
    checksum += ciphertext[0];
  }
  unsigned long encTime = endBenchmark();

  SERIAL_OUT.print(F_STR("AES-128 Encrypt (1000 blocks): "));
  SERIAL_OUT.print(encTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(1000.0 / encTime * 1000);
  SERIAL_OUT.println(F_STR(" blocks/ms)"));

  // AES-128 Decryption benchmark
  mbedtls_aes_setkey_dec(&aes, key, 128);

  startBenchmark();
  for (int i = 0; i < 1000; i++) {
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, ciphertext, decrypted);
    checksum += decrypted[0];
  }
  unsigned long decTime = endBenchmark();

  SERIAL_OUT.print(F_STR("AES-128 Decrypt (1000 blocks): "));
  SERIAL_OUT.print(decTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(1000.0 / decTime * 1000);
  SERIAL_OUT.println(F_STR(" blocks/ms)"));

  // Throughput calculation (16 bytes per block)
  float encThroughput = (1000 * 16) / (encTime / 1000000.0) / 1024;
  float decThroughput = (1000 * 16) / (decTime / 1000000.0) / 1024;

  SERIAL_OUT.print(F_STR("Encrypt throughput: "));
  SERIAL_OUT.print(encThroughput, 1);
  SERIAL_OUT.println(F_STR(" KB/s"));

  SERIAL_OUT.print(F_STR("Decrypt throughput: "));
  SERIAL_OUT.print(decThroughput, 1);
  SERIAL_OUT.println(F_STR(" KB/s"));

  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println(checksum);

  mbedtls_aes_free(&aes);
}

#endif  // ESP32

// ==================== RP2040 ADDITIONAL BENCHMARKS ====================

#if defined(ARDUINO_ARCH_RP2040)

// RP2040 DMA Benchmark
void benchmarkRP2040DMA() {
  printHeader("MEMORY: DMA vs CPU COPY (RP2040)");

  const size_t bufSize = 4096;
  uint8_t *src = (uint8_t *)malloc(bufSize);
  uint8_t *dst = (uint8_t *)malloc(bufSize);

  if (!src || !dst) {
    SERIAL_OUT.println(F_STR("Failed to allocate buffers"));
    if (src) free(src);
    if (dst) free(dst);
    return;
  }

  // Initialize source
  for (size_t i = 0; i < bufSize; i++) {
    src[i] = i & 0xFF;
  }

  // CPU memcpy benchmark
  startBenchmark();
  for (int i = 0; i < 100; i++) {
    memcpy(dst, src, bufSize);
  }
  unsigned long cpuTime = endBenchmark();

  SERIAL_OUT.print(F_STR("CPU memcpy ("));
  SERIAL_OUT.print(bufSize * 100);
  SERIAL_OUT.print(F_STR(" bytes): "));
  SERIAL_OUT.print(cpuTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print((bufSize * 100.0) / cpuTime);
  SERIAL_OUT.println(F_STR(" MB/s)"));

  // DMA benchmark
  int dma_chan = dma_claim_unused_channel(true);

  dma_channel_config c = dma_channel_get_default_config(dma_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, true);

  startBenchmark();
  for (int i = 0; i < 100; i++) {
    dma_channel_configure(dma_chan, &c, dst, src, bufSize, true);
    dma_channel_wait_for_finish_blocking(dma_chan);
  }
  unsigned long dmaTime = endBenchmark();

  SERIAL_OUT.print(F_STR("DMA transfer ("));
  SERIAL_OUT.print(bufSize * 100);
  SERIAL_OUT.print(F_STR(" bytes): "));
  SERIAL_OUT.print(dmaTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print((bufSize * 100.0) / dmaTime);
  SERIAL_OUT.println(F_STR(" MB/s)"));

  // Speedup
  SERIAL_OUT.print(F_STR("DMA speedup: "));
  SERIAL_OUT.print((float)cpuTime / dmaTime, 2);
  SERIAL_OUT.println(F_STR("x"));

  // Verify correctness
  bool match = true;
  for (size_t i = 0; i < bufSize; i++) {
    if (dst[i] != src[i]) {
      match = false;
      break;
    }
  }
  SERIAL_OUT.print(F_STR("Data verification: "));
  SERIAL_OUT.println(match ? F_STR("PASS") : F_STR("FAIL"));

  dma_channel_unclaim(dma_chan);
  free(src);
  free(dst);
}

// RP2040 PIO Benchmark
void benchmarkRP2040PIO() {
  printHeader("I/O: PIO STATE MACHINE (RP2040)");

  SERIAL_OUT.println(F_STR("RP2040 Programmable I/O Information:"));
  SERIAL_OUT.println(F_STR("  2x PIO blocks, 4 state machines each"));
  SERIAL_OUT.print(F_STR("  System clock: "));
  SERIAL_OUT.print(clock_get_hz(clk_sys) / 1000000);
  SERIAL_OUT.println(F_STR(" MHz"));

  // PIO can run at system clock speed
  float maxPioFreq = clock_get_hz(clk_sys) / 1000000.0;
  SERIAL_OUT.print(F_STR("  Max PIO frequency: "));
  SERIAL_OUT.print(maxPioFreq, 0);
  SERIAL_OUT.println(F_STR(" MHz"));

  // Theoretical GPIO toggle rate (2 cycles per toggle)
  SERIAL_OUT.print(F_STR("  Theoretical GPIO toggle: "));
  SERIAL_OUT.print(maxPioFreq / 2, 0);
  SERIAL_OUT.println(F_STR(" MHz"));

  // Compare with digitalWrite
  const int testPin = 25;  // LED pin
  pinMode(testPin, OUTPUT);

  volatile uint32_t toggles = 0;
  startBenchmark();
  for (int i = 0; i < 10000; i++) {
    digitalWrite(testPin, HIGH);
    digitalWrite(testPin, LOW);
    toggles += 2;
  }
  unsigned long gpioTime = endBenchmark();

  float gpioFreq = (toggles / 2.0) / (gpioTime / 1000000.0) / 1000;

  SERIAL_OUT.print(F_STR("  digitalWrite toggle rate: "));
  SERIAL_OUT.print(gpioFreq, 1);
  SERIAL_OUT.println(F_STR(" kHz"));

  // Direct register toggle
  startBenchmark();
  for (int i = 0; i < 100000; i++) {
    sio_hw->gpio_set = 1ul << testPin;
    sio_hw->gpio_clr = 1ul << testPin;
  }
  unsigned long regTime = endBenchmark();

  float regFreq = 100000.0 / (regTime / 1000000.0) / 1000000;

  SERIAL_OUT.print(F_STR("  Direct register toggle: "));
  SERIAL_OUT.print(regFreq, 2);
  SERIAL_OUT.println(F_STR(" MHz"));

  SERIAL_OUT.println();
  SERIAL_OUT.print(F_STR("PIO speedup potential: "));
  SERIAL_OUT.print(maxPioFreq / 2 / (regFreq), 0);
  SERIAL_OUT.println(F_STR("x over direct register"));
}

// RP2040 Interpolator Benchmark
void benchmarkRP2040Interpolator() {
  printHeader("COMPUTE: HARDWARE INTERPOLATOR (RP2040)");

  SERIAL_OUT.println(F_STR("RP2040 has 2 hardware interpolators per core"));
  SERIAL_OUT.println(F_STR("Used for: affine texture mapping, lane operations"));
  SERIAL_OUT.println();

  // Configure interpolator 0 for simple linear interpolation
  interp_config cfg = interp_default_config();
  interp_config_set_blend(&cfg, true);
  interp_set_config(interp0, 0, &cfg);

  // Benchmark interpolator operations
  volatile uint32_t result = 0;

  startBenchmark();
  for (int i = 0; i < 10000; i++) {
    // Set base and accumulator values
    interp0->base[0] = 0;
    interp0->base[1] = 1000;
    interp0->accum[0] = i % 256;  // Blend factor 0-255

    // Read interpolated result
    result += interp0->peek[0];
  }
  unsigned long interpTime = endBenchmark();

  SERIAL_OUT.print(F_STR("Interpolator ops (10000): "));
  SERIAL_OUT.print(interpTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(10000.0 / interpTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  // Compare with software interpolation
  volatile int32_t softResult = 0;
  startBenchmark();
  for (int i = 0; i < 10000; i++) {
    int32_t a = 0;
    int32_t b = 1000;
    int32_t t = i % 256;
    softResult += a + ((b - a) * t) / 256;
  }
  unsigned long softTime = endBenchmark();

  SERIAL_OUT.print(F_STR("Software lerp (10000): "));
  SERIAL_OUT.print(softTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(10000.0 / softTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  SERIAL_OUT.print(F_STR("Interpolator speedup: "));
  SERIAL_OUT.print((float)softTime / interpTime, 2);
  SERIAL_OUT.println(F_STR("x"));

  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println(result + softResult);
}

// RP2040 PWM Benchmark
void benchmarkRP2040PWM() {
  printHeader("I/O: PWM HARDWARE (RP2040)");

  const int pwmPin = 25;  // LED pin

  SERIAL_OUT.println(F_STR("RP2040 PWM Features:"));
  SERIAL_OUT.println(F_STR("  8 PWM slices, 2 channels each (16 outputs)"));
  SERIAL_OUT.println(F_STR("  16-bit counter, 16-bit wrap value"));

  uint32_t sysClk = clock_get_hz(clk_sys);
  SERIAL_OUT.print(F_STR("  System clock: "));
  SERIAL_OUT.print(sysClk / 1000000);
  SERIAL_OUT.println(F_STR(" MHz"));

  // Get slice for our pin
  uint slice_num = pwm_gpio_to_slice_num(pwmPin);
  uint channel = pwm_gpio_to_channel(pwmPin);

  SERIAL_OUT.print(F_STR("  Pin "));
  SERIAL_OUT.print(pwmPin);
  SERIAL_OUT.print(F_STR(" -> Slice "));
  SERIAL_OUT.print(slice_num);
  SERIAL_OUT.print(F_STR(", Channel "));
  SERIAL_OUT.println(channel == PWM_CHAN_A ? 'A' : 'B');

  // Configure for maximum frequency
  gpio_set_function(pwmPin, GPIO_FUNC_PWM);

  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, 1.0f);  // No division
  pwm_config_set_wrap(&config, 255);     // 8-bit resolution
  pwm_init(slice_num, &config, true);

  float pwmFreq = (float)sysClk / 256;
  SERIAL_OUT.print(F_STR("  8-bit PWM frequency: "));
  SERIAL_OUT.print(pwmFreq / 1000, 1);
  SERIAL_OUT.println(F_STR(" kHz"));

  // Benchmark duty cycle updates
  startBenchmark();
  for (int i = 0; i < 10000; i++) {
    pwm_set_gpio_level(pwmPin, i & 0xFF);
  }
  unsigned long updateTime = endBenchmark();

  SERIAL_OUT.print(F_STR("Duty cycle updates (10000): "));
  SERIAL_OUT.print(updateTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(10000.0 / updateTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  // Test different resolutions
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("PWM frequency vs resolution:"));

  uint16_t resolutions[] = { 256, 1024, 4096, 16384, 65535 };
  const char *resNames[] = { "8-bit", "10-bit", "12-bit", "14-bit", "16-bit" };

  for (int r = 0; r < 5; r++) {
    pwm_config_set_wrap(&config, resolutions[r] - 1);
    pwm_init(slice_num, &config, true);

    float freq = (float)sysClk / resolutions[r];
    SERIAL_OUT.print(F_STR("  "));
    SERIAL_OUT.print(resNames[r]);
    SERIAL_OUT.print(F_STR(": "));
    if (freq >= 1000000) {
      SERIAL_OUT.print(freq / 1000000, 2);
      SERIAL_OUT.println(F_STR(" MHz"));
    } else {
      SERIAL_OUT.print(freq / 1000, 1);
      SERIAL_OUT.println(F_STR(" kHz"));
    }
  }

  // Cleanup
  pwm_set_enabled(slice_num, false);
  gpio_set_function(pwmPin, GPIO_FUNC_SIO);
  pinMode(pwmPin, OUTPUT);
  digitalWrite(pwmPin, LOW);
}

// Pico W BLE Benchmark
#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
void benchmarkPicoWBLE() {
  printHeader("WIRELESS: BLE (Pico W)");

  SERIAL_OUT.println(F_STR("Pico W has BLE via CYW43439 module"));
  SERIAL_OUT.println(F_STR("BLE requires BTstack library"));
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("To use BLE on Pico W:"));
  SERIAL_OUT.println(F_STR("  1. Include <BTstackLib.h>"));
  SERIAL_OUT.println(F_STR("  2. Call BTstack.setup()"));
  SERIAL_OUT.println(F_STR("  3. Use BTstack API for scanning/advertising"));
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("Note: Full BLE benchmark requires BTstack integration"));
}
#endif

#endif  // ARDUINO_ARCH_RP2040

// ==================== UNO R4 WIFI ADDITIONAL BENCHMARKS ====================

#if defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)

// Uno R4 DAC Benchmark
void benchmarkUnoR4DAC() {
  printHeader("I/O: DAC OUTPUT (Uno R4)");

  SERIAL_OUT.println(F_STR("Renesas RA4M1 has 1x 12-bit DAC on pin A0/DAC"));

  const int dacPin = DAC;

  // Set 12-bit resolution
  analogWriteResolution(12);

  // DAC write speed benchmark
  volatile uint32_t dacValue = 0;
  startBenchmark();
  for (int i = 0; i < 10000; i++) {
    analogWrite(dacPin, dacValue & 0xFFF);
    dacValue++;
  }
  unsigned long dacTime = endBenchmark();

  SERIAL_OUT.print(F_STR("analogWrite() 12-bit (10000 ops): "));
  SERIAL_OUT.print(dacTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(10000.0 / dacTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  // DAC ramp test
  startBenchmark();
  for (int cycle = 0; cycle < 10; cycle++) {
    for (int val = 0; val < 4096; val += 16) {
      analogWrite(dacPin, val);
    }
  }
  unsigned long rampTime = endBenchmark();

  uint32_t rampOps = 10 * (4096 / 16);
  SERIAL_OUT.print(F_STR("DAC ramp (10 cycles, "));
  SERIAL_OUT.print(rampOps);
  SERIAL_OUT.print(F_STR(" writes): "));
  SERIAL_OUT.print(rampTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print((float)rampOps / rampTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  // Calculate effective sample rate
  float sampleRate = rampOps / (rampTime / 1000000.0);
  SERIAL_OUT.print(F_STR("Effective sample rate: "));
  SERIAL_OUT.print(sampleRate / 1000, 1);
  SERIAL_OUT.println(F_STR(" kHz"));

  // Reset to 8-bit and clear
  analogWriteResolution(8);
  analogWrite(dacPin, 0);
}

// Uno R4 RTC Benchmark
void benchmarkUnoR4RTC() {
  printHeader("TIMING: INTERNAL RTC (Uno R4)");

  SERIAL_OUT.println(F_STR("Renesas RA4M1 has built-in RTC"));

  // Initialize RTC
  RTC.begin();

  // Set initial time
  RTCTime startTime(1, Month::JANUARY, 2024, 12, 0, 0, DayOfWeek::MONDAY, SaveLight::SAVING_TIME_INACTIVE);
  RTC.setTime(startTime);

  // RTC read speed benchmark
  RTCTime currentTime;
  volatile uint32_t timeSum = 0;

  startBenchmark();
  for (int i = 0; i < 1000; i++) {
    RTC.getTime(currentTime);
    timeSum += currentTime.getSeconds();
  }
  unsigned long readTime = endBenchmark();

  SERIAL_OUT.print(F_STR("RTC.getTime() (1000 ops): "));
  SERIAL_OUT.print(readTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(1000.0 / readTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  // RTC write speed benchmark
  startBenchmark();
  for (int i = 0; i < 100; i++) {
    RTCTime newTime(1, Month::JANUARY, 2024, 12, 0, i % 60, DayOfWeek::MONDAY, SaveLight::SAVING_TIME_INACTIVE);
    RTC.setTime(newTime);
  }
  unsigned long writeTime = endBenchmark();

  SERIAL_OUT.print(F_STR("RTC.setTime() (100 ops): "));
  SERIAL_OUT.print(writeTime);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(100.0 / writeTime * 1000);
  SERIAL_OUT.println(F_STR(" ops/ms)"));

  // RTC tick accuracy test
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("RTC tick accuracy (1 second test):"));

  RTC.getTime(currentTime);
  int startSec = currentTime.getSeconds();
  unsigned long startMicros = micros();

  // Wait for second to change
  int currentSec = startSec;
  while (currentSec == startSec) {
    RTC.getTime(currentTime);
    currentSec = currentTime.getSeconds();
  }

  // Now wait for next second change
  startSec = currentSec;
  startMicros = micros();
  while (currentSec == startSec) {
    RTC.getTime(currentTime);
    currentSec = currentTime.getSeconds();
  }
  unsigned long elapsed = micros() - startMicros;

  float error = ((float)elapsed - 1000000) / 1000000 * 100;
  SERIAL_OUT.print(F_STR("  1 RTC second = "));
  SERIAL_OUT.print(elapsed);
  SERIAL_OUT.print(F_STR(" us ("));
  SERIAL_OUT.print(error, 3);
  SERIAL_OUT.println(F_STR("% error)"));

  SERIAL_OUT.print(F_STR("Checksum: "));
  SERIAL_OUT.println(timeSum);
}

// Uno R4 WiFi BLE Benchmark
#if defined(ARDUINO_UNOR4_WIFI)
void benchmarkUnoR4BLE() {
  printHeader("WIRELESS: BLE (Uno R4 WiFi)");

  SERIAL_OUT.println(F_STR("Uno R4 WiFi has BLE via ESP32-S3 module"));
  SERIAL_OUT.println(F_STR("BLE requires ArduinoBLE library"));
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("To use BLE on Uno R4 WiFi:"));
  SERIAL_OUT.println(F_STR("  1. Install ArduinoBLE library"));
  SERIAL_OUT.println(F_STR("  2. #include <ArduinoBLE.h>"));
  SERIAL_OUT.println(F_STR("  3. Call BLE.begin()"));
  SERIAL_OUT.println(F_STR("  4. Use BLE.scan() for device discovery"));
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("Note: Full BLE benchmark requires ArduinoBLE"));
}
#endif

#endif  // ARDUINO_UNOR4_WIFI || ARDUINO_UNOR4_MINIMA

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

// Watchdog Timer Benchmark
void benchmarkWatchdog() {
  printHeader("SYSTEM: WATCHDOG TIMER");

#if defined(ESP32)
  SERIAL_OUT.println(F_STR("ESP32 Watchdog Information:"));
  SERIAL_OUT.println(F_STR("  Task WDT: Monitors FreeRTOS tasks"));
  SERIAL_OUT.println(F_STR("  Interrupt WDT: Monitors interrupt handling"));
  SERIAL_OUT.println(F_STR("  RTC WDT: Low-power watchdog"));
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("Watchdog is managed by ESP-IDF/FreeRTOS"));
  SERIAL_OUT.println(F_STR("Default timeout: ~5 seconds"));

#elif defined(ARDUINO_ARCH_RP2040)
  SERIAL_OUT.println(F_STR("RP2040 Watchdog Features:"));
  SERIAL_OUT.println(F_STR("  Max timeout: ~8.3 seconds"));
  SERIAL_OUT.println(F_STR("  Resolution: 1 microsecond"));
  SERIAL_OUT.println();

  // Test watchdog setup time (but don't enable it)
  startBenchmark();
  for (int i = 0; i < 1000; i++) {
    // Just measure time to check if WDT caused reset
    bool wasWdt = watchdog_caused_reboot();
    (void)wasWdt;
  }
  unsigned long wdtCheckTime = endBenchmark();

  SERIAL_OUT.print(F_STR("watchdog_caused_reboot() (1000 calls): "));
  SERIAL_OUT.print(wdtCheckTime);
  SERIAL_OUT.println(F_STR(" us"));

  SERIAL_OUT.print(F_STR("Last reset was WDT: "));
  SERIAL_OUT.println(watchdog_caused_reboot() ? F_STR("YES") : F_STR("NO"));

#elif defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  SERIAL_OUT.println(F_STR("Renesas RA4M1 Watchdog:"));
  SERIAL_OUT.println(F_STR("  Independent WDT (IWDT)"));
  SERIAL_OUT.println(F_STR("  Configurable timeout periods"));
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("Use WDT library for configuration"));

#elif defined(__AVR__)
  SERIAL_OUT.println(F_STR("AVR Watchdog Features:"));
  SERIAL_OUT.println(F_STR("  Timeout options: 16ms to 8s"));
  SERIAL_OUT.println(F_STR("  Can trigger reset or interrupt"));
  SERIAL_OUT.println();

  // Check reset cause (register names differ between classic AVR and megaAVR)
  #if defined(BOARD_MEGAAVR)
    // megaAVR (AVR128DA28, etc.) uses RSTFR register
    #if defined(RSTFR)
      uint8_t rstfr = RSTFR;
      SERIAL_OUT.print(F_STR("Reset cause: "));
      if (rstfr & 0x01) SERIAL_OUT.print(F_STR("Power-on "));
      if (rstfr & 0x02) SERIAL_OUT.print(F_STR("Brown-out "));
      if (rstfr & 0x04) SERIAL_OUT.print(F_STR("External "));
      if (rstfr & 0x08) SERIAL_OUT.print(F_STR("Watchdog "));
      SERIAL_OUT.println();
    #else
      SERIAL_OUT.println(F_STR("Reset cause: Not available on this megaAVR variant"));
    #endif
  #else
    // Classic AVR (ATmega328P, etc.) uses MCUSR register
    uint8_t mcusr = MCUSR;
    SERIAL_OUT.print(F_STR("Reset cause: "));
    if (mcusr & (1 << WDRF)) SERIAL_OUT.print(F_STR("WDT "));
    if (mcusr & (1 << BORF)) SERIAL_OUT.print(F_STR("Brown-out "));
    if (mcusr & (1 << EXTRF)) SERIAL_OUT.print(F_STR("External "));
    if (mcusr & (1 << PORF)) SERIAL_OUT.print(F_STR("Power-on "));
    SERIAL_OUT.println();
  #endif

#else
  SERIAL_OUT.println(F_STR("Watchdog information not available for this board"));
#endif
}

// Sleep Mode Benchmark
void benchmarkSleepModes() {
  printHeader("POWER: SLEEP MODE TIMING");

#if defined(ESP32)
  // Already covered in benchmarkESP32Sleep()
  SERIAL_OUT.println(F_STR("See ESP32 Light Sleep benchmark above"));

#elif defined(ARDUINO_ARCH_RP2040)
  SERIAL_OUT.println(F_STR("RP2040 Sleep Modes:"));
  SERIAL_OUT.println(F_STR("  DORMANT: Deep sleep, wake on GPIO/RTC"));
  SERIAL_OUT.println(F_STR("  SLEEP: WFI/WFE, wake on interrupt"));
  SERIAL_OUT.println();

  // Measure __wfi timing
  SERIAL_OUT.println(F_STR("Testing WFI (Wait For Interrupt):"));

  // Set up a timer to wake us
  unsigned long wakeTime = 0;
  unsigned long sleepStart = 0;

  // Use systick interrupt to wake (happens every 1ms)
  sleepStart = micros();
  __wfi();
  wakeTime = micros() - sleepStart;

  SERIAL_OUT.print(F_STR("  WFI wake time: "));
  SERIAL_OUT.print(wakeTime);
  SERIAL_OUT.println(F_STR(" us"));

  // Multiple WFI cycles
  startBenchmark();
  for (int i = 0; i < 100; i++) {
    __wfi();
  }
  unsigned long wfiTime = endBenchmark();

  SERIAL_OUT.print(F_STR("  100x WFI cycles: "));
  SERIAL_OUT.print(wfiTime);
  SERIAL_OUT.println(F_STR(" us"));

#elif defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  SERIAL_OUT.println(F_STR("Renesas RA4M1 Low Power Modes:"));
  SERIAL_OUT.println(F_STR("  Sleep: CPU stopped, peripherals active"));
  SERIAL_OUT.println(F_STR("  Deep Sleep: Most peripherals stopped"));
  SERIAL_OUT.println(F_STR("  Software Standby: Lowest power"));
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("Use LowPower library for sleep control"));

#elif defined(__AVR__)
  SERIAL_OUT.println(F_STR("AVR Sleep Modes:"));
  SERIAL_OUT.println(F_STR("  IDLE: CPU stopped"));
  SERIAL_OUT.println(F_STR("  ADC Noise Reduction: ADC runs"));
  SERIAL_OUT.println(F_STR("  Power-down: Deepest sleep"));
  SERIAL_OUT.println(F_STR("  Power-save: Timer2 + async"));
  SERIAL_OUT.println(F_STR("  Standby: Oscillator runs"));
  SERIAL_OUT.println();

  // Test idle mode timing
  set_sleep_mode(SLEEP_MODE_IDLE);

  // Use Timer0 interrupt to wake (happens frequently)
  unsigned long idleStart = micros();
  sleep_enable();
  sleep_cpu();
  sleep_disable();
  unsigned long idleTime = micros() - idleStart;

  SERIAL_OUT.print(F_STR("Idle mode wake: "));
  SERIAL_OUT.print(idleTime);
  SERIAL_OUT.println(F_STR(" us"));

#else
  SERIAL_OUT.println(F_STR("Sleep mode info not available for this board"));
#endif
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
  delay(2000);  // Wait for serial connection
#endif

  calibrateBenchmarkTime();

  SERIAL_OUT.println();
  SERIAL_OUT.println();
  printDivider();
  SERIAL_OUT.println(F_STR("  UNIVERSAL ARDUINO BENCHMARK SUITE"));
  printDivider();
  SERIAL_OUT.println();

  // System info
  printSystemInfo();

  // CPU benchmarks
  benchmarkIntegerOps();
  benchmarkFloatOps();
  benchmarkStringOps();
  benchmarkCPUStress();

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
  benchmarkAnalogIO();
  benchmarkSerial();

  // Board-specific
  benchmarkFlash();
#ifdef HAS_LED_MATRIX
  benchmarkLEDMatrix();
#endif
#ifdef HAS_WIFI
  benchmarkWiFi();
#endif
#ifdef HAS_BLE
  benchmarkBLE();
#endif

  // Advanced benchmarks
  benchmarkAdvancedMath();
  benchmarkTimingPrecision();
  benchmarkStackDepth();
#if defined(ARDUINO_ARCH_RP2040)
  benchmarkSHA1();
#endif
#if defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
  benchmarkMultiCore();
#endif
#if defined(ESP32)
  benchmarkESP32Crypto();
  benchmarkHardwareRNG();
#endif
#if defined(BOARD_STM32U5)
  benchmarkHardwareRNG();
#endif
#if defined(ARDUINO_UNOR4_WIFI)
  benchmarkSoftwareATSE();
#endif
#if defined(BOARD_STM32U5) && defined(HAS_DUAL_PROCESSOR)
  //benchmarkArduinoBridge();
#endif

  // Multiduino-specific benchmarks
#ifdef HAS_RTC
  benchmarkRTC();
#endif

  // ========== NEW BENCHMARKS ==========

  // ESP32 Additional Benchmarks
#if defined(ESP32)
#if defined(CONFIG_IDF_TARGET_ESP32)
  benchmarkESP32DAC();
#endif
  benchmarkESP32Touch();
  benchmarkESP32Sleep();
  benchmarkESP32AES();
#endif

  // RP2040 Additional Benchmarks
#if defined(ARDUINO_ARCH_RP2040)
  benchmarkRP2040DMA();
  benchmarkRP2040PIO();
  benchmarkRP2040Interpolator();
  benchmarkRP2040PWM();
#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  benchmarkPicoWBLE();
#endif
#endif

  // Uno R4 Additional Benchmarks
#if defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  benchmarkUnoR4DAC();
  benchmarkUnoR4RTC();
#if defined(ARDUINO_UNOR4_WIFI)
  benchmarkUnoR4BLE();
#endif
#endif

  // General Benchmarks (all boards)
  benchmarkPWM();
  benchmarkInterruptLatency();
  benchmarkSPI();
  benchmarkWatchdog();
  benchmarkSleepModes();

  // Final summary
  printHeader("BENCHMARK COMPLETE!");
  SERIAL_OUT.println(F_STR("Results saved in Serial Monitor."));
  SERIAL_OUT.println(F_STR("You can copy/paste the output for analysis."));
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("To run again, press the RESET button or"));
  SERIAL_OUT.println(F_STR("re-upload the sketch."));
  printDivider();
}

void loop() {
  // Nothing to do - benchmark runs once in setup()
  delay(1000);
}