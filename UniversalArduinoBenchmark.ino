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

// Multiduino (your custom Uno-derivative)
#elif defined(ARDUINO_AVR_MULTIDUINO)
#define BOARD_NAME "Multiduino"
#include <EEPROM.h>
#define BOARD_AVR

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
#define SERIAL_BAUD 115200

// ==================== GLOBAL VARIABLES ====================
uint8_t testBuffer[256];

// ==================== HELPER FUNCTIONS ====================

void printDivider() {
  Serial.println(F("========================================"));
}

void printHeader(const char* title) {
  Serial.println();
  printDivider();
  Serial.println(title);
  printDivider();
}

unsigned long benchmarkStart;

void startBenchmark() {
  benchmarkStart = micros();
}

unsigned long endBenchmark() {
  return micros() - benchmarkStart;
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
    Serial.println(F("Temperature sensor not available on this board"));
    Serial.println(F("Running stress test without temperature monitoring..."));
    Serial.println();
  }

  // Initial temperature reading
  float startTemp = 0;
#if defined(ESP32)
  startTemp = temperatureRead();
  Serial.print(F("Start Temperature: "));
  Serial.print(startTemp);
  Serial.println(F(" °C"));
#elif defined(ARDUINO_ARCH_RP2040)
  startTemp = analogReadTemp(3.3f);
  Serial.print(F("Start Temperature: "));
  Serial.print(startTemp);
  Serial.println(F(" °C"));
#elif defined(BOARD_TEENSY) && defined(__IMXRT1062__)
  startTemp = tempmonGetTemp();
  Serial.print(F("Start Temperature: "));
  Serial.print(startTemp);
  Serial.println(F(" °C"));
#endif

  Serial.print(F("Running intensive computation for 10 seconds..."));
  Serial.println();

  // CPU stress test - run all cores
  unsigned long stressStart = millis();
  unsigned long iterations = 0;
  volatile float result = 1.0f;

  while (millis() - stressStart < 10000) {
    // Mix of integer and float operations
    for (int i = 0; i < 100; i++) {
      result = result * 1.0001f + sqrt((float)i);
      result = sin(result) + cos(result);
      iterations++;
    }

    // Periodic yield to prevent watchdog timeout (but don't print)
    if ((iterations % 10000) == 0) {
#if defined(ARDUINO_ARCH_RP2040) || defined(ESP32) || defined(ESP8266)
      yield();
#endif
    }
  }

  unsigned long stressDuration = millis() - stressStart;

  Serial.println();
  Serial.print(F("Stress test complete: "));
  Serial.print(iterations);
  Serial.print(F(" iterations in "));
  Serial.print(stressDuration);
  Serial.println(F(" ms"));

  Serial.print(F("Performance: "));
  Serial.print((float)iterations / stressDuration);
  Serial.println(F(" iterations/ms"));

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

    Serial.println();
    Serial.print(F("Final Temperature: "));
    Serial.print(endTemp);
    Serial.println(F(" °C"));

    Serial.print(F("Total Temperature Gain: +"));
    Serial.print(endTemp - startTemp);
    Serial.println(F(" °C"));

    if (endTemp - startTemp > 10) {
      Serial.println(F("⚠️  Significant heating detected - ensure adequate cooling"));
    } else if (endTemp - startTemp > 5) {
      Serial.println(F("ℹ️  Normal temperature increase under load"));
    } else {
      Serial.println(F("✓ Minimal temperature increase - good thermal performance"));
    }
  }
}

void benchmarkIntegerOps() {
  printHeader("CPU: INTEGER OPERATIONS");

  // Addition - use volatile uint64_t to prevent optimization and overflow
  volatile uint64_t acc = 0;
  startBenchmark();
  for (uint32_t i = 0; i < BENCHMARK_ITERATIONS; i++) {
    acc += i;
  }
  unsigned long addTime = endBenchmark();
  Serial.print(F("Checksum: "));
  Serial.println((uint32_t)(acc & 0xFFFFFFFF));

  // Multiplication - use LCG-style updates to prevent optimization
  acc = 1;
  startBenchmark();
  for (uint32_t i = 1; i <= BENCHMARK_ITERATIONS / 100; i++) {
    acc = (acc * (i | 1)) & 0xFFFFFFFF;  // Ensure odd multiplier, prevent overflow
  }
  unsigned long mulTime = endBenchmark();
  Serial.print(F("Checksum: "));
  Serial.println((uint32_t)acc);

  // Division - vary both dividend and divisor
  acc = 0xFFFFFFFFULL;
  startBenchmark();
  for (uint32_t i = 1; i <= BENCHMARK_ITERATIONS / 100; i++) {
    uint32_t divisor = (i % 127) + 2;  // 2-128, avoid div-by-1
    acc = (acc / divisor) + i;         // Accumulate to prevent optimization
  }
  unsigned long divTime = endBenchmark();
  Serial.print(F("Checksum: "));
  Serial.println((uint32_t)acc);

  Serial.print(F("Addition ("));
  Serial.print(BENCHMARK_ITERATIONS);
  Serial.print(F(" ops): "));
  Serial.print(addTime);
  Serial.print(F(" μs ("));
  Serial.print((float)BENCHMARK_ITERATIONS / addTime * 1000);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Multiplication ("));
  Serial.print(BENCHMARK_ITERATIONS / 100);
  Serial.print(F(" ops): "));
  Serial.print(mulTime);
  Serial.print(F(" μs ("));
  Serial.print((float)(BENCHMARK_ITERATIONS / 100) / mulTime * 1000);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Division ("));
  Serial.print(BENCHMARK_ITERATIONS / 100);
  Serial.print(F(" ops): "));
  Serial.print(divTime);
  Serial.print(F(" μs ("));
  Serial.print((float)(BENCHMARK_ITERATIONS / 100) / divTime * 1000);
  Serial.println(F(" ops/ms)"));
}

void benchmarkFloatOps() {
  printHeader("CPU: FLOATING POINT OPERATIONS");

  // Float addition - volatile prevents optimization
  volatile float fresult = 0.0f;
  startBenchmark();
  for (uint32_t i = 0; i < BENCHMARK_ITERATIONS / 10; i++) {
    fresult += 3.14159f;
  }
  unsigned long faddTime = endBenchmark();
  Serial.print(F("Checksum: "));
  Serial.println(fresult);

  // Float multiplication
  fresult = 1.0f;
  startBenchmark();
  for (uint32_t i = 0; i < BENCHMARK_ITERATIONS / 10; i++) {
    fresult *= 1.0001f;
  }
  unsigned long fmulTime = endBenchmark();
  Serial.print(F("Checksum: "));
  Serial.println(fresult);

  // Sqrt - accumulate to prevent optimization
  fresult = 0.0f;
  startBenchmark();
  for (uint32_t i = 0; i < BENCHMARK_ITERATIONS / 100; i++) {
    fresult += sqrt((float)i);
  }
  unsigned long sqrtTime = endBenchmark();
  Serial.print(F("Checksum: "));
  Serial.println(fresult);

  // Sin/Cos - accumulate to prevent optimization
  fresult = 0.0f;
  startBenchmark();
  for (uint32_t i = 0; i < BENCHMARK_ITERATIONS / 100; i++) {
    fresult += sin((float)i / 100.0f) + cos((float)i / 100.0f);
  }
  unsigned long trigTime = endBenchmark();
  Serial.print(F("Checksum: "));
  Serial.println(fresult);

  Serial.print(F("Float Addition ("));
  Serial.print(BENCHMARK_ITERATIONS / 10);
  Serial.print(F(" ops): "));
  Serial.print(faddTime);
  Serial.print(F(" μs ("));
  Serial.print((float)(BENCHMARK_ITERATIONS / 10) / faddTime * 1000);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Float Multiply ("));
  Serial.print(BENCHMARK_ITERATIONS / 10);
  Serial.print(F(" ops): "));
  Serial.print(fmulTime);
  Serial.print(F(" μs ("));
  Serial.print((float)(BENCHMARK_ITERATIONS / 10) / fmulTime * 1000);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Square Root ("));
  Serial.print(BENCHMARK_ITERATIONS / 100);
  Serial.print(F(" ops): "));
  Serial.print(sqrtTime);
  Serial.print(F(" μs ("));
  Serial.print((float)(BENCHMARK_ITERATIONS / 100) / sqrtTime * 1000);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Sin/Cos ("));
  Serial.print(BENCHMARK_ITERATIONS / 100);
  Serial.print(F(" ops): "));
  Serial.print(trigTime);
  Serial.print(F(" μs ("));
  Serial.print((float)(BENCHMARK_ITERATIONS / 100) / trigTime * 1000);
  Serial.println(F(" ops/ms)"));
}

void benchmarkStringOps() {
  printHeader("CPU: STRING OPERATIONS");

  // String concatenation
  startBenchmark();
  String testString = "";
  for (int i = 0; i < 100; i++) {
    testString += "X";
  }
  unsigned long concatTime = endBenchmark();

  // String comparison
  String str1 = "TestString123";
  String str2 = "TestString123";
  startBenchmark();
  volatile bool cmpResult;
  for (int i = 0; i < 1000; i++) {
    cmpResult = (str1 == str2);
  }
  unsigned long cmpTime = endBenchmark();

  // Integer to String
  startBenchmark();
  String numStr;
  for (int i = 0; i < 1000; i++) {
    numStr = String(i);
  }
  unsigned long toStrTime = endBenchmark();

  Serial.print(F("Concatenation (100 ops): "));
  Serial.print(concatTime);
  Serial.print(F(" μs ("));
  Serial.print(100.0 / concatTime * 1000);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Comparison (1000 ops): "));
  Serial.print(cmpTime);
  Serial.print(F(" μs ("));
  Serial.print(1000.0 / cmpTime * 1000);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Int to String (1000 ops): "));
  Serial.print(toStrTime);
  Serial.print(F(" μs ("));
  Serial.print(1000.0 / toStrTime * 1000);
  Serial.println(F(" ops/ms)"));
}

// ==================== MEMORY BENCHMARKS ====================

void benchmarkSRAM() {
  printHeader("MEMORY: SRAM READ/WRITE");

  // Sequential write
  startBenchmark();
  for (uint16_t i = 0; i < 256; i++) {
    testBuffer[i] = (uint8_t)i;
  }
  unsigned long writeTime = endBenchmark();

  // Sequential read - accumulate to prevent optimization
  volatile uint32_t checksum = 0;
  startBenchmark();
  for (uint16_t i = 0; i < 256; i++) {
    checksum += testBuffer[i];
  }
  unsigned long readTime = endBenchmark();
  Serial.print(F("Read checksum: "));
  Serial.println((uint32_t)checksum);

  // Random access - accumulate to prevent optimization
  checksum = 0;
  startBenchmark();
  for (uint16_t i = 0; i < 256; i++) {
    uint8_t idx = (i * 7 + 13) % 256;  // Pseudo-random
    checksum += testBuffer[idx];
  }
  unsigned long randomTime = endBenchmark();
  Serial.print(F("Random checksum: "));
  Serial.println((uint32_t)checksum);

  Serial.print(F("Sequential Write (256 bytes): "));
  Serial.print(writeTime);
  Serial.print(F(" μs ("));
  Serial.print(256.0 / writeTime * 1000);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Sequential Read (256 bytes): "));
  Serial.print(readTime);
  Serial.print(F(" μs ("));
  Serial.print(256.0 / readTime * 1000);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Random Access (256 ops): "));
  Serial.print(randomTime);
  Serial.print(F(" μs ("));
  Serial.print(256.0 / randomTime * 1000);
  Serial.println(F(" ops/ms)"));
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
    Serial.print(F("EEPROM Size: "));
    Serial.print(eepromSize);
    Serial.println(F(" bytes"));

    // Write test (smaller sample)
    int testSize = min(64, eepromSize);

#if defined(ESP32) || defined(ESP8266)
    // ESP32/ESP8266: write to RAM buffer
    startBenchmark();
    for (int i = 0; i < testSize; i++) {
      EEPROM.write(i, (uint8_t)i);
    }
    unsigned long ramWriteTime = endBenchmark();

    Serial.print(F("RAM Buffer Write ("));
    Serial.print(testSize);
    Serial.print(F(" bytes): "));
    Serial.print(ramWriteTime);
    Serial.print(F(" μs ("));
    Serial.print((float)testSize / ramWriteTime * 1000);
    Serial.println(F(" ops/ms)"));

    // Commit multiple times to measure min/median/max (captures erase events)
    Serial.println(F("Flash Commit Test (10 commits):"));
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

    Serial.print(F("  Min: "));
    Serial.print(minCommit);
    Serial.print(F(" μs ("));
    Serial.print(minCommit / 1000.0);
    Serial.println(F(" ms)"));

    Serial.print(F("  Median: "));
    Serial.print(medianCommit);
    Serial.print(F(" μs ("));
    Serial.print(medianCommit / 1000.0);
    Serial.println(F(" ms)"));

    Serial.print(F("  Max: "));
    Serial.print(maxCommit);
    Serial.print(F(" μs ("));
    Serial.print(maxCommit / 1000.0);
    Serial.println(F(" ms) ← Includes erase"));
#else
    // AVR/native EEPROM: each write goes to hardware
    startBenchmark();
    for (int i = 0; i < testSize; i++) {
      EEPROM.write(i, (uint8_t)i);
    }
    unsigned long writeTime = endBenchmark();

    Serial.print(F("Hardware Write ("));
    Serial.print(testSize);
    Serial.print(F(" bytes): "));
    Serial.print(writeTime);
    Serial.print(F(" μs ("));
    Serial.print((float)testSize / writeTime * 1000);
    Serial.println(F(" ops/ms)"));
#endif

    // Read test - measure actual reads with checksum
    delay(10);  // Let EEPROM settle
    volatile uint32_t checksum = 0;
    startBenchmark();
    for (int i = 0; i < testSize; i++) {
      checksum += EEPROM.read(i);
    }
    unsigned long readTime = endBenchmark();

    Serial.print(F("Read ("));
    Serial.print(testSize);
    Serial.print(F(" bytes): "));
    Serial.print(readTime);
    Serial.print(F(" μs ("));
    Serial.print((float)testSize / readTime * 1000);
    Serial.println(F(" ops/ms)"));
    Serial.print(F("Read checksum: "));
    Serial.println((uint32_t)checksum);
  } else {
    Serial.println(F("EEPROM not available"));
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
    Serial.print(F("PSRAM Size: "));
    Serial.print(ESP.getPsramSize() / 1024);
    Serial.println(F(" KB"));

    Serial.print(F("Free PSRAM: "));
    Serial.print(ESP.getFreePsram() / 1024);
    Serial.println(F(" KB"));

    // Allocate test buffer in PSRAM
    uint8_t* psramBuffer = (uint8_t*)ps_malloc(4096);
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

      Serial.print(F("Write (4096 bytes): "));
      Serial.print(writeTime);
      Serial.print(F(" μs ("));
      Serial.print(4096.0 / writeTime * 1000);
      Serial.println(F(" ops/ms)"));

      Serial.print(F("Read (4096 bytes): "));
      Serial.print(readTime);
      Serial.print(F(" μs ("));
      Serial.print(4096.0 / readTime * 1000);
      Serial.println(F(" ops/ms)"));

      Serial.print(F("Read checksum: "));
      Serial.println((uint32_t)checksum);

      free(psramBuffer);
    } else {
      Serial.println(F("Failed to allocate PSRAM"));
    }
  } else {
    Serial.println(F("PSRAM not found"));
  }
}
#endif

// ==================== I/O BENCHMARKS ====================

void benchmarkDigitalIO() {
  printHeader("I/O: DIGITAL PIN OPERATIONS");

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
  testPin = 13;  // Classic Arduino default
#endif

  pinMode(testPin, OUTPUT);

  // digitalWrite benchmark
  volatile uint32_t dwOps = 0;
  startBenchmark();
  for (int i = 0; i < 1000; i++) {
    digitalWrite(testPin, HIGH);
    digitalWrite(testPin, LOW);
    dwOps += 2;
  }
  unsigned long writeTime = endBenchmark();

// Direct port manipulation (AVR only)
#ifdef __AVR__
  volatile uint8_t *out = portOutputRegister(digitalPinToPort(testPin));
  uint8_t mask = digitalPinToBitMask(testPin);
  volatile uint32_t portOps = 0;
  startBenchmark();
  for (int i = 0; i < 1000; i++) {
    *out |= mask;   // Set
    *out &= ~mask;  // Clear
    portOps += 2;
  }
  unsigned long portTime = endBenchmark();
#endif

// Direct register write (ESP32)
#ifdef ESP32
  volatile uint32_t regOps = 0;
  startBenchmark();
  for (int i = 0; i < 1000; i++) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
    gpio_set_level((gpio_num_t)testPin, 1);
    gpio_set_level((gpio_num_t)testPin, 0);
#else
    digitalWrite(testPin, HIGH);
    digitalWrite(testPin, LOW);
#endif
    regOps += 2;
  }
  unsigned long regTime = endBenchmark();
#endif

// Direct register write (RP2040)
#ifdef ARDUINO_ARCH_RP2040
  volatile uint32_t regOps = 0;
  startBenchmark();
  for (int i = 0; i < 1000; i++) {
    sio_hw->gpio_set = 1ul << testPin;  // Set
    sio_hw->gpio_clr = 1ul << testPin;  // Clear
    regOps += 2;
  }
  unsigned long regTime = endBenchmark();
#endif

  Serial.print(F("digitalWrite() ("));
  Serial.print(dwOps);
  Serial.print(F(" ops): "));
  Serial.print(writeTime);
  Serial.print(F(" μs ("));
  Serial.print(dwOps * 1000.0 / writeTime);
  Serial.println(F(" ops/ms)"));

#ifdef __AVR__
  Serial.print(F("Direct Port ("));
  Serial.print(portOps);
  Serial.print(F(" ops): "));
  Serial.print(portTime);
  Serial.print(F(" μs ("));
  Serial.print(portOps * 1000.0 / portTime);
  Serial.println(F(" ops/ms)"));
  Serial.print(F("Speedup: "));
  Serial.print((float)writeTime / portTime);
  Serial.println(F("x faster"));
#endif

#if defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
  Serial.print(F("Direct Register ("));
  Serial.print(regOps);
  Serial.print(F(" ops): "));
  Serial.print(regTime);
  Serial.print(F(" μs ("));
  Serial.print(regOps * 1000.0 / regTime);
  Serial.println(F(" ops/ms)"));
  Serial.print(F("Speedup: "));
  Serial.print((float)writeTime / regTime);
  Serial.println(F("x faster"));
#endif
}

void benchmarkAnalogIO() {
  printHeader("I/O: ANALOG OPERATIONS");

// Find analog pins
#if defined(ESP32)
  int analogInPin = 36;   // VP
  int analogOutPin = 25;  // DAC1
#elif defined(ESP8266)
  int analogInPin = A0;
  int analogOutPin = -1;  // No DAC
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

  // analogRead benchmark - accumulate to prevent optimization
  if (analogInPin >= 0) {
    volatile uint32_t sum = 0;
    startBenchmark();
    for (int i = 0; i < 100; i++) {
      sum += analogRead(analogInPin);
    }
    unsigned long readTime = endBenchmark();

    Serial.print(F("analogRead() (100 ops): "));
    Serial.print(readTime);
    Serial.print(F(" μs ("));
    Serial.print(100.0 / readTime * 1000);
    Serial.println(F(" ops/ms)"));
    Serial.print(F("ADC average: "));
    Serial.println((uint32_t)(sum / 100));
  }

  // analogWrite/PWM benchmark
  if (analogOutPin >= 0) {
    pinMode(analogOutPin, OUTPUT);
    volatile uint32_t iterations = 0;
    startBenchmark();
    for (int i = 0; i < 100; i++) {
      analogWrite(analogOutPin, i % 256);
      iterations++;
    }
    unsigned long writeTime = endBenchmark();

    Serial.print(F("analogWrite() ("));
    Serial.print(iterations);
    Serial.print(F(" ops): "));
    Serial.print(writeTime);
    Serial.print(F(" μs ("));
    Serial.print(iterations * 1000.0 / writeTime);
    Serial.println(F(" ops/ms)"));
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
  startBenchmark();
  for (int i = 0; i < 100; i++) {
    Serial.print(i);
  }
  unsigned long enqueueTime = endBenchmark();

  // Measure what flush() actually does
  unsigned long flushStart = micros();
  Serial.flush();  // Wait for TX buffer to drain to UART FIFO
  unsigned long flushTime = micros() - flushStart;

  // Calculate theoretical wire time (10 bits per byte: start + 8 data + stop)
  // At 115200 baud: each bit = 1/115200 sec = 8.68 μs
  // Each byte = 10 bits = 86.8 μs
  unsigned long theoreticalWireTime = (unsigned long)(expectedBytes * 10 * 1000000.0 / SERIAL_BAUD);

  Serial.println();
  Serial.print(F("Serial Enqueue ("));
  Serial.print(expectedBytes);
  Serial.print(F(" bytes): "));
  Serial.print(enqueueTime);
  Serial.print(F(" μs ("));
  Serial.print(expectedBytes * 1000.0 / enqueueTime);
  Serial.println(F(" bytes/ms CPU)"));

  Serial.print(F("flush() Time: "));
  Serial.print(flushTime);
  Serial.print(F(" μs (buffer→FIFO)"));
  Serial.println();

  Serial.print(F("Theoretical Wire Time: "));
  Serial.print(theoreticalWireTime);
  Serial.print(F(" μs ("));
  Serial.print(theoreticalWireTime / 1000.0);
  Serial.println(F(" ms)"));

  Serial.print(F("Wire Throughput: "));
  Serial.print(expectedBytes * 1000.0 / theoreticalWireTime);
  Serial.print(F(" bytes/ms ("));
  Serial.print(SERIAL_BAUD / 10);  // 8N1 = 10 bits per byte
  Serial.println(F(" bytes/sec theoretical)"));

  // Compare enqueue vs wire
  Serial.print(F("Enqueue/Wire Ratio: "));
  Serial.print((float)enqueueTime / theoreticalWireTime * 100);
  Serial.print(F("% ("));
  if (enqueueTime < theoreticalWireTime) {
    Serial.println(F("buffered, won't block)"));
  } else {
    Serial.println(F("CPU-bound)"));
  }
}

// ==================== BOARD-SPECIFIC BENCHMARKS ====================

#ifdef HAS_LED_MATRIX
void benchmarkLEDMatrix() {
  printHeader("DISPLAY: LED Matrix (Uno R4)");

  Serial.println(F("12x8 LED Matrix Available: YES"));
  Serial.println(F("Running LED animation test..."));

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

  Serial.print(F("Blink animation (20 cycles): "));
  Serial.print(blinkTime);
  Serial.println(F(" ms"));

  Serial.print(F("Pattern switching (100 frames): "));
  Serial.print(patternTime);
  Serial.print(F(" μs ("));
  Serial.print(100.0 / patternTime * 1000);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Bitmap rendering (50 frames): "));
  Serial.print(bitmapTime);
  Serial.print(F(" μs ("));
  Serial.print(50.0 / bitmapTime * 1000);
  Serial.println(F(" ops/ms)"));
}
#endif

#ifdef HAS_WIFI
void benchmarkWiFi() {
  printHeader("WIRELESS: WiFi CAPABILITIES");

  Serial.print(F("WiFi Available: YES"));

// Identify WiFi chip/library
#if defined(ARDUINO_UNOR4_WIFI)
  Serial.println(F(" (ESP32-S3 via WiFiS3)"));
#elif defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_NANO_RP2040_CONNECT)
  Serial.println(F(" (Nina W102 via WiFiNINA)"));
#elif defined(ARDUINO_SAMD_MKR1000)
  Serial.println(F(" (WINC1500 via WiFi101)"));
#elif defined(ESP32) || defined(ESP8266)
  Serial.println(F(" (Native)"));
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
  Serial.println(F(" (CYW43439)"));
#else
  Serial.println();
#endif

// MAC Address - different methods for different libraries
#if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
  // ESP and Pico W need mode set first
  WiFi.mode(WIFI_STA);
  delay(100);
  Serial.print(F("MAC Address: "));
  Serial.println(WiFi.macAddress());
#elif defined(ARDUINO_UNOR4_WIFI)
  // Uno R4 WiFi - no mode setting needed
  Serial.print(F("MAC Address: "));
  byte mac[6];
  WiFi.macAddress(mac);
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i > 0) Serial.print(":");
  }
  Serial.println();
#else
  // WiFiNINA/WiFi101 - need to get MAC as byte array
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print(F("MAC Address: "));
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i > 0) Serial.print(":");
  }
  Serial.println();
#endif

  // Scan networks
  Serial.print(F("Scanning networks... "));
  int n = WiFi.scanNetworks();
  Serial.print(n);
  Serial.println(F(" networks found"));

  if (n > 0) {
    Serial.println(F("Strongest 3 networks:"));
    for (int i = 0; i < min(3, n); i++) {
      Serial.print(F("  "));
      Serial.print(i + 1);
      Serial.print(F(": "));
      Serial.print(WiFi.SSID(i));
      Serial.print(F(" ("));
      Serial.print(WiFi.RSSI(i));
      Serial.println(F(" dBm)"));
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
  Serial.println(F("BLE Available: YES"));

#if defined(ESP32)
  Serial.println(F("Initializing BLE..."));
  BLEDevice::init("");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  Serial.print(F("Scanning for BLE devices (5 sec)... "));
  BLEScanResults* foundDevices = pBLEScan->start(5, false);
  int deviceCount = foundDevices->getCount();
  Serial.print(deviceCount);
  Serial.println(F(" devices found"));

  if (deviceCount > 0) {
    Serial.println(F("Discovered devices:"));
    for (int i = 0; i < min(5, deviceCount); i++) {
      BLEAdvertisedDevice device = foundDevices->getDevice(i);
      Serial.print(F("  "));
      Serial.print(i + 1);
      Serial.print(F(": "));
      if (device.haveName()) {
        Serial.print(device.getName().c_str());
      } else {
        Serial.print(F("Unknown"));
      }
      Serial.print(F(" ["));
      Serial.print(device.getAddress().toString().c_str());
      Serial.print(F("] RSSI: "));
      Serial.print(device.getRSSI());
      Serial.println(F(" dBm"));
    }
  }

  pBLEScan->clearResults();
  BLEDevice::deinit(false);

#elif defined(BOARD_NRF52) || defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_SAMD_NANO_33_IOT)
  // For boards using ArduinoBLE library
  Serial.println(F("BLE scan requires ArduinoBLE library"));
  Serial.println(F("Install via Library Manager: 'ArduinoBLE'"));
  Serial.println(F("Scan example:"));
  Serial.println(F("  BLE.begin() → BLE.scan() → check BLE.available()"));
#else
  Serial.println(F("BLE hardware detected but scan not implemented"));
#endif
}
#endif

void benchmarkFlash() {
  printHeader("STORAGE: Flash Information");

#if defined(ESP32)
  Serial.print(F("Flash Size: "));
  Serial.print(ESP.getFlashChipSize() / 1024);
  Serial.println(F(" KB"));

  Serial.print(F("Flash Speed: "));
  Serial.print(ESP.getFlashChipSpeed() / 1000000);
  Serial.println(F(" MHz"));

  Serial.print(F("Sketch Size: "));
  Serial.print(ESP.getSketchSize() / 1024);
  Serial.println(F(" KB"));

  Serial.print(F("Free Sketch Space: "));
  Serial.print(ESP.getFreeSketchSpace() / 1024);
  Serial.println(F(" KB"));
#elif defined(ESP8266)
  Serial.print(F("Flash Size: "));
  Serial.print(ESP.getFlashChipSize() / 1024);
  Serial.println(F(" KB"));

  Serial.print(F("Flash Speed: "));
  Serial.print(ESP.getFlashChipSpeed() / 1000000);
  Serial.println(F(" MHz"));

  Serial.print(F("Sketch Size: "));
  Serial.print(ESP.getSketchSize() / 1024);
  Serial.println(F(" KB"));

  Serial.print(F("Free Sketch Space: "));
  Serial.print(ESP.getFreeSketchSpace() / 1024);
  Serial.println(F(" KB"));
#else
  Serial.println(F("Flash info not available on this platform"));
#endif
}

// ==================== ADVANCED MATH BENCHMARKS ====================

void benchmarkAdvancedMath() {
  printHeader("ADVANCED MATH: Transcendental Functions");

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
    checksum += fmod((float)i, 7.3f);
  }
  unsigned long fmodTime = endBenchmark();

  Serial.print(F("Checksum: "));
  Serial.println(checksum);

  Serial.print(F("atan2() (100 ops): "));
  Serial.print(atan2Time);
  Serial.print(F(" μs ("));
  Serial.print(100.0 / atan2Time * 1000);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("log() (100 ops): "));
  Serial.print(logTime);
  Serial.print(F(" μs ("));
  Serial.print(100.0 / logTime * 1000);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("exp() (100 ops): "));
  Serial.print(expTime);
  Serial.print(F(" μs ("));
  Serial.print(100.0 / expTime * 1000);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("pow() (100 ops): "));
  Serial.print(powTime);
  Serial.print(F(" μs ("));
  Serial.print(100.0 / powTime * 1000);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("fmod() (1000 ops): "));
  Serial.print(fmodTime);
  Serial.print(F(" μs ("));
  Serial.print(1000.0 / fmodTime * 1000);
  Serial.println(F(" ops/ms)"));
}

// ==================== TIMING PRECISION BENCHMARKS ====================

void benchmarkTimingPrecision() {
  printHeader("TIMING: millis() and micros() Precision");

  // Test millis() accuracy
  unsigned long startMillis = millis();
  delay(1000);
  unsigned long endMillis = millis();
  long millisError = abs((long)(endMillis - startMillis) - 1000);

  Serial.print(F("millis() 1-second test: "));
  Serial.print(endMillis - startMillis);
  Serial.print(F(" ms (±"));
  Serial.print(millisError);
  Serial.println(F(" ms error)"));

  // Test micros() resolution
  unsigned long micro1 = micros();
  unsigned long micro2 = micros();
  unsigned long microResolution = micro2 - micro1;

  Serial.print(F("micros() resolution: "));
  Serial.print(microResolution);
  Serial.println(F(" μs"));

  // Test micros() consistency over short period
  unsigned long startMicros = micros();
  delayMicroseconds(1000);
  unsigned long endMicros = micros();
  long microsError = abs((long)(endMicros - startMicros) - 1000);

  Serial.print(F("micros() 1ms test: "));
  Serial.print(endMicros - startMicros);
  Serial.print(F(" μs (±"));
  Serial.print(microsError);
  Serial.println(F(" μs error)"));

// ESP32: CPU cycle counter cross-check
#ifdef ESP32
  Serial.println();
  Serial.println(F("ESP32 CPU Cycle Counter:"));

  // Get CPU frequency
  uint32_t cpuFreqMHz = ESP.getCpuFreqMHz();
  Serial.print(F("CPU Frequency: "));
  Serial.print(cpuFreqMHz);
  Serial.println(F(" MHz"));

  // Test cycle counter vs micros()
  uint32_t cycleStart = ESP.getCycleCount();
  unsigned long microsStart = micros();

  // Busy wait for ~1000 us
  delayMicroseconds(1000);

  uint32_t cycleEnd = ESP.getCycleCount();
  unsigned long microsEnd = micros();

  uint32_t cycleDelta = cycleEnd - cycleStart;
  unsigned long microsDelta = microsEnd - microsStart;

  Serial.print(F("Cycles elapsed: "));
  Serial.println(cycleDelta);
  Serial.print(F("micros() elapsed: "));
  Serial.print(microsDelta);
  Serial.println(F(" μs"));

  // Calculate expected cycles
  float expectedCycles = microsDelta * cpuFreqMHz;
  Serial.print(F("Expected cycles: "));
  Serial.println((uint32_t)expectedCycles);

  // Calculate jitter
  float jitterPercent = abs((float)cycleDelta - expectedCycles) / expectedCycles * 100.0f;
  Serial.print(F("Timing jitter: "));
  Serial.print(jitterPercent, 2);
  Serial.println(F("%"));
#endif

  // Clock frequency estimate
  Serial.println();
  Serial.print(F("Clock accuracy: "));
  float accuracy = 100.0f - ((float)millisError / 10.0f);
  Serial.print(accuracy);
  Serial.println(F("%"));
}

// ==================== STACK DEPTH BENCHMARK ====================

volatile int recursionCounter = 0;

int testRecursion(int depth) {
  recursionCounter++;
  volatile char buffer[32];  // Consume stack space
  buffer[0] = (char)depth;

  if (depth > 0) {
    return testRecursion(depth - 1) + 1;
  }
  return 1;
}

void benchmarkStackDepth() {
  printHeader("MEMORY: Stack Depth Test");

  // Test safe recursion depth
  recursionCounter = 0;
  int testDepth = 100;
  int result = testRecursion(testDepth);

  Serial.print(F("Recursion test ("));
  Serial.print(testDepth);
  Serial.print(F(" deep): "));
  if (recursionCounter == testDepth + 1) {
    Serial.println(F("PASS"));
  } else {
    Serial.println(F("FAIL"));
  }

  Serial.print(F("Each call uses ~32 bytes"));
  Serial.println();
  Serial.print(F("Test consumed ~"));
  Serial.print(testDepth * 32);
  Serial.println(F(" bytes of stack"));
}

// ==================== MULTI-CORE BENCHMARKS ====================

#if defined(ESP32) || defined(ARDUINO_ARCH_RP2040)

#ifdef ESP32
TaskHandle_t Task1;
TaskHandle_t Task2;
volatile unsigned long core0Count = 0;
volatile unsigned long core1Count = 0;
volatile bool testRunning = false;

void core0Task(void* parameter) {
  while (true) {
    if (testRunning) {
      core0Count++;
    }
    delayMicroseconds(1);
  }
}

void core1Task(void* parameter) {
  while (true) {
    if (testRunning) {
      core1Count++;
    }
    delayMicroseconds(1);
  }
}
#endif

void benchmarkMultiCore() {
  printHeader("MULTI-CORE: Parallel Performance");

#ifdef ESP32
  Serial.println(F("ESP32 Dual-Core Test"));

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

  Serial.print(F("Core 0 iterations: "));
  Serial.println(core0Count);
  Serial.print(F("Core 1 iterations: "));
  Serial.println(core1Count);
  Serial.print(F("Total iterations: "));
  Serial.println(core0Count + core1Count);
  Serial.print(F("Core balance: "));
  float balance = (float)min(core0Count, core1Count) / (float)max(core0Count, core1Count) * 100.0f;
  Serial.print(balance);
  Serial.println(F("%"));

  // Clean up
  vTaskDelete(Task1);
  vTaskDelete(Task2);

#elif defined(ARDUINO_ARCH_RP2040)
  Serial.println(F("RP2040 Dual-Core Test"));
  Serial.println(F("Single core performance:"));

  volatile unsigned long count = 0;
  unsigned long start = millis();
  while (millis() - start < 1000) {
    count++;
  }

  Serial.print(F("Iterations in 1 second: "));
  Serial.println(count);
  Serial.println(F("Note: Full dual-core requires multicore library"));
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
  
  Serial.println(F("Testing throughput at different baud rates..."));
  delay(100);
  
  for (int i = 0; i < numRates; i++) {
    Serial.end();
    delay(100);
    Serial.begin(baudRates[i]);
    delay(100);
    
    // Test throughput at this baud rate
    const char testData[] = "0123456789";
    startBenchmark();
    for (int j = 0; j < 19; j++) {
      Serial.print(testData);
    }
    Serial.flush();
    unsigned long baudTime = endBenchmark();
    
    Serial.print(F("Baud "));
    Serial.print(baudRates[i]);
    Serial.print(F(": "));
    Serial.print(baudTime);
    Serial.print(F(" μs ("));
    float bytesPerSec = 190000000.0f / baudTime;
    Serial.print(bytesPerSec, 0);
    Serial.println(F(" bytes/sec)"));
    
    delay(50);
  }
  
  // Return to standard baud
  Serial.end();
  delay(100);
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("Returned to 115200 baud"));
}
*/

// ==================== HARDWARE RNG BENCHMARK ====================

#ifdef ESP32
void benchmarkHardwareRNG() {
  printHeader("CRYPTO: Hardware RNG");

  // Test RNG speed
  startBenchmark();
  volatile uint32_t rngSum = 0;
  for (int i = 0; i < 1000; i++) {
    rngSum += esp_random();
  }
  unsigned long rngTime = endBenchmark();

  Serial.print(F("Checksum: "));
  Serial.println(rngSum);

  Serial.print(F("Hardware RNG (1000 values): "));
  Serial.print(rngTime);
  Serial.print(F(" μs ("));
  Serial.print(1000.0f / rngTime * 1000, 2);
  Serial.println(F(" values/ms)"));

  // Test randomness distribution
  uint32_t bins[4] = { 0, 0, 0, 0 };
  for (int i = 0; i < 10000; i++) {
    uint32_t val = esp_random();
    bins[val % 4]++;
  }

  Serial.println(F("Distribution (10000 samples):"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("  Bin "));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(bins[i]);
    Serial.print(F(" ("));
    Serial.print(bins[i] / 100.0f, 1);
    Serial.println(F("%)"));
  }
  Serial.println(F("(Ideal: 25% each bin)"));
}
#endif

// ==================== SYSTEM INFO ====================

void printSystemInfo() {
  printHeader("SYSTEM INFORMATION");

  Serial.print(F("Board: "));
  Serial.println(BOARD_NAME);

#if defined(ESP32)
  Serial.print(F("Chip Model: "));
  Serial.println(ESP.getChipModel());
  Serial.print(F("Chip Revision: "));
  Serial.println(ESP.getChipRevision());
  Serial.print(F("CPU Frequency: "));
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println(F(" MHz"));
  Serial.print(F("Cores: "));
  Serial.println(ESP.getChipCores());
  Serial.print(F("SDK Version: "));
  Serial.println(ESP.getSdkVersion());
#elif defined(ESP8266)
  Serial.print(F("CPU Frequency: "));
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println(F(" MHz"));
  Serial.print(F("Chip ID: "));
  Serial.println(ESP.getChipId());
  Serial.print(F("SDK Version: "));
  Serial.println(ESP.getSdkVersion());
#elif defined(ARDUINO_ARCH_RP2040)
  Serial.print(F("CPU Frequency: "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));
#if defined(ARDUINO_NANO_RP2040_CONNECT)
  Serial.println(F("Features: WiFi (Nina W102), BLE, IMU (LSM6DSOX), Mic"));
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
  Serial.println(F("Features: WiFi (CYW43439)"));
#endif
#elif defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  Serial.print(F("MCU: Renesas RA4M1 (ARM Cortex-M4)"));
  Serial.println();
  Serial.print(F("CPU Frequency: "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));
#ifdef ARDUINO_UNOR4_WIFI
  Serial.println(F("Features: WiFi (ESP32-S3), 12x8 LED Matrix"));
#else
  Serial.println(F("Features: 12x8 LED Matrix"));
#endif
#elif defined(BOARD_SAMD)
  Serial.print(F("MCU: SAMD (ARM Cortex-M0+)"));
  Serial.println();
  Serial.print(F("CPU Frequency: "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));
#if defined(ARDUINO_SAMD_NANO_33_IOT)
  Serial.println(F("Features: WiFi, BLE, IMU (LSM6DS3)"));
#elif defined(ARDUINO_SAMD_MKRWIFI1010)
  Serial.println(F("Features: WiFi (Nina W102), Crypto (ECC508)"));
#endif
#elif defined(BOARD_NRF52)
  Serial.print(F("MCU: nRF52840 (ARM Cortex-M4F)"));
  Serial.println();
  Serial.print(F("CPU Frequency: 64 MHz"));
  Serial.println();
  Serial.println(F("Features: BLE 5.0, IMU (LSM9DS1)"));
#elif defined(BOARD_STM32H7)
  Serial.print(F("MCU: STM32H7 (ARM Cortex-M7)"));
  Serial.println();
  Serial.print(F("CPU Frequency: "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));
#ifdef HAS_DUAL_CORE
  Serial.println(F("Cores: Dual Core (M7 + M4)"));
#endif
#elif defined(BOARD_TEENSY)
  Serial.print(F("CPU Frequency: "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));
#if defined(__IMXRT1062__)
  Serial.println(F("MCU: i.MX RT1062 (ARM Cortex-M7)"));
#endif
#else
  Serial.print(F("CPU Frequency: "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));
#endif

  // RAM Info
  Serial.print(F("Free RAM: "));
#if defined(ESP32)
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(F(" KB"));
  Serial.print(F("Total Heap: "));
  Serial.print(ESP.getHeapSize() / 1024);
  Serial.println(F(" KB"));
  Serial.print(F("Min Free Heap: "));
  Serial.print(ESP.getMinFreeHeap() / 1024);
  Serial.println(F(" KB"));
#elif defined(ESP8266)
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(F(" KB"));
#elif defined(__AVR__)
  extern int __heap_start, *__brkval;
  int v;
  int freeRam = (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
  Serial.print(freeRam);
  Serial.println(F(" bytes"));
// Show total RAM for common AVR boards
#if defined(__AVR_ATmega328P__)
  Serial.println(F("Total RAM: 2 KB"));
#elif defined(__AVR_ATmega2560__)
  Serial.println(F("Total RAM: 8 KB"));
#elif defined(__AVR_ATmega32U4__)
  Serial.println(F("Total RAM: 2.5 KB"));
#endif
#elif defined(ARDUINO_SAM_DUE)
  extern char _end;
  extern "C" char *sbrk(int i);
  char *ramstart = (char *)0x20070000;
  char *ramend = (char *)0x20088000;
  int freeRam = ramend - sbrk(0);
  Serial.print(freeRam / 1024);
  Serial.println(F(" KB"));
  Serial.println(F("Total RAM: 96 KB"));
#elif defined(ARDUINO_ARCH_RP2040)
  // RP2040 has 264KB RAM
  Serial.println(F("~264 KB (RP2040)"));
#elif defined(BOARD_NRF52)
  Serial.println(F("~256 KB (nRF52840)"));
#elif defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  Serial.println(F("32 KB (RA4M1)"));
#else
  Serial.println(F("Unknown"));
#endif

  Serial.print(F("Compile Date: "));
  Serial.print(__DATE__);
  Serial.print(F(" "));
  Serial.println(__TIME__);
}

// ==================== MAIN FUNCTIONS ====================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(2000);  // Wait for serial connection

  Serial.println();
  Serial.println();
  printDivider();
  Serial.println(F("  UNIVERSAL ARDUINO BENCHMARK SUITE"));
  printDivider();
  Serial.println();

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
#if defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
  benchmarkMultiCore();
#endif
#if defined(ESP32)
  benchmarkHardwareRNG();
#endif

  // Final summary
  printHeader("BENCHMARK COMPLETE!");
  Serial.println(F("Results saved in Serial Monitor."));
  Serial.println(F("You can copy/paste the output for analysis."));
  Serial.println();
  Serial.println(F("To run again, press the RESET button or"));
  Serial.println(F("re-upload the sketch."));
  printDivider();
}

void loop() {
  // Nothing to do - benchmark runs once in setup()
  delay(1000);
}
