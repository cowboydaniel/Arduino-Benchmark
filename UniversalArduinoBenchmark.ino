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

#if defined(ARDUINO_ARCH_RP2040)
#include <Hash.h>
#endif

#include "BenchmarkHelpers.h"

// ==================== GLOBAL VARIABLES ====================
uint8_t testBuffer[256];
unsigned long gMinBenchUs = 20000;
const uint8_t kJitterTrials = 5;

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
      result = result * 1.0001f + sqrtf((float)i);
      result = fmodf(result, twoPi);
      result = sinf(result) + cosf(result);
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
      Serial.write('.');
      lastIo = millis();
    }
#endif
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

  const uint32_t minDurationMs = 5;

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
  TimedLoopResult mulResult = runTimedLoop(minDurationMs, 100, [&]() {
    for (uint32_t i = 1; i <= 100; i++) {
      acc = (acc * (i | 1)) & 0xFFFFFFFF;  // Ensure odd multiplier, prevent overflow
    }
  });
  Serial.print(F("Checksum: "));
  Serial.println((uint32_t)acc);

  // Division - vary both dividend and divisor
  acc = 0xFFFFFFFFULL;
  TimedLoopResult divResult = runTimedLoop(minDurationMs, 100, [&]() {
    for (uint32_t i = 1; i <= 100; i++) {
      uint32_t divisor = (i % 127) + 2;  // 2-128, avoid div-by-1
      acc = (acc / divisor) + i;         // Accumulate to prevent optimization
    }
  });
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
  Serial.print(mulResult.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(mulResult.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(mulResult.opsPerMs);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Division ("));
  Serial.print(divResult.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(divResult.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(divResult.opsPerMs);
  Serial.println(F(" ops/ms)"));
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
  TimedLoopResult sqrtResult = runTimedLoop(minDurationMs, 100, [&]() {
    for (uint32_t i = 0; i < 100; i++) {
      fresult += sqrt((float)i);
    }
  });
  Serial.print(F("Checksum: "));
  Serial.println(fresult);

  // Sin/Cos - accumulate to prevent optimization
  fresult = 0.0f;
  TimedLoopResult trigResult = runTimedLoop(minDurationMs, 100, [&]() {
    for (uint32_t i = 0; i < 100; i++) {
      fresult += sin((float)i / 100.0f) + cos((float)i / 100.0f);
    }
  });
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
  Serial.print(sqrtResult.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(sqrtResult.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(sqrtResult.opsPerMs);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Sin/Cos ("));
  Serial.print(trigResult.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(trigResult.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(trigResult.opsPerMs);
  Serial.println(F(" ops/ms)"));
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

  Serial.print(F("Arduino String (heap stress) - Concatenation ("));
  Serial.print(concatResult.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(concatResult.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(concatResult.opsPerMs);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Arduino String (heap stress) - Comparison ("));
  Serial.print(cmpResultData.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(cmpResultData.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(cmpResultData.opsPerMs);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Arduino String (heap stress) - Int to String ("));
  Serial.print(toStrResult.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(toStrResult.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(toStrResult.opsPerMs);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("snprintf (fixed buffer, "));
  Serial.print(snprintfResult.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(snprintfResult.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(snprintfResult.opsPerMs);
  Serial.println(F(" ops/ms)"));
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
  Serial.print(F("Read checksum: "));
  Serial.println((uint32_t)checksum);

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
  Serial.print(F("Random checksum: "));
  Serial.println((uint32_t)checksum);

  Serial.print(F("Sequential Write ("));
  Serial.print(writeResult.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(writeResult.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(writeResult.opsPerMs);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Sequential Read ("));
  Serial.print(readResult.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(readResult.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(readResult.opsPerMs);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("Random Access ("));
  Serial.print(randomTotalOps);
  Serial.print(F(" ops): "));
  Serial.print(randomElapsedMicros);
  Serial.print(F(" μs (median "));
  Serial.print(randomOpsPerMs);
  Serial.print(F(" ops/ms, "));
  Serial.print(kJitterTrials);
  Serial.println(F(" trials)"));
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
  testPin = 13;  // Classic Arduino default
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
  volatile uint8_t* out = portOutputRegister(digitalPinToPort(testPin));
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
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
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

  Serial.print(F("digitalWrite() ("));
  Serial.print(dwOpsPerTrial);
  Serial.print(F(" ops): "));
  Serial.print(writeTime);
  Serial.print(F(" μs (median "));
  Serial.print(writeOpsPerMs);
  Serial.print(F(" ops/ms, "));
  Serial.print(kJitterTrials);
  Serial.println(F(" trials)"));

#ifdef __AVR__
  Serial.print(F("Direct Port ("));
  Serial.print(portTotalOps);
  Serial.print(F(" ops): "));
  Serial.print(portElapsedMicros);
  Serial.print(F(" μs (median "));
  Serial.print(portOpsPerMs);
  Serial.print(F(" ops/ms, "));
  Serial.print(kJitterTrials);
  Serial.println(F(" trials)"));
  Serial.print(F("Speedup: "));
  Serial.print(portOpsPerMs / writeOpsPerMs);
  Serial.println(F("x faster"));
#endif

#if defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
  Serial.print(F("Direct Register ("));
  Serial.print(regTotalOps);
  Serial.print(F(" ops): "));
  Serial.print(regElapsedMicros);
  Serial.print(F(" μs (median "));
  Serial.print(regOpsPerMs);
  Serial.print(F(" ops/ms, "));
  Serial.print(kJitterTrials);
  Serial.println(F(" trials)"));
  Serial.print(F("Speedup: "));
  Serial.print(regOpsPerMs / writeOpsPerMs);
  Serial.println(F("x faster"));
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
  int analogInPin = 1;    // ADC1 channel
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  int analogInPin = 1;  // ADC1 channel
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  int analogInPin = 0;  // ADC1 channel
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
  int analogInPin = 0;  // ADC1 channel
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
  int analogInPin = 0;  // ADC1 channel
#else
  int analogInPin = 36;  // VP (ESP32 ADC1)
#endif
#if defined(CONFIG_IDF_TARGET_ESP32)
  int analogOutPin = 25;  // DAC1
#else
  int analogOutPin = -1;  // No DAC on these variants
#endif
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
    bool allZero = true;
    TimedLoopResult readResult = runTimedLoop(minDurationMs, 1, [&]() {
      int value = analogRead(analogInPin);
      sum += value;
      if (value != 0) {
        allZero = false;
      }
    });

    Serial.print(F("analogRead() ("));
    Serial.print(readResult.totalOps);
    Serial.print(F(" ops): "));
    Serial.print(readResult.elapsedMicros);
    Serial.print(F(" μs ("));
    Serial.print(readResult.opsPerMs);
    Serial.println(F(" ops/ms)"));
    Serial.print(F("ADC average: "));
    Serial.println((uint32_t)(sum / readResult.totalOps));
    if (allZero) {
      Serial.println(F("Warning: ADC reads were all zero; ADC pin may be invalid or floating."));
    }
  }

  // analogWrite/PWM benchmark
  if (analogOutPin >= 0) {
#if defined(ESP32)
    const int pwmFreq = 5000;
    const int pwmResolution = 8;

    startBenchmark();
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    int pwmChannel = ledcAttach(analogOutPin, pwmFreq, pwmResolution);
#else
    const int pwmChannel = 0;
    ledcSetup(pwmChannel, pwmFreq, pwmResolution);
    ledcAttachPin(analogOutPin, pwmChannel);
#endif
    unsigned long setupTime = endBenchmark();

    uint32_t pwmValue = 0;
    TimedLoopResult updateResult = runTimedLoop(minDurationMs, 1, [&]() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
      ledcWrite(analogOutPin, pwmValue % 256);
#else
        ledcWrite(pwmChannel, pwmValue % 256);
#endif
      pwmValue++;
    });

    Serial.print(F("PWM setup: "));
    Serial.print(setupTime);
    Serial.print(F(" μs ("));
    Serial.print(1000.0 / setupTime);
    Serial.println(F(" ops/ms)"));

    Serial.print(F("PWM duty update ("));
    Serial.print(updateResult.totalOps);
    Serial.print(F(" ops): "));
    Serial.print(updateResult.elapsedMicros);
    Serial.print(F(" μs ("));
    Serial.print(updateResult.opsPerMs);
    Serial.println(F(" ops/ms)"));
#else
    pinMode(analogOutPin, OUTPUT);
    uint32_t pwmValue = 0;
    TimedLoopResult writeResult = runTimedLoop(minDurationMs, 1, [&]() {
      analogWrite(analogOutPin, pwmValue % 256);
      pwmValue++;
    });

    Serial.print(F("analogWrite() ("));
    Serial.print(writeResult.totalOps);
    Serial.print(F(" ops): "));
    Serial.print(writeResult.elapsedMicros);
    Serial.print(F(" μs ("));
    Serial.print(writeResult.opsPerMs);
    Serial.println(F(" ops/ms)"));
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
    Serial.flush();
    startBenchmark();
    for (int i = 0; i < 100; i++) {
      Serial.print(i);
    }
    unsigned long enqueueTime = endBenchmark();
    enqueueTimeMedian.add(enqueueTime);
    enqueueRateMedian.add(expectedBytes * 1000.0f / enqueueTime);
  }
  unsigned long enqueueTime = enqueueTimeMedian.median();
  float enqueueRate = enqueueRateMedian.median();

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
  Serial.print(F(" μs (median "));
  Serial.print(enqueueRate);
  Serial.print(F(" bytes/ms CPU, "));
  Serial.print(kJitterTrials);
  Serial.println(F(" trials)"));

  Serial.print(F("flush() time (implementation-dependent): "));
  Serial.print(flushTime);
  Serial.print(F(" μs"));
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

#if defined(ARDUINO_ARCH_RP2040)
void benchmarkSHA1() {
  printHeader("CRYPTO: SHA1");

  Serial.println(F("Example digests:"));
  Serial.print(F("SHA1:"));
  Serial.println(sha1("abc"));

  uint8_t hash[20];
  sha1("test", &hash[0]);
  Serial.print(F("SHA1:"));
  for (uint16_t i = 0; i < 20; i++) {
    Serial.printf("%02x", hash[i]);
  }
  Serial.println();

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

  Serial.print(F("Checksum: "));
  Serial.println(checksum);

  Serial.print(F("SHA1 String ("));
  Serial.print(stringResult.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(stringResult.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(stringResult.opsPerMs);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("SHA1 Buffer ("));
  Serial.print(bufferResult.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(bufferResult.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(bufferResult.opsPerMs);
  Serial.println(F(" ops/ms)"));
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

  const uint8_t inputSize = 128;
  uint8_t input[inputSize];
  for (uint8_t i = 0; i < inputSize; i++) {
    input[i] = (uint8_t)(i * 3 + 7);
  }

  uint8_t digest[64];
  char hexDigest[129];
  volatile uint32_t checksum = 0;
  uint32_t lastProgressMs = millis();
  const uint32_t maxCryptoMs = 15000;
  const uint32_t cryptoStartMs = lastProgressMs;
  bool cryptoTimedOut = false;
  auto benchYield = []() {
#if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_RP2040)
    yield();
#endif
  };
  auto checkCryptoTimeout = [&]() -> bool {
    if (!cryptoTimedOut && (millis() - cryptoStartMs > maxCryptoMs)) {
      Serial.println(F("Warning: crypto benchmark timed out."));
      cryptoTimedOut = true;
    }
    return cryptoTimedOut;
  };

  auto toHex = [&](const uint8_t *data, size_t len, char *out) {
    static const char *kHex = "0123456789abcdef";
    for (size_t i = 0; i < len; i++) {
      out[i * 2] = kHex[data[i] >> 4];
      out[i * 2 + 1] = kHex[data[i] & 0x0F];
    }
    out[len * 2] = '\0';
  };

  const uint32_t minDurationMs = max(5UL, (gMinBenchUs + 999UL) / 1000UL);

  Serial.println(F("Example digests:"));
  benchmarkMd5(input, inputSize, digest);
  toHex(digest, 16, hexDigest);
  Serial.print(F("MD5: "));
  Serial.println(hexDigest);

  benchmarkSha1(input, inputSize, digest);
  toHex(digest, 20, hexDigest);
  Serial.print(F("SHA1: "));
  Serial.println(hexDigest);

  benchmarkSha256(input, inputSize, digest);
  toHex(digest, 32, hexDigest);
  Serial.print(F("SHA256: "));
  Serial.println(hexDigest);

  benchmarkSha512(input, inputSize, digest);
  toHex(digest, 64, hexDigest);
  Serial.print(F("SHA512: "));
  Serial.println(hexDigest);

#if defined(MBEDTLS_SHA3_C)
  {
    mbedtls_sha3_context sha3;
    mbedtls_sha3_init(&sha3);
    mbedtls_sha3_starts(&sha3, 256);
    mbedtls_sha3_update(&sha3, input, inputSize);
    mbedtls_sha3_finish(&sha3, digest);
    mbedtls_sha3_free(&sha3);
    toHex(digest, 32, hexDigest);
    Serial.print(F("SHA3-256: "));
    Serial.println(hexDigest);
  }
#else
  Serial.println(F("SHA3-256: not available in this build"));
#endif

  Serial.println(F("...running HEX encode"));
  TimedLoopResult hexResult = runTimedLoop(minDurationMs, 20, [&]() {
    for (uint8_t i = 0; i < 20; i++) {
      if (checkCryptoTimeout()) {
        return;
      }
      toHex(input, inputSize, hexDigest);
      checksum += hexDigest[0];
      if ((i % 5) == 0) {
        benchYield();
      }
    }
  });
  Serial.println(F("...running MD5"));
  TimedLoopResult md5Result = runTimedLoop(minDurationMs, 10, [&]() {
    for (uint8_t i = 0; i < 10; i++) {
      if (checkCryptoTimeout()) {
        return;
      }
      benchmarkMd5(input, inputSize, digest);
      checksum += digest[0];
      if ((i % 3) == 0) {
        benchYield();
      }
    }
  });
  Serial.println(F("...running SHA1"));
  TimedLoopResult sha1Result = runTimedLoop(minDurationMs, 10, [&]() {
    for (uint8_t i = 0; i < 10; i++) {
      if (checkCryptoTimeout()) {
        return;
      }
      benchmarkSha1(input, inputSize, digest);
      checksum += digest[0];
      if ((i % 3) == 0) {
        benchYield();
      }
    }
  });
  Serial.println(F("...running SHA256"));
  TimedLoopResult sha256Result = runTimedLoop(minDurationMs, 10, [&]() {
    for (uint8_t i = 0; i < 10; i++) {
      if (checkCryptoTimeout()) {
        return;
      }
      benchmarkSha256(input, inputSize, digest);
      checksum += digest[0];
      if ((i % 3) == 0) {
        benchYield();
      }
    }
  });
  Serial.println(F("...running SHA512"));
  TimedLoopResult sha512Result = runTimedLoop(minDurationMs, 10, [&]() {
    for (uint8_t i = 0; i < 10; i++) {
      if (checkCryptoTimeout()) {
        return;
      }
      benchmarkSha512(input, inputSize, digest);
      checksum += digest[0];
      if ((i % 2) == 0) {
        benchYield();
      }
    }
  });
#if defined(MBEDTLS_SHA3_C)
  Serial.println(F("...running SHA3-256"));
  TimedLoopResult sha3Result = runTimedLoop(minDurationMs, 5, [&]() {
    for (uint8_t i = 0; i < 5; i++) {
      if (checkCryptoTimeout()) {
        return;
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
  });
#endif

  const uint8_t saltSize = 16;
  uint8_t salt[saltSize];
  for (uint8_t i = 0; i < saltSize; i++) {
    salt[i] = (uint8_t)(0xA5 ^ i);
  }
  const char *password = "esp32-benchmark";
  const uint32_t pbkdf2Iterations = 500;

  Serial.println(F("...running PBKDF2-HMAC-SHA256"));
  TimedLoopResult pbkdf2Result = runTimedLoop(minDurationMs, 1, [&]() {
    if (checkCryptoTimeout()) {
      return;
    }
#if defined(MBEDTLS_VERSION_NUMBER) && MBEDTLS_VERSION_NUMBER >= 0x03000000
    if (mbedtls_pkcs5_pbkdf2_hmac_ext(MBEDTLS_MD_SHA256,
                                     reinterpret_cast<const unsigned char *>(password),
                                     strlen(password),
                                     salt,
                                     saltSize,
                                     pbkdf2Iterations,
                                     32,
                                     digest) == 0) {
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
                                   digest) == 0) {
        checksum += digest[0];
      }
    }
    mbedtls_md_free(&ctx);
#endif
    if (millis() - lastProgressMs > 1000) {
      Serial.print('.');
      lastProgressMs = millis();
    }
    benchYield();
  });
  Serial.print(F("Checksum: "));
  Serial.println(checksum);

  Serial.print(F("HEX encode ("));
  Serial.print(hexResult.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(hexResult.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(hexResult.opsPerMs);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("MD5 ("));
  Serial.print(md5Result.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(md5Result.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(md5Result.opsPerMs);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("SHA1 ("));
  Serial.print(sha1Result.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(sha1Result.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(sha1Result.opsPerMs);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("SHA256 ("));
  Serial.print(sha256Result.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(sha256Result.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(sha256Result.opsPerMs);
  Serial.println(F(" ops/ms)"));

  Serial.print(F("SHA512 ("));
  Serial.print(sha512Result.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(sha512Result.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(sha512Result.opsPerMs);
  Serial.println(F(" ops/ms)"));

#if defined(MBEDTLS_SHA3_C)
  Serial.print(F("SHA3-256 ("));
  Serial.print(sha3Result.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(sha3Result.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(sha3Result.opsPerMs);
  Serial.println(F(" ops/ms)"));
#endif

  Serial.print(F("PBKDF2-HMAC-SHA256 ("));
  Serial.print(pbkdf2Result.totalOps);
  Serial.print(F(" ops): "));
  Serial.print(pbkdf2Result.elapsedMicros);
  Serial.print(F(" μs ("));
  Serial.print(pbkdf2Result.opsPerMs);
  Serial.println(F(" ops/ms)"));
  Serial.print(F("  Iterations per op: "));
  Serial.println(pbkdf2Iterations);
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

#if defined(__AVR__)
  Serial.println(F("Stack depth test skipped on low-SRAM AVR targets."));
  Serial.println(F("Reason: recursion can exhaust limited SRAM quickly."));
  return;
#endif

  // Test safe recursion depth
  recursionCounter = 0;
  int testDepth = 100;
  int result = testRecursion(testDepth);
  (void)result;

  Serial.print(F("Recursion test ("));
  Serial.print(testDepth);
  Serial.print(F(" deep): "));
  if (recursionCounter == testDepth + 1) {
    Serial.println(F("PASS"));
  } else {
    Serial.println(F("FAIL"));
  }
  Serial.println(F("Per-call stack usage not measured on this platform."));
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
  volatile uint32_t accumulator = 0;
  while (true) {
    if (testRunning) {
      core0Count++;
      accumulator += 3;
      accumulator ^= (accumulator << 1);
    }
  }
}

void core1Task(void* parameter) {
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
#if defined(HAS_PICO_MULTICORE)
  Serial.println(F("Running dual-core workload for 1 second..."));

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

  Serial.print(F("Core 0 iterations: "));
  Serial.println(rp2040Core0Count);
  Serial.print(F("Core 1 iterations: "));
  Serial.println(rp2040Core1Count);
  Serial.print(F("Total iterations: "));
  Serial.println(total);
  Serial.print(F("Scaling efficiency: "));
  Serial.print(scaling, 2);
  Serial.println(F("x"));
#else
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

// ==================== SOFTWARE ATSE BENCHMARK ====================

// Arduino Uno R4 WiFi has an ESP32-S3 WiFi module with secure element capabilities
// SoftwareATSE library is included with the WiFiS3 library on Uno R4 WiFi
#if defined(ARDUINO_UNOR4_WIFI)

#include <SoftwareATSE.h>

constexpr int kAtseKeyId = 999; // Reserved for benchmark use to avoid clobbering other keys.

void benchmarkSoftwareATSE() {
  printHeader("CRYPTO: SoftwareATSE (Uno R4 WiFi)");
  
  Serial.println(F("Initializing SoftwareATSE..."));
  
  // Initialize SoftwareATSE
  if (!SATSE.begin()) {
    Serial.println(F("SoftwareATSE initialization failed"));
    Serial.println(F("The WiFi module may not be responding or"));
    Serial.println(F("secure element features may not be available."));
    return;
  }
  
  Serial.println(F("SoftwareATSE initialized successfully!"));
  Serial.println();
  
  // ===== Public Key Generation Benchmark =====
  Serial.println(F("--- Public Key Generation ---"));
  
  bool keyGenSupported = false;
  const uint16_t keySlot = kAtseKeyId;
  unsigned long keyGenStart = millis();
  byte publicKey[64];  // Buffer for public key
  byte privateKey[32]; // Buffer for private key
  
  if (SATSE.generatePrivateKey(keySlot, privateKey) == 1) {
    // Try to generate public key in the same slot
    if (SATSE.generatePublicKey(kAtseKeyId, publicKey) == 1) {
      unsigned long keyGenTime = millis() - keyGenStart;
      keyGenSupported = true;
      
      Serial.print(F("SoftwareATSE: Public key generation (ms): "));
      Serial.println(keyGenTime);
      
      Serial.print(F("  Rate: "));
      if (keyGenTime > 0) {
        Serial.print(1000.0f / keyGenTime, 3);
        Serial.println(F(" keys/sec"));
      } else {
        Serial.println(F("N/A (too fast)"));
      }
    } else {
      Serial.println(F("SoftwareATSE: Public key generation - not supported"));
    }
  } else {
    Serial.println(F("SoftwareATSE: Private key generation failed; skipping public key test"));
  }
  
  Serial.println();
  
  // ===== Configuration Write Benchmark =====
  Serial.println(F("--- Secure Storage Operations ---"));
  
  uint8_t testConfig[256];
  for (int i = 0; i < 256; i++) {
    testConfig[i] = i & 0xFF;
  }
  
  unsigned long writeStart = micros();
  bool writeSuccess = false;
  
  if (SATSE.writeConfiguration(testConfig) == 1) {
    unsigned long writeTime = micros() - writeStart;
    writeSuccess = true;
    
    Serial.print(F("Config write (\xC2\xB5s): "));
    Serial.println(writeTime);
    Serial.print(F("  (ms): "));
    Serial.println(writeTime / 1000.0f, 3);
    
    Serial.print(F("  Write speed: "));
    if (writeTime > 0) {
      Serial.print(256.0f * 1000000.0f / writeTime, 2);
      Serial.println(F(" bytes/sec"));
    } else {
      Serial.println(F("N/A (too fast)"));
    }
  } else {
    Serial.println(F("Config write - not supported"));
  }
  
  Serial.println();
  
  // ===== Signing Benchmark =====
  Serial.println(F("--- EC Signing ---"));
  
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
      Serial.print(F("EC signing (avg ms/op): "));
      Serial.println(avgSignTimeMs, 3);
      
      Serial.print(F("  Rate: "));
      Serial.print(1000.0f / avgSignTimeMs, 2);
      Serial.println(F(" ops/sec"));
      
      Serial.print(F("  Successful operations: "));
      Serial.print(successfulSigns);
      Serial.print(F("/"));
      Serial.println(NUM_SIGN_OPS);
    } else {
      Serial.println(F("EC signing - not supported"));
    }
  } else {
    Serial.println(F("Signing tests skipped (key generation not supported)"));
  }
  
  Serial.println();
  
  // ===== Secure RNG Benchmark =====
  Serial.println(F("--- Secure Random Number Generator ---"));
  
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
    
    Serial.print(F("RNG checksum: "));
    Serial.println(rngChecksum);
    
    if (successfulReads > 0) {
      unsigned long totalBytes = successfulReads * RNG_BUFFER_SIZE;
      float bytesPerSec = (totalBytes * 1000000.0f) / rngTime;
      
      Serial.print(F("Secure RNG throughput: "));
      Serial.print(bytesPerSec, 2);
      Serial.println(F(" bytes/sec"));
      
      Serial.print(F("  Total bytes generated: "));
      Serial.println(totalBytes);
      
      Serial.print(F("  Time: "));
      Serial.print(rngTime / 1000.0f, 2);
      Serial.println(F(" ms"));
      
      Serial.print(F("  Successful reads: "));
      Serial.print(successfulReads);
      Serial.print(F("/"));
      Serial.println(RNG_ITERATIONS);
      
      // Distribution test
      Serial.println(F("Distribution test (first 1000 bytes):"));
      uint32_t byteBins[4] = {0, 0, 0, 0};
      
      for (int i = 0; i < 1000; i++) {
        uint8_t randomByte;
        if (SATSE.random(&randomByte, 1)) {
          byteBins[randomByte % 4]++;
        }
      }
      
      for (int i = 0; i < 4; i++) {
        Serial.print(F("  Bin "));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(byteBins[i]);
        Serial.print(F(" ("));
        Serial.print(byteBins[i] / 10.0f, 1);
        Serial.println(F("%)"));
      }
      Serial.println(F("  (Ideal: 25% each bin)"));
    } else {
      Serial.println(F("Secure RNG - read operations failed"));
    }
  } else {
    Serial.println(F("Secure RNG - not supported"));
  }
  
  Serial.println();
  Serial.println(F("SoftwareATSE benchmarks complete"));
}

#else  // !ARDUINO_UNOR4_WIFI

// Stub function for non-Uno R4 WiFi boards
void benchmarkSoftwareATSE() {
  // This function intentionally left empty for non-R4 boards
  // It won't be called due to compile-time guards in setup()
}

#endif  // ARDUINO_UNOR4_WIFI

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

  calibrateBenchmarkTime();

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
#if defined(ARDUINO_UNOR4_WIFI)
  benchmarkSoftwareATSE();
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
