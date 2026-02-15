/*
 * Standalone I2C Wire benchmark test extracted from Part2_PlatformBenchmarks
 * Runs only the I2C benchmark section.
 */

#include <Arduino.h>
#include <Wire.h>

#ifndef F_STR
#define F_STR(x) F(x)
#endif

#define SERIAL_OUT Serial

static unsigned long benchmarkStartMicros = 0;

static inline void startBenchmark() {
  benchmarkStartMicros = micros();
}

static inline unsigned long endBenchmark() {
  return micros() - benchmarkStartMicros;
}

static void printHeader(const char* title) {
  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("========================================"));
  SERIAL_OUT.println(title);
  SERIAL_OUT.println(F_STR("========================================"));
}

static void runI2CWriteBenchmark(const __FlashStringHelper* label, uint32_t clockHz, int iterations, int progressStep) {
  uint16_t statusCounts[6] = {0, 0, 0, 0, 0, 0};

  Wire.setClock(clockHz);
  startBenchmark();

  int completed = 0;
  for (int i = 0; i < iterations; i++) {
    if (progressStep > 0 && i > 0 && (i % progressStep) == 0) {
      SERIAL_OUT.print(F_STR("."));
    }

    Wire.beginTransmission(0x08);
    Wire.write(i & 0xFF);
    uint8_t status = Wire.endTransmission();

    if (status < 6) {
      statusCounts[status]++;
    } else {
      statusCounts[4]++;
    }

    completed++;

    // Abort early if the bus repeatedly reports hard failures/timeouts.
    if ((statusCounts[4] + statusCounts[5]) >= 10) {
      break;
    }
  }

  unsigned long elapsed = endBenchmark();

  if (progressStep > 0) {
    SERIAL_OUT.println();
  }

  SERIAL_OUT.print(label);
  SERIAL_OUT.print(F_STR(": "));
  SERIAL_OUT.print(elapsed);
  SERIAL_OUT.print(F_STR(" us for "));
  SERIAL_OUT.print(completed);
  SERIAL_OUT.print(F_STR(" writes ("));
  SERIAL_OUT.print((completed > 0 && elapsed > 0) ? (completed * 1000.0f / elapsed) : 0.0f);
  SERIAL_OUT.println(F_STR(" txn/ms)"));

  SERIAL_OUT.print(F_STR("  Status codes: ok="));
  SERIAL_OUT.print(statusCounts[0]);
  SERIAL_OUT.print(F_STR(", addrNACK="));
  SERIAL_OUT.print(statusCounts[2]);
  SERIAL_OUT.print(F_STR(", dataNACK="));
  SERIAL_OUT.print(statusCounts[3]);
  SERIAL_OUT.print(F_STR(", other="));
  SERIAL_OUT.print(statusCounts[4]);
  SERIAL_OUT.print(F_STR(", timeout="));
  SERIAL_OUT.println(statusCounts[5]);

  if (completed < iterations) {
    SERIAL_OUT.println(F_STR("  Early stop: repeated bus errors/timeouts"));
  }
}

static void benchmarkWireI2C() {
  printHeader("I/O: WIRE I2C COMMUNICATION");

  // Number of I2C buses
  SERIAL_OUT.print(F_STR("I2C buses: "));
#if defined(WIRE_INTERFACES_COUNT)
  SERIAL_OUT.println(WIRE_INTERFACES_COUNT);
#elif defined(ESP32)
  SERIAL_OUT.println(2);
#elif defined(ARDUINO_ARCH_RP2040)
  SERIAL_OUT.println(2);
#elif defined(ARDUINO_SAM_DUE)
  SERIAL_OUT.println(2);
#elif defined(BOARD_STM32H7)
  SERIAL_OUT.println(3);
#elif defined(BOARD_NRF52)
  SERIAL_OUT.println(2);
#else
  SERIAL_OUT.println(1);
#endif

  // Internal buffer size
  SERIAL_OUT.print(F_STR("Buffer size: "));
#if defined(BUFFER_LENGTH)
  SERIAL_OUT.print(BUFFER_LENGTH);
  SERIAL_OUT.println(F_STR(" bytes"));
#elif defined(I2C_BUFFER_LENGTH)
  SERIAL_OUT.print(I2C_BUFFER_LENGTH);
  SERIAL_OUT.println(F_STR(" bytes"));
#elif defined(WIRE_BUFFER_LENGTH)
  SERIAL_OUT.print(WIRE_BUFFER_LENGTH);
  SERIAL_OUT.println(F_STR(" bytes"));
#else
  SERIAL_OUT.println(F_STR("unknown"));
#endif

  Wire.begin();

#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);  // 3ms per transaction to avoid long stalls on stuck bus
  SERIAL_OUT.println(F_STR("I2C timeout guard: enabled (3 ms)"));
#endif

  SERIAL_OUT.println();
  SERIAL_OUT.println(F_STR("Testing I2C transaction speed"));
  SERIAL_OUT.println(F_STR("(No device - measuring bus overhead)"));
  SERIAL_OUT.println();

  const int i2cIterations = 1000;
  const int i2cProgressStep = 0;

  runI2CWriteBenchmark(F_STR("100 kHz"), 100000, i2cIterations, i2cProgressStep);
  runI2CWriteBenchmark(F_STR("400 kHz"), 400000, i2cIterations, i2cProgressStep);

#if defined(ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(BOARD_STM32H7)
  runI2CWriteBenchmark(F_STR("1 MHz"), 1000000, i2cIterations, i2cProgressStep);
#endif

#if defined(ESP32) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32C6) && !defined(CONFIG_IDF_TARGET_ESP32H2)
  Wire.end();
#endif
}

void setup() {
  SERIAL_OUT.begin(115200);

#if defined(USBCON)
  unsigned long waitStart = millis();
  while (!SERIAL_OUT && millis() - waitStart < 3000) {
    delay(10);
  }
#endif

  delay(300);
  benchmarkWireI2C();
}

void loop() {
  // Intentionally empty: run once.
}
