#ifndef BENCHMARK_HELPERS_H
#define BENCHMARK_HELPERS_H

#include <Arduino.h>

struct MinDurationResult {
  uint32_t ops;
  unsigned long elapsedUs;
};

template <typename Func>
MinDurationResult runForAtLeastUs(unsigned long minUs, Func fn) {
  MinDurationResult result = {};
  unsigned long start = micros();
  unsigned long elapsed = 0;
  do {
    result.ops += fn();
#if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_RP2040)
    yield();
#endif
    elapsed = micros() - start;
  } while (elapsed < minUs);
  result.elapsedUs = elapsed;
  return result;
}

struct TimedLoopResult {
  unsigned long elapsedMicros;
  uint32_t iterations;
  uint32_t totalOps;
  float opsPerMs;
};

template <typename Func>
TimedLoopResult runTimedLoop(uint32_t minDurationMs, uint32_t opsPerIteration, Func func) {
  TimedLoopResult result = {};
  unsigned long start = micros();
  unsigned long elapsed = 0;
  do {
    func();
    result.iterations++;
    result.totalOps += opsPerIteration;
#if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_RP2040)
    yield();
#endif
    elapsed = micros() - start;
  } while (elapsed < (minDurationMs * 1000UL));
  result.elapsedMicros = elapsed;
  result.opsPerMs = (result.totalOps * 1000.0f) / result.elapsedMicros;
  return result;
}

#endif
