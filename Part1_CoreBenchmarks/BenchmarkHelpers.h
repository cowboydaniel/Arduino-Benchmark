#ifndef BENCHMARK_HELPERS_H
#define BENCHMARK_HELPERS_H

// Benchmark helper functions compatible with older C++ standards
// Works with Arduino Uno (AVR), Uno Q (Zephyr/STM32U585), and other boards

#define BENCHMARK_VERSION "1.0.0"

// CSV output mode: 0 = human-readable (default), 1 = CSV column output
// In CSV mode, only label,value pairs are printed — no decorative output.
// Set to 1 before uploading, then copy the CSV block from Serial Monitor
// and run: python tools/import_results.py results.csv <spreadsheet>.xlsx
#ifndef CSV_OUTPUT
#define CSV_OUTPUT 0
#endif

#include <Arduino.h>

namespace benchmark_detail {
template<bool Value>
struct bool_constant {
  static const bool value = Value;
  using type = bool_constant<Value>;
};

using true_type = bool_constant<true>;
using false_type = bool_constant<false>;

template<typename A, typename B>
struct is_same : false_type {};

template<typename A>
struct is_same<A, A> : true_type {};
}  // namespace benchmark_detail

struct TimedLoopResult {
  unsigned long elapsedMicros;
  uint32_t iterations;
  uint32_t totalOps;
  float opsPerMs;
};

template<typename Func>
bool runFuncAndCheck(Func& func, benchmark_detail::true_type) {
  func();
  return true;
}

template<typename Func>
bool runFuncAndCheck(Func& func, benchmark_detail::false_type) {
  return func();
}

template<typename Func>
TimedLoopResult runTimedLoop(uint32_t minDurationMs, uint32_t opsPerIteration, Func func) {
  TimedLoopResult result = {};
  unsigned long start = micros();
  unsigned long elapsed = 0;
  do {
    bool shouldContinue = runFuncAndCheck(
      func,
      typename benchmark_detail::is_same<decltype(func()), void>::type());
    if (!shouldContinue) {
      elapsed = micros() - start;
      break;
    }
    result.iterations += 1;
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
