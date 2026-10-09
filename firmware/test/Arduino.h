// Minimal Arduino shim so the RMCS-2303 driver can be compiled and exercised
// on the host. This exists purely for firmware/test/test_rmcs2303.cpp - it lets
// us prove the LRC arithmetic and frame parser are correct before any of it
// touches a real drive, where a malformed frame means an unexpected motor move.

#ifndef LABROBOT_TEST_ARDUINO_H
#define LABROBOT_TEST_ARDUINO_H

#include <cstdint>
#include <cstring>
#include <deque>
#include <string>

// Test-controlled clock so timeout behaviour is deterministic.
extern uint32_t g_fake_millis;

// When set, every millis() call advances the clock by 1 ms. Real time passes
// inside a poll() too, and code that samples millis() more than once per pass
// can then compare timestamps taken at different instants. A clock that only
// ticks between iterations hides that entire class of bug.
extern bool g_millis_auto_advance;

inline uint32_t millis() {
  const uint32_t t = g_fake_millis;
  if (g_millis_auto_advance) ++g_fake_millis;
  return t;
}

// Stands in for Arduino's HardwareSerial. Bytes written by the driver land in
// tx; bytes the test injects into rx are handed back by read().
class HardwareSerial {
 public:
  std::string tx;
  std::deque<uint8_t> rx;

  int available() { return static_cast<int>(rx.size()); }

  int read() {
    if (rx.empty()) return -1;
    const uint8_t c = rx.front();
    rx.pop_front();
    return c;
  }

  size_t write(const uint8_t* buf, size_t n) {
    tx.append(reinterpret_cast<const char*>(buf), n);
    return n;
  }

  void feed(const std::string& s) {
    for (char c : s) rx.push_back(static_cast<uint8_t>(c));
  }
};

#endif  // LABROBOT_TEST_ARDUINO_H
