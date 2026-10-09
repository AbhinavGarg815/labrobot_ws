// A simulated RMCS-2303 for host-side tests.
//
// It consumes whatever the firmware transmitted, parses it with an independent
// implementation of the MODBUS ASCII rules, and replies. Because the parsing
// here is written separately from rmcs2303.cpp, a framing bug in either one
// shows up as a test failure rather than cancelling out.

#ifndef LABROBOT_TEST_FAKE_DRIVE_H
#define LABROBOT_TEST_FAKE_DRIVE_H

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "Arduino.h"

class FakeDrive {
 public:
  explicit FakeDrive(uint8_t slave = 7) : slave_(slave) {}

  // Registers the drive exposes, by data address.
  std::map<uint8_t, uint16_t> registers;

  // Every write the firmware performed, in order: (address, value).
  std::vector<std::pair<uint8_t, uint16_t>> writes;
  // Reads matter as much as writes: wheel velocity is differentiated from
  // consecutive position reads, so a firmware that stops reading has stale
  // odometry without ever failing a transaction.
  std::vector<uint8_t> reads;

  // When true, requests are consumed but never answered - simulates an
  // unpowered or unplugged drive so timeout handling can be exercised.
  bool silent = false;

  // Corrupts the next reply's LRC, to exercise error recovery.
  bool corrupt_next_reply = false;

  void setPosition(int32_t counts) {
    const uint32_t raw = static_cast<uint32_t>(counts);
    registers[20] = static_cast<uint16_t>(raw & 0xFFFF);
    registers[22] = static_cast<uint16_t>(raw >> 16);
  }

  // Processes any complete request sitting in port.tx and queues the reply
  // into port.rx.
  void service(HardwareSerial& port) {
    size_t start;
    while ((start = port.tx.find(':')) != std::string::npos) {
      const size_t end = port.tx.find("\r\n", start);
      if (end == std::string::npos) return;  // request still incomplete

      const std::string frame = port.tx.substr(start + 1, end - start - 1);
      port.tx.erase(0, end + 2);
      handle(frame, port);
    }
  }

 private:
  static uint8_t hexByte(const std::string& s, size_t i) {
    return static_cast<uint8_t>(std::stoul(s.substr(i, 2), nullptr, 16));
  }

  static void appendByte(std::string& out, uint8_t b, uint8_t& sum) {
    static const char* kHex = "0123456789ABCDEF";
    sum = static_cast<uint8_t>(sum + b);
    out.push_back(kHex[b >> 4]);
    out.push_back(kHex[b & 0x0F]);
  }

  void handle(const std::string& frame, HardwareSerial& port) {
    if (frame.size() < 14) return;

    const uint8_t slave = hexByte(frame, 0);
    const uint8_t fc = hexByte(frame, 2);
    const uint8_t addr = hexByte(frame, 6);
    const uint16_t data =
        static_cast<uint16_t>((hexByte(frame, 8) << 8) | hexByte(frame, 10));

    if (slave != slave_ || silent) return;

    std::string body;
    uint8_t sum = 0;

    if (fc == 0x06) {
      registers[addr] = data;
      writes.emplace_back(addr, data);
      // Write replies echo the request verbatim.
      appendByte(body, slave, sum);
      appendByte(body, fc, sum);
      appendByte(body, 0x00, sum);
      appendByte(body, addr, sum);
      appendByte(body, static_cast<uint8_t>(data >> 8), sum);
      appendByte(body, static_cast<uint8_t>(data & 0xFF), sum);
    } else if (fc == 0x03) {
      reads.push_back(addr);
      const uint8_t reg_count = static_cast<uint8_t>(data);
      const uint8_t byte_count = static_cast<uint8_t>(reg_count * 2);
      appendByte(body, slave, sum);
      appendByte(body, fc, sum);
      appendByte(body, byte_count, sum);
      // Registers step by 2 in this address space.
      for (uint8_t i = 0; i < reg_count; ++i) {
        const uint8_t a = static_cast<uint8_t>(addr + i * 2);
        const uint16_t v = registers.count(a) ? registers[a] : 0;
        appendByte(body, static_cast<uint8_t>(v >> 8), sum);
        appendByte(body, static_cast<uint8_t>(v & 0xFF), sum);
      }
    } else {
      return;
    }

    uint8_t lrc = static_cast<uint8_t>(-static_cast<int8_t>(sum));
    if (corrupt_next_reply) {
      lrc = static_cast<uint8_t>(lrc ^ 0xFF);
      corrupt_next_reply = false;
    }
    uint8_t ignored = 0;
    std::string reply = ":" + body;
    appendByte(reply, lrc, ignored);
    reply += "\r\n";
    port.feed(reply);
  }

  uint8_t slave_;
};

#endif  // LABROBOT_TEST_FAKE_DRIVE_H
