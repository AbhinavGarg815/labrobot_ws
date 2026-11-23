// Host-side tests for the Jetson <-> Mega wire format.

#include "../labrobot_firmware/labrobot_protocol.h"

#include <cstdio>
#include <cstring>
#include <vector>

using namespace labrobot::protocol;

static int g_failures = 0;

static void expectTrue(const char* what, bool cond) {
  if (cond) {
    std::printf("  ok   %s\n", what);
  } else {
    std::printf("  FAIL %s\n", what);
    ++g_failures;
  }
}

static void expectEqI(const char* what, long got, long want) {
  if (got == want) {
    std::printf("  ok   %s (%ld)\n", what, got);
  } else {
    std::printf("  FAIL %s: got %ld want %ld\n", what, got, want);
    ++g_failures;
  }
}

// Pushes a whole buffer through the parser, returning how many frames emerged.
static int pump(FrameParser& p, const uint8_t* data, size_t len) {
  int frames = 0;
  for (size_t i = 0; i < len; ++i) {
    if (p.feed(data[i])) ++frames;
  }
  return frames;
}

int main() {
  std::printf("Host protocol tests\n");

  {
    const uint8_t v[] = "123456789";
    expectEqI("CRC-16/CCITT-FALSE check value", crc16(v, 9), 0x29B1);
  }

  // Velocity round trip, including negatives
  {
    uint8_t buf[kMaxFrame];
    VelocityCommand cmd{-6283, 6283};  // full reverse / full forward
    const size_t n = encodeVelocity(cmd, buf, sizeof(buf));
    expectEqI("velocity frame length", static_cast<long>(n),
              kVelocityPayloadLen + kFrameOverhead);
    expectTrue("sync bytes", buf[0] == kSync0 && buf[1] == kSync1);

    FrameParser p;
    expectEqI("one frame parsed", pump(p, buf, n), 1);
    expectEqI("type is velocity", p.type(), kCmdVelocity);

    VelocityCommand got{};
    expectTrue("decodes", decodeVelocity(p.payload(), p.payloadLen(), &got));
    expectEqI("left preserved (negative)", got.left_mrad_s, -6283);
    expectEqI("right preserved", got.right_mrad_s, 6283);
  }

  // Status round trip, including negative 32-bit counts
  {
    uint8_t buf[kMaxFrame];
    Status s{};
    s.seq = 200;
    s.flags = kFlagLeftHealthy | kFlagRightHealthy | kFlagInitialised;
    s.left_counts = -1234567;   // reverse travel past the 16-bit boundary
    s.right_counts = 7654321;
    s.left_mrad_s = -1500;
    s.right_mrad_s = 1500;
    s.left_faults = 3;
    s.right_faults = 65535;
    s.host_frames = 4321;
    s.host_errors = 7;
    const size_t n = encodeStatus(s, buf, sizeof(buf));
    expectEqI("status frame length", static_cast<long>(n),
              kStatusPayloadLen + kFrameOverhead);

    FrameParser p;
    expectEqI("status parsed", pump(p, buf, n), 1);
    Status g{};
    expectTrue("decodes", decodeStatus(p.payload(), p.payloadLen(), &g));
    expectEqI("seq", g.seq, 200);
    expectEqI("flags", g.flags, s.flags);
    expectEqI("left counts (negative, >16 bit)", g.left_counts, -1234567);
    expectEqI("right counts", g.right_counts, 7654321);
    expectEqI("left mrad/s (negative)", g.left_mrad_s, -1500);
    expectEqI("right faults (max)", g.right_faults, 65535);
    expectEqI("host frames", g.host_frames, 4321);
    expectEqI("host errors", g.host_errors, 7);
  }

  // Payload-less commands
  {
    for (uint8_t t : {kCmdStop, kCmdEstop, kCmdResetOdom, kCmdPing}) {
      uint8_t buf[kMaxFrame];
      const size_t n = encodeCommand(t, buf, sizeof(buf));
      FrameParser p;
      const bool ok = pump(p, buf, n) == 1 && p.type() == t && p.payloadLen() == 0;
      char name[64];
      std::snprintf(name, sizeof(name), "zero-payload command 0x%02X", t);
      expectTrue(name, ok);
    }
  }

  {
    uint8_t good[kMaxFrame];
    VelocityCommand cmd{1234, -4321};
    const size_t n = encodeVelocity(cmd, good, sizeof(good));

    int missed = 0, tested = 0;
    for (size_t byte = 0; byte < n; ++byte) {
      for (int bit = 0; bit < 8; ++bit) {
        uint8_t bad[kMaxFrame];
        std::memcpy(bad, good, n);
        bad[byte] ^= static_cast<uint8_t>(1 << bit);
        if (std::memcmp(bad, good, n) == 0) continue;
        ++tested;
        FrameParser p;
        if (pump(p, bad, n) == 1) {
          // Corrupting a sync byte just means no frame starts - that is fine.
          VelocityCommand got{};
          if (decodeVelocity(p.payload(), p.payloadLen(), &got) &&
              (got.left_mrad_s != cmd.left_mrad_s ||
               got.right_mrad_s != cmd.right_mrad_s)) {
            ++missed;
          }
        }
      }
    }
    char name[96];
    std::snprintf(name, sizeof(name),
                  "all %d single-bit corruptions rejected or harmless", tested);
    expectTrue(name, missed == 0);
  }

  {
    uint8_t buf[kMaxFrame];
    uint8_t payload[2] = {0x10, 0x20};
    const size_t n = encodeFrame(kCmdVelocity, payload, 2, buf, sizeof(buf));
    FrameParser p;
    expectEqI("short velocity frame yields no message", pump(p, buf, n), 0);
    expectTrue("flagged as wrong payload length",
               p.lastError() == FrameError::kWrongPayloadLen);
  }

  // Unknown type is rejected even with a valid CRC
  {
    uint8_t buf[kMaxFrame];
    const size_t n = encodeFrame(0x7E, nullptr, 0, buf, sizeof(buf));
    FrameParser p;
    expectEqI("unknown type yields no message", pump(p, buf, n), 0);
    expectTrue("flagged as unknown type",
               p.lastError() == FrameError::kUnknownType);
  }

  // Oversized LEN is rejected without overrunning the buffer
  {
    FrameParser p;
    const uint8_t junk[] = {kSync0, kSync1, 0xFF, kCmdVelocity, 0, 0, 0, 0};
    expectEqI("oversized length yields no message",
              pump(p, junk, sizeof(junk)), 0);
    expectTrue("flagged as bad length", p.lastError() == FrameError::kBadLength);
  }

  {
    uint8_t good[kMaxFrame];
    const size_t n = encodeVelocity({500, -500}, good, sizeof(good));

    std::vector<uint8_t> stream;
    const uint8_t garbage[] = {0x00, 0xFF, 0xA5, 0x12, 'h', 'e', 'l', 'l', 'o'};
    stream.insert(stream.end(), garbage, garbage + sizeof(garbage));
    stream.insert(stream.end(), good, good + n);

    FrameParser p;
    expectEqI("parses through leading garbage", pump(p, stream.data(), stream.size()), 1);
    VelocityCommand got{};
    decodeVelocity(p.payload(), p.payloadLen(), &got);
    expectEqI("value intact after garbage", got.left_mrad_s, 500);
  }

  {
    uint8_t good[kMaxFrame];
    const size_t n = encodeVelocity({500, -500}, good, sizeof(good));

    std::vector<uint8_t> stream;
    stream.insert(stream.end(), good, good + n - 3);  // truncated
    stream.insert(stream.end(), good, good + n);      // likely consumed
    stream.insert(stream.end(), good, good + n);      // must parse

    FrameParser p;
    const int frames = pump(p, stream.data(), stream.size());
    expectTrue("truncation costs at most one extra frame", frames >= 1);
    VelocityCommand got{};
    decodeVelocity(p.payload(), p.payloadLen(), &got);
    expectEqI("value intact after recovery", got.left_mrad_s, 500);
  }

  {
    uint8_t good[kMaxFrame];
    const size_t n = encodeVelocity({777, -777}, good, sizeof(good));

    FrameParser p;
    pump(p, good, n - 3);  // truncated frame leaves the parser mid-payload
    p.reset();             // gap detected
    expectEqI("no frame lost when reset() follows a gap", pump(p, good, n), 1);
    VelocityCommand got{};
    decodeVelocity(p.payload(), p.payloadLen(), &got);
    expectEqI("value intact after reset", got.left_mrad_s, 777);
  }

  // Back-to-back frames with no gap
  {
    uint8_t buf[kMaxFrame * 3];
    size_t n = 0;
    n += encodeVelocity({100, 200}, buf + n, sizeof(buf) - n);
    n += encodeCommand(kCmdStop, buf + n, sizeof(buf) - n);
    n += encodeVelocity({-300, -400}, buf + n, sizeof(buf) - n);
    FrameParser p;
    expectEqI("three back-to-back frames", pump(p, buf, n), 3);
    VelocityCommand got{};
    decodeVelocity(p.payload(), p.payloadLen(), &got);
    expectEqI("last frame's value", got.left_mrad_s, -300);
  }

  // Buffer-capacity guard
  {
    uint8_t tiny[4];
    expectEqI("encode refuses an undersized buffer",
              static_cast<long>(encodeVelocity({0, 0}, tiny, sizeof(tiny))), 0);
  }

  std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "ALL PASS" : "FAILED",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
