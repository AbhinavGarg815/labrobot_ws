
#include "../labrobot_firmware/host_link.h"

#include <cstdio>
#include <vector>

#include "fake_drive.h"

uint32_t g_fake_millis = 0;
bool g_millis_auto_advance = false;

using labrobot::HostLink;
using labrobot::WheelDrive;
namespace proto = labrobot::protocol;

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

static void expectNear(const char* what, long got, long want, long tol) {
  if (got >= want - tol && got <= want + tol) {
    std::printf("  ok   %s (%ld)\n", what, got);
  } else {
    std::printf("  FAIL %s: got %ld want %ld +/- %ld\n", what, got, want, tol);
    ++g_failures;
  }
}

// One complete rig: host port, two wheels each with a simulated drive.
struct Rig {
  HardwareSerial host;
  HardwareSerial lport, rport;
  FakeDrive ldrive, rdrive;
  WheelDrive left, right;
  HostLink link;

  Rig() {
    left.begin(&lport, 7, 13, 20000);
    right.begin(&rport, 7, 13, 20000);
    HostLink::Config cfg;
    link.begin(&host, &left, &right, cfg);
  }

  // Advance everything by `ms` milliseconds of simulated time.
  void run(int ms) {
    for (int i = 0; i < ms; ++i) {
      link.poll();
      left.poll();
      right.poll();
      ldrive.service(lport);
      rdrive.service(rport);
      left.poll();
      right.poll();
      g_fake_millis += 1;
    }
  }

  void send(const uint8_t* frame, size_t n) { host.feed(std::string(frame, frame + n)); }

  void sendVelocity(int16_t l, int16_t r) {
    uint8_t f[proto::kMaxFrame];
    const size_t n = proto::encodeVelocity({l, r}, f, sizeof(f));
    send(f, n);
  }

  void sendCommand(uint8_t type) {
    uint8_t f[proto::kMaxFrame];
    const size_t n = proto::encodeCommand(type, f, sizeof(f));
    send(f, n);
  }

  // Pulls every complete Status frame out of what the link transmitted.
  std::vector<proto::Status> statuses() {
    std::vector<proto::Status> out;
    proto::FrameParser p;
    for (unsigned char c : host.tx) {
      if (p.feed(static_cast<uint8_t>(c)) && p.type() == proto::kMsgStatus) {
        proto::Status s{};
        if (proto::decodeStatus(p.payload(), p.payloadLen(), &s)) out.push_back(s);
      }
    }
    return out;
  }

  bool sawType(uint8_t type) {
    proto::FrameParser p;
    for (unsigned char c : host.tx) {
      if (p.feed(static_cast<uint8_t>(c)) && p.type() == type) return true;
    }
    return false;
  }
};

int main() {
  std::printf("HostLink tests\n");

  {
    Rig rig;
    expectEqI("full scale +6283 mrad/s -> base rpm",
              rig.link.wheelMradSToBaseRpm(6283), 14460);
    expectEqI("full scale -6283 mrad/s -> base rpm",
              rig.link.wheelMradSToBaseRpm(-6283), -14460);
    expectEqI("zero maps to zero", rig.link.wheelMradSToBaseRpm(0), 0);
    // 10 wheel-rpm = 1.047 rad/s = 1047 mrad/s -> 2410 base rpm.
    expectNear("10 wheel-rpm round trip", rig.link.wheelMradSToBaseRpm(1047), 2410, 5);
    // Over-range must saturate, never wrap into a huge speed.
    expectEqI("over-range saturates high",
              rig.link.wheelMradSToBaseRpm(32000), 14460);
    expectEqI("over-range saturates low",
              rig.link.wheelMradSToBaseRpm(-32000), -14460);
    // Feedback direction: 2089 counts/s is 10 wheel-rpm.
    expectNear("counts/s -> mrad/s", rig.link.countsPerSecToMradS(2088.7f), 1047, 5);
    expectNear("negative counts/s -> mrad/s",
               rig.link.countsPerSecToMradS(-2088.7f), -1047, 5);
  }

  // A velocity command reaches the drives
  {
    Rig rig;
    rig.run(60);  // let one-time drive setup finish
    rig.ldrive.writes.clear();
    rig.rdrive.writes.clear();

    rig.sendVelocity(1047, 1047);  // 10 wheel-rpm both sides
    rig.run(60);

    bool wrote_speed = false;
    for (const auto& kv : rig.ldrive.writes) {
      if (kv.first == labrobot::REG_SPEED_CMD && kv.second == 2410) wrote_speed = true;
    }
    expectTrue("velocity command reached the left drive", wrote_speed);
    expectTrue("link counted one frame", rig.link.framesReceived() == 1);
    expectTrue("no frame errors", rig.link.frameErrors() == 0);
  }

  // Status is published, and carries encoder state
  {
    Rig rig;
    rig.ldrive.setPosition(12532);   // exactly one wheel revolution
    rig.rdrive.setPosition(-6266);
    rig.run(200);

    auto st = rig.statuses();
    expectTrue("status frames published", st.size() >= 5);
    if (!st.empty()) {
      const auto& s = st.back();
      // Left is inverted nowhere in this rig, so counts pass through.
      expectEqI("status carries left counts", s.left_counts, 12532);
      expectEqI("status carries right counts", s.right_counts, -6266);
      expectTrue("initialised flag set", (s.flags & proto::kFlagInitialised) != 0);
      expectTrue("both wheels healthy",
                 (s.flags & proto::kFlagLeftHealthy) &&
                 (s.flags & proto::kFlagRightHealthy));
    }
    // Sequence numbers must advance so the host can spot dropped frames.
    if (st.size() >= 2) {
      expectEqI("seq increments",
                static_cast<uint8_t>(st[1].seq - st[0].seq), 1);
    }
  }

  // WATCHDOG: silence brakes the robot
  {
    Rig rig;
    rig.run(60);
    rig.sendVelocity(1047, 1047);
    rig.run(100);
    expectTrue("watchdog not tripped while commands flow", !rig.link.watchdogTripped());

    rig.ldrive.writes.clear();
    rig.run(600);  // silence, past the 500 ms timeout
    expectTrue("watchdog trips after timeout", rig.link.watchdogTripped());

    bool braked = false;
    for (const auto& kv : rig.ldrive.writes) {
      if (kv.first == labrobot::REG_CONTROL &&
          (kv.second == labrobot::CTRL_BRAKE_CW || kv.second == labrobot::CTRL_BRAKE_CCW)) {
        braked = true;
      }
    }
    expectTrue("watchdog actually braked the drive", braked);
    expectEqI("drive speed commanded to zero",
              rig.ldrive.registers[labrobot::REG_SPEED_CMD], 0);

    auto st = rig.statuses();
    expectTrue("watchdog flag reported to host",
               !st.empty() && (st.back().flags & proto::kFlagWatchdogTrip));
  }

  // Watchdog is refreshed by a steady command stream
  {
    Rig rig;
    rig.run(60);
    for (int i = 0; i < 10; ++i) {
      rig.sendVelocity(1047, 1047);
      rig.run(100);  // 100 ms apart, well inside the 500 ms timeout
    }
    expectTrue("steady commands keep the watchdog clear", !rig.link.watchdogTripped());
  }

  // Watchdog recovers when commands resume
  {
    Rig rig;
    rig.run(60);
    rig.sendVelocity(1047, 1047);
    rig.run(700);
    expectTrue("tripped after silence", rig.link.watchdogTripped());
    rig.sendVelocity(1047, 1047);
    rig.run(60);
    expectTrue("clears once commands resume", !rig.link.watchdogTripped());
  }

  {
    Rig rig;
    rig.run(2000);
    expectTrue("no spurious trip before first command", !rig.link.watchdogTripped());
  }

  // E-stop
  {
    Rig rig;
    rig.run(60);
    rig.ldrive.writes.clear();
    rig.sendCommand(proto::kCmdEstop);
    rig.run(80);
    expectEqI("e-stop control word sent",
              rig.ldrive.registers[labrobot::REG_CONTROL], labrobot::CTRL_ESTOP);
    auto st = rig.statuses();
    expectTrue("e-stop flag reported",
               !st.empty() && (st.back().flags & proto::kFlagEstopped));
  }

  // Ping / pong
  {
    Rig rig;
    rig.run(30);
    rig.sendCommand(proto::kCmdPing);
    rig.run(30);
    expectTrue("ping answered with pong", rig.sawType(proto::kMsgPong));
  }

  // Corrupt frames are ignored, and do not disturb the drives
  {
    Rig rig;
    rig.run(60);
    rig.sendVelocity(1047, 1047);
    rig.run(60);
    rig.ldrive.writes.clear();

    // A velocity frame with a mangled CRC must be dropped entirely.
    uint8_t f[proto::kMaxFrame];
    const size_t n = proto::encodeVelocity({6283, 6283}, f, sizeof(f));
    f[n - 1] ^= 0xFF;
    rig.send(f, n);
    rig.run(80);

    bool applied_full_speed = false;
    for (const auto& kv : rig.ldrive.writes) {
      if (kv.first == labrobot::REG_SPEED_CMD && kv.second == 14460) {
        applied_full_speed = true;
      }
    }
    expectTrue("corrupt command never reached the drive", !applied_full_speed);
    expectTrue("frame error counted", rig.link.frameErrors() >= 1);
  }

  {
    Rig rig;
    rig.run(60);
    uint8_t f[proto::kMaxFrame];
    const size_t n = proto::encodeVelocity({1047, 1047}, f, sizeof(f));

    rig.send(f, n - 3);  // truncated
    rig.run(30);         // gap longer than frame_gap_ms
    rig.ldrive.writes.clear();
    rig.send(f, n);      // complete frame
    rig.run(80);

    bool applied = false;
    for (const auto& kv : rig.ldrive.writes) {
      if (kv.first == labrobot::REG_SPEED_CMD && kv.second == 2410) applied = true;
    }
    expectTrue("frame after a truncation is still applied", applied);
  }

  {
    g_millis_auto_advance = true;
    Rig rig;
    rig.run(60);

    bool ever_tripped = false;
    for (int i = 0; i < 20; ++i) {
      rig.sendVelocity(1047, 1047);
      rig.run(5);  // 20 Hz-ish, far inside the 500 ms timeout
      if (rig.link.watchdogTripped()) ever_tripped = true;
    }
    expectTrue("watchdog stays clear when the clock advances mid-poll",
               !ever_tripped);
    expectTrue("commands were actually received",
               rig.link.framesReceived() >= 20);
    expectEqI("and none were rejected", rig.link.frameErrors(), 0);
    g_millis_auto_advance = false;
  }

  std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "ALL PASS" : "FAILED",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
