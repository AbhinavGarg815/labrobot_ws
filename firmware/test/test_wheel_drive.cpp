// Host-side tests for the per-wheel RMCS-2303 sequencing.

#include "../labrobot_firmware/wheel_drive.h"

#include <cstdio>

#include "fake_drive.h"

uint32_t g_fake_millis = 0;
bool g_millis_auto_advance = false;

using labrobot::WheelDrive;

static int g_failures = 0;

static void expectTrue(const char* what, bool cond) {
  if (cond) {
    std::printf("  ok   %s\n", what);
  } else {
    std::printf("  FAIL %s\n", what);
    ++g_failures;
  }
}

static void expectEqI32(const char* what, int32_t got, int32_t want) {
  if (got == want) {
    std::printf("  ok   %s (%d)\n", what, got);
  } else {
    std::printf("  FAIL %s: got %d want %d\n", what, got, want);
    ++g_failures;
  }
}

static void run(WheelDrive& w, HardwareSerial& port, FakeDrive& drive, int iters) {
  for (int i = 0; i < iters; ++i) {
    w.poll();
    drive.service(port);
    w.poll();
    g_fake_millis += 1;
  }
}

// Drives past the two one-time setup writes so tests start from RUNNING.
static void initialise(WheelDrive& w, HardwareSerial& port, FakeDrive& drive) {
  run(w, port, drive, 40);
}

static bool wroteValue(const FakeDrive& d, uint8_t addr, uint16_t value) {
  for (const auto& kv : d.writes) {
    if (kv.first == addr && kv.second == value) return true;
  }
  return false;
}

int main() {
  std::printf("WheelDrive state-machine tests\n");

  // One-time initialisation
  {
    HardwareSerial port;
    FakeDrive drive;
    WheelDrive w;
    w.begin(&port, 7, 13, 500);
    expectTrue("starts uninitialised", !w.initialised());
    initialise(w, port, drive);
    expectTrue("reports initialised", w.initialised());
    expectTrue("wrote lines-per-rotation = 13",
               wroteValue(drive, labrobot::REG_LINES_PER_ROT, 13));
    expectTrue("wrote acceleration = 500",
               wroteValue(drive, labrobot::REG_ACCELERATION, 500));
  }

  // Forward: direction must be written BEFORE magnitude
  {
    HardwareSerial port;
    FakeDrive drive;
    WheelDrive w;
    w.begin(&port, 7, 13, 500);
    initialise(w, port, drive);
    drive.writes.clear();

    w.setTargetRpm(2410);  // 10 wheel-rpm * 241
    run(w, port, drive, 20);

    expectTrue("enabled CW", wroteValue(drive, labrobot::REG_CONTROL,
                                        labrobot::CTRL_ENABLE_CW));
    expectTrue("speed magnitude written",
               wroteValue(drive, labrobot::REG_SPEED_CMD, 2410));

    int ctrl_idx = -1, speed_idx = -1;
    for (size_t i = 0; i < drive.writes.size(); ++i) {
      if (drive.writes[i].first == labrobot::REG_CONTROL && ctrl_idx < 0)
        ctrl_idx = static_cast<int>(i);
      if (drive.writes[i].first == labrobot::REG_SPEED_CMD && speed_idx < 0)
        speed_idx = static_cast<int>(i);
    }
    expectTrue("direction written before speed",
               ctrl_idx >= 0 && speed_idx > ctrl_idx);
  }

  {
    HardwareSerial port;
    FakeDrive drive;
    WheelDrive w;
    w.begin(&port, 7, 13, 500);
    initialise(w, port, drive);
    drive.writes.clear();

    w.setTargetRpm(-2410);
    run(w, port, drive, 20);

    expectTrue("enabled CCW", wroteValue(drive, labrobot::REG_CONTROL,
                                         labrobot::CTRL_ENABLE_CCW));
    expectTrue("magnitude is positive 2410",
               wroteValue(drive, labrobot::REG_SPEED_CMD, 2410));
    bool wrote_negative = false;
    for (const auto& kv : drive.writes) {
      if (kv.first == labrobot::REG_SPEED_CMD && kv.second > 32767) wrote_negative = true;
    }
    expectTrue("never wrote a wrapped negative speed", !wrote_negative);
  }

  // Zero brakes rather than coasting
  {
    HardwareSerial port;
    FakeDrive drive;
    WheelDrive w;
    w.begin(&port, 7, 13, 500);
    initialise(w, port, drive);
    w.setTargetRpm(2410);
    run(w, port, drive, 20);
    drive.writes.clear();

    w.setTargetRpm(0);
    run(w, port, drive, 20);
    expectTrue("braked at zero", wroteValue(drive, labrobot::REG_CONTROL,
                                            labrobot::CTRL_BRAKE_CW));
    expectTrue("speed zeroed", wroteValue(drive, labrobot::REG_SPEED_CMD, 0));
  }

  // Encoder feedback, forward and reverse
  {
    HardwareSerial port;
    FakeDrive drive;
    WheelDrive w;
    w.begin(&port, 7, 13, 500);
    initialise(w, port, drive);

    drive.setPosition(123456);
    run(w, port, drive, 30);
    expectEqI32("reads positive position", w.positionCounts(), 123456);

    drive.setPosition(-98765);
    run(w, port, drive, 30);
    expectEqI32("reads negative position", w.positionCounts(), -98765);
  }

  // E-stop
  {
    HardwareSerial port;
    FakeDrive drive;
    WheelDrive w;
    w.begin(&port, 7, 13, 500);
    initialise(w, port, drive);
    drive.writes.clear();

    w.emergencyStop();
    run(w, port, drive, 20);
    expectTrue("sent E-stop control word",
               wroteValue(drive, labrobot::REG_CONTROL, labrobot::CTRL_ESTOP));
  }

  // A failed write must be retried, not assumed applied
  {
    HardwareSerial port;
    FakeDrive drive;
    WheelDrive w;
    w.begin(&port, 7, 13, 500);
    initialise(w, port, drive);
    drive.writes.clear();

    drive.corrupt_next_reply = true;
    w.setTargetRpm(2410);
    run(w, port, drive, 60);

    // Despite the corrupted acknowledgement, the drive must end up holding the commanded values.
    expectTrue("control eventually applied after a bad reply",
               drive.registers[labrobot::REG_CONTROL] == labrobot::CTRL_ENABLE_CW);
    expectTrue("speed eventually applied after a bad reply",
               drive.registers[labrobot::REG_SPEED_CMD] == 2410);
  }

  // Health tracking on a dead link
  {
    HardwareSerial port;
    FakeDrive drive;
    WheelDrive w;
    w.begin(&port, 7, 13, 500);
    initialise(w, port, drive);
    expectTrue("healthy while the drive answers", w.healthy());

    drive.silent = true;
    for (int i = 0; i < 100; ++i) {
      w.poll();
      drive.service(port);
      g_fake_millis += 20;
    }
    expectTrue("unhealthy after repeated timeouts", !w.healthy());
    expectTrue("at least kFaultThreshold timeouts occurred",
               w.timeouts() >= WheelDrive::kFaultThreshold);

    // And it must recover once the drive answers again.
    drive.silent = false;
    run(w, port, drive, 300);
    expectTrue("recovers when the link returns", w.healthy());
  }

  {
    HardwareSerial port;
    FakeDrive drive;
    WheelDrive w;
    w.begin(&port, 7, 13, 500);
    w.setInverted(true);
    initialise(w, port, drive);
    drive.writes.clear();

    // Positive request must reach the drive as CCW once inverted.
    w.setTargetRpm(2410);
    run(w, port, drive, 20);
    expectTrue("inverted: positive request -> CCW at the drive",
               wroteValue(drive, labrobot::REG_CONTROL, labrobot::CTRL_ENABLE_CCW));
    expectTrue("inverted: magnitude still unsigned 2410",
               wroteValue(drive, labrobot::REG_SPEED_CMD, 2410));

    drive.setPosition(5000);
    run(w, port, drive, 30);
    expectEqI32("inverted: encoder sign flipped", w.positionCounts(), -5000);

    // And a non-inverted wheel must be unaffected.
    HardwareSerial port2;
    FakeDrive drive2;
    WheelDrive w2;
    w2.begin(&port2, 7, 13, 500);
    w2.setInverted(false);
    initialise(w2, port2, drive2);
    drive2.setPosition(5000);
    run(w2, port2, drive2, 30);
    expectEqI32("non-inverted: encoder sign unchanged", w2.positionCounts(), 5000);
  }

  {
    HardwareSerial port;
    FakeDrive drive;
    WheelDrive w;
    w.begin(&port, 7, 13, 500);
    initialise(w, port, drive);
    drive.writes.clear();
    drive.reads.clear();

    // A different speed on every single slot - the worst case for starvation.
    for (int i = 0; i < 40; ++i) {
      w.setTargetRpm(static_cast<int16_t>(2000 + i * 7));
      run(w, port, drive, 1);
    }

    expectTrue("position still read despite a speed change every slot",
               drive.reads.size() > 0);
    // kMaxWritesBetweenPos = 2, so the worst case is one read per three transactions.
    expectTrue("position read at least every few transactions",
               drive.reads.size() >= 5);
    expectTrue("speed commands still got through",
               drive.writes.size() >= 5);
  }

  std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "ALL PASS" : "FAILED",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
