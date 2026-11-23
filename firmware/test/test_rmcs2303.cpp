// Host-side tests for the RMCS-2303 MODBUS ASCII driver.

#include "../labrobot_firmware/rmcs2303.h"

#include <cstdio>
#include <string>

uint32_t g_fake_millis = 0;
bool g_millis_auto_advance = false;

using labrobot::Rmcs2303;

static int g_failures = 0;

static void expectEq(const char* what, const std::string& got,
                     const std::string& want) {
  if (got == want) {
    std::printf("  ok   %s\n", what);
  } else {
    std::printf("  FAIL %s\n         got  '%s'\n         want '%s'\n", what,
                got.c_str(), want.c_str());
    ++g_failures;
  }
}

static void expectEqU32(const char* what, uint32_t got, uint32_t want) {
  if (got == want) {
    std::printf("  ok   %s (0x%08X)\n", what, got);
  } else {
    std::printf("  FAIL %s: got 0x%08X want 0x%08X\n", what, got, want);
    ++g_failures;
  }
}

static void expectTrue(const char* what, bool cond) {
  if (cond) {
    std::printf("  ok   %s\n", what);
  } else {
    std::printf("  FAIL %s\n", what);
    ++g_failures;
  }
}

static void pumpUntilSettled(Rmcs2303& drv, HardwareSerial& port) {
  (void)port;
  for (int i = 0; i < 1000; ++i) {
    drv.poll();
    if (drv.complete() || drv.failed()) return;
    g_fake_millis += 1;
  }
}

int main() {
  std::printf("RMCS-2303 driver tests\n");

  {
    HardwareSerial port;
    Rmcs2303 drv;
    drv.begin(&port, 7);
    expectTrue("beginWrite accepted while idle",
               drv.beginWrite(labrobot::REG_CONTROL, labrobot::CTRL_ENABLE_CW));
    expectEq("write frame (enable CW)", port.tx, ":070600020101EF\r\n");
    expectTrue("write frame is 17 chars", port.tx.size() == 17);
    expectTrue("second request rejected while in flight",
               !drv.beginWrite(labrobot::REG_SPEED_CMD, 100));
  }

  // Speed command: slave 7, FC 06, reg 14 (0x0E), 1446 rpm (0x05A6).
  {
    HardwareSerial port;
    Rmcs2303 drv;
    drv.begin(&port, 7);
    drv.beginWrite(labrobot::REG_SPEED_CMD, 1446);
    expectEq("write frame (speed 1446)", port.tx, ":0706000E05A63A\r\n");
  }

  {
    HardwareSerial port;
    Rmcs2303 drv;
    drv.begin(&port, 7);
    drv.beginRead(labrobot::REG_POS_FB_LSB, 2);
    expectEq("read frame (position, 2 regs)", port.tx, ":070300140002E0\r\n");
  }

  {
    HardwareSerial port;
    Rmcs2303 drv;
    drv.begin(&port, 7);
    drv.beginRead(labrobot::REG_POS_FB_LSB, 2);
    port.feed(":0703042345000189\r\n");
    pumpUntilSettled(drv, port);
    expectTrue("positive read completed", drv.complete());
    expectTrue("two registers returned", drv.responseCount() == 2);
    const uint32_t pos = (static_cast<uint32_t>(drv.responseRegister(1)) << 16) |
                         drv.responseRegister(0);
    expectEqU32("assembled position", pos, 0x00012345u);
  }

  {
    HardwareSerial port;
    Rmcs2303 drv;
    drv.begin(&port, 7);
    drv.beginRead(labrobot::REG_POS_FB_LSB, 2);
    port.feed(":070304FFFFFFFFF6\r\n");
    pumpUntilSettled(drv, port);
    expectTrue("negative read completed", drv.complete());
    const int32_t pos = static_cast<int32_t>(
        (static_cast<uint32_t>(drv.responseRegister(1)) << 16) |
        drv.responseRegister(0));
    expectTrue("position sign-extends to -1", pos == -1);
  }

  {
    HardwareSerial port;
    Rmcs2303 drv;
    drv.begin(&port, 7);
    drv.beginRead(labrobot::REG_POS_FB_LSB, 2);
    port.feed(":0703042345000100\r\n");  // LRC 0x00, should be 0x89
    pumpUntilSettled(drv, port);
    expectTrue("bad LRC marked failed", drv.failed());
    expectTrue("bad LRC counted", drv.crcErrors() == 1);

    // The link must still work for the very next transaction.
    drv.acknowledge();
    port.tx.clear();
    expectTrue("driver usable after bad LRC",
               drv.beginRead(labrobot::REG_POS_FB_LSB, 2));
    port.feed(":0703042345000189\r\n");
    pumpUntilSettled(drv, port);
    expectTrue("recovers on next good frame", drv.complete());
  }

  {
    HardwareSerial port;
    Rmcs2303 drv;
    drv.begin(&port, 7);
    drv.beginRead(labrobot::REG_POS_FB_LSB, 2);
    port.feed(":0903042345000187\r\n");
    pumpUntilSettled(drv, port);
    expectTrue("foreign slave reply rejected", drv.failed());
  }

  // Timeout on a silent drive
  {
    HardwareSerial port;
    Rmcs2303 drv;
    drv.begin(&port, 7);
    g_fake_millis = 1000;
    drv.beginRead(labrobot::REG_POS_FB_LSB, 2);
    pumpUntilSettled(drv, port);
    expectTrue("silent drive times out", drv.failed());
    expectTrue("timeout counted", drv.timeouts() == 1);
  }

  {
    HardwareSerial port;
    Rmcs2303 drv;
    drv.begin(&port, 7);
    drv.beginRead(labrobot::REG_POS_FB_LSB, 2);
    port.feed(":07030423");                // truncated, no terminator
    port.feed(":0703042345000189\r\n");    // then a clean frame
    pumpUntilSettled(drv, port);
    expectTrue("resyncs on embedded ':'", drv.complete());
  }

  std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "ALL PASS" : "FAILED",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
