// Host-side tests for the RMCS-2303 MODBUS ASCII driver.
//
// Build and run:
//     firmware/test/run_tests.sh
//
// The expected frames below are hand-computed from the RMCS-2303 Operating
// Manual v1.0 framing rule:
//     ':' slave fc addrHi addrLo dataHi dataLo lrc CRLF
//     LRC = two's complement of the sum of the raw payload bytes.

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

// Drives poll() until the transaction leaves AWAITING or the fake clock runs
// past the driver timeout.
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

  // --- Write framing -------------------------------------------------------
  // Enable + CW in digital speed mode: slave 7, FC 06, reg 2, value 0x0101.
  // sum = 07+06+00+02+01+01 = 0x11 -> LRC = 0xEF
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
  // sum = 07+06+00+0E+05+A6 = 0xC6 -> LRC = 0x3A
  {
    HardwareSerial port;
    Rmcs2303 drv;
    drv.begin(&port, 7);
    drv.beginWrite(labrobot::REG_SPEED_CMD, 1446);
    expectEq("write frame (speed 1446)", port.tx, ":0706000E05A63A\r\n");
  }

  // --- Read framing --------------------------------------------------------
  // Position feedback: slave 7, FC 03, reg 20 (0x14), 2 registers.
  // sum = 07+03+00+14+00+02 = 0x20 -> LRC = 0xE0
  {
    HardwareSerial port;
    Rmcs2303 drv;
    drv.begin(&port, 7);
    drv.beginRead(labrobot::REG_POS_FB_LSB, 2);
    expectEq("read frame (position, 2 regs)", port.tx, ":070300140002E0\r\n");
  }

  // --- Read response, positive position ------------------------------------
  // Reply carrying LSB=0x2345, MSB=0x0001 -> position 0x00012345.
  // sum = 07+03+04+23+45+00+01 = 0x77 -> LRC = 0x89
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

  // --- Read response, negative position ------------------------------------
  // Reverse travel must sign-extend correctly: LSB=0xFFFF, MSB=0xFFFF is -1.
  // sum = 07+03+04+FF+FF+FF+FF = 0x0A (mod 256) -> LRC = 0xF6
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

  // --- Corrupt LRC is rejected, and does NOT kill the link -----------------
  // This is the specific defect that makes the vendor library unusable: it
  // calls port->end() here, permanently disabling the motor UART.
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

  // --- Reply from the wrong slave is ignored -------------------------------
  // sum = 09+03+04+23+45+00+01 = 0x79 -> LRC = 0x87
  {
    HardwareSerial port;
    Rmcs2303 drv;
    drv.begin(&port, 7);
    drv.beginRead(labrobot::REG_POS_FB_LSB, 2);
    port.feed(":0903042345000187\r\n");
    pumpUntilSettled(drv, port);
    expectTrue("foreign slave reply rejected", drv.failed());
  }

  // --- Timeout on a silent drive -------------------------------------------
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

  // --- Resynchronisation after truncated frame -----------------------------
  // A partial frame followed by a good one must still parse: ':' resets state.
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
