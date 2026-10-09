// labrobot motor-control firmware for the Arduino Mega 2560.
//
// Topology:
//   Serial  (USB) -> Jetson Orin Nano, and the bench console
//   Serial1       -> RMCS-2303 driving the LEFT wheel   (slave 7)
//   Serial2       -> RMCS-2303 driving the RIGHT wheel  (slave 7)
//
// Both drives use slave address 7. That is legal because each sits on its own
// point-to-point UART, and it is deliberate: separate UARTs let the two Modbus
// transactions overlap, which matters because the link is fixed at 9600 baud
// (~35 ms per write, ~37 ms per 2-register read).
//
// PRODUCTION MODE (default) speaks the binary framed protocol in
// labrobot_protocol.h to the Jetson at 500000 baud, and brakes both wheels if
// commands stop arriving.
//
// BENCH MODE (firmware/flash.sh --bench) is a human console at 115200 baud for
// commissioning and diagnosis. Newline-terminated keys:
//
//   f      both wheels forward at the test speed
//   b      both wheels reverse at the test speed
//   l / r  spin left / right in place
//   s      stop (brake)
//   +/-    raise/lower test speed by 5 wheel-rpm
//   h      zero both encoders
//   e      emergency stop (cuts motor power; any motion command re-enables)
//   ?      print status once  (also re-arms the auto-brake while moving)
//   n      neutral: release both motors so the wheels can be turned by hand
//   d      dump the raw bytes of the last Modbus exchange on each link
//   p      probe how the drive interprets the Modbus read count field
//
// Status prints twice a second: encoder counts, measured rpm, link health.
//
// SAFETY: the robot has no kill switch other than removing the battery. Put it
// on blocks with the wheels clear of the bench before sending a motion command.

// Build mode. Production (0) speaks the binary protocol to the Jetson; bench
// mode (1) is the human console used to commission and diagnose the drives.
// Select at build time:  firmware/flash.sh --bench
#ifndef LABROBOT_BENCH_MODE
#define LABROBOT_BENCH_MODE 0
#endif

#include "host_link.h"
#include "labrobot_protocol.h"
#include "rmcs2303.h"
#include "wheel_drive.h"

using labrobot::WheelDrive;

// ---------------------------------------------------------------------------
// Robot constants. These must agree with labrobot_properties.xacro.
// ---------------------------------------------------------------------------

static const uint8_t  kSlaveAddress   = 7;
static const uint16_t kLinesPerRot    = 13;     // RMCS-5030 encoder
static const uint16_t kGearRatio      = 241;    // 1:241
static const uint16_t kAcceleration   = 20000;  // drive reset value (0x4E20)

// The two motors are mounted mirrored, so exactly one of these must differ from
// the other or the robot spins on the spot instead of driving. Flipping BOTH
// keeps them coordinated while reversing which way the robot calls "forward".
//
// Determined on the bench 2026-07-30: with (false, true) the wheels moved
// together but backwards, so both are inverted here.
static const bool kInvertLeft  = true;
static const bool kInvertRight = false;
static const uint16_t kMaxWheelRpm    = 60;     // mechanical ceiling

// 52 quadrature counts per base-motor revolution through the 1:241 gearbox.
static const float kCountsPerWheelRev = 52.0f * 241.0f;  // 12532

// The RMCS-2303 speed register is in BASE-MOTOR rpm, so a wheel-rpm setpoint is
// multiplied by the gear ratio. 60 wheel-rpm = 14460 base-motor rpm, which fits
// in the unsigned 16-bit register and in the int16 we carry it around in.
static const uint16_t kMaxBaseRpm     = kMaxWheelRpm * kGearRatio;

static const uint32_t kMotorBaud      = 9600;   // fixed in drive firmware
static const uint32_t kConsoleBaud    = 115200;   // bench mode
// 500000 is exact on a 16 MHz ATmega2560 (U2X, UBRR=3, 0% error), unlike
// 460800 or 921600 which carry several percent and corrupt frames.
static const uint32_t kHostBaud       = 500000;   // production mode

static const uint16_t kStatusPeriodMs = 500;

// Bench safety: motion auto-brakes if no further command arrives within this
// window. The robot has no kill switch, so an operator console that crashes,
// an SSH drop or an unplugged USB cable must not leave the wheels driving.
// Every motion command re-arms it, so deliberate driving just needs a keypress.
static const uint16_t kAutoBrakeMs = 4000;

// ---------------------------------------------------------------------------

static WheelDrive g_left;
static WheelDrive g_right;

#if LABROBOT_BENCH_MODE
static uint16_t g_test_wheel_rpm = 10;  // conservative default
static uint32_t g_last_status_ms = 0;
static uint32_t g_motion_started_ms = 0;
static bool g_moving = false;

// Converts a signed wheel-rpm setpoint to the base-motor rpm the drive wants,
// clamping to the mechanical limit. The datasheet warns that commanding a speed
// the motor cannot reach gives an "uncertain result", so we never exceed it.
static int16_t wheelRpmToBaseRpm(int16_t wheel_rpm) {
  if (wheel_rpm > static_cast<int16_t>(kMaxWheelRpm)) wheel_rpm = kMaxWheelRpm;
  if (wheel_rpm < -static_cast<int16_t>(kMaxWheelRpm)) wheel_rpm = -kMaxWheelRpm;
  return static_cast<int16_t>(static_cast<int32_t>(wheel_rpm) * kGearRatio);
}

static void drive(int16_t left_wheel_rpm, int16_t right_wheel_rpm) {
  g_left.setTargetRpm(wheelRpmToBaseRpm(left_wheel_rpm));
  g_right.setTargetRpm(wheelRpmToBaseRpm(right_wheel_rpm));
  g_moving = (left_wheel_rpm != 0) || (right_wheel_rpm != 0);
  g_motion_started_ms = millis();
}

static void stopMotion() {
  g_left.brake();
  g_right.brake();
  g_moving = false;
}

static void printStatus() {
  Serial.print(F("L pos="));
  Serial.print(g_left.positionCounts());
  Serial.print(F(" wrpm="));
  Serial.print(g_left.velocityCountsPerSec() * 60.0f / kCountsPerWheelRev, 2);
  Serial.print(g_left.healthy() ? F(" OK ") : F(" FAULT "));
  Serial.print(F("(to="));
  Serial.print(g_left.timeouts());
  Serial.print(F(" crc="));
  Serial.print(g_left.crcErrors());
  Serial.print(F(")  |  R pos="));
  Serial.print(g_right.positionCounts());
  Serial.print(F(" wrpm="));
  Serial.print(g_right.velocityCountsPerSec() * 60.0f / kCountsPerWheelRev, 2);
  Serial.print(g_right.healthy() ? F(" OK ") : F(" FAULT "));
  Serial.print(F("(to="));
  Serial.print(g_right.timeouts());
  Serial.print(F(" crc="));
  Serial.print(g_right.crcErrors());
  Serial.print(F(") errL="));
  Serial.print(labrobot::parseErrorName(g_left.lastError()));
  Serial.print(F(" errR="));
  Serial.print(labrobot::parseErrorName(g_right.lastError()));
  Serial.print(F("  set="));
  Serial.print(g_test_wheel_rpm);
  Serial.println(F(" wheel-rpm"));
}

// Dumps the exact bytes on the wire for one drive. The request we sent is
// printed next to the reply we got, so a wrong slave id, a baud mismatch or a
// truncated frame is visible directly rather than inferred from a counter.
static void dumpLink(const __FlashStringHelper* label, const WheelDrive& w) {
  Serial.print(label);
  Serial.print(F("  last error: "));
  Serial.println(labrobot::parseErrorName(w.lastError()));

  Serial.print(F("    sent : '"));
  Serial.print(w.lastRequest());
  Serial.println(F("'"));

  Serial.print(F("    recv : '"));
  Serial.print(w.lastFrame());
  Serial.print(F("'  ("));
  Serial.print(w.lastFrameLen());
  Serial.println(F(" chars)"));

  // Hex, because a reply full of framing errors often contains bytes that do
  // not print, and those are exactly the interesting ones.
  Serial.print(F("    hex  :"));
  const char* f = w.lastFrame();
  for (uint8_t i = 0; i < w.lastFrameLen(); ++i) {
    Serial.print(' ');
    if (static_cast<uint8_t>(f[i]) < 0x10) Serial.print('0');
    Serial.print(static_cast<uint8_t>(f[i]), HEX);
  }
  Serial.println();
}

static void dumpBothLinks() {
  Serial.println(F("---- link diagnostics ----"));
  dumpLink(F("LEFT  (Serial1)"), g_left);
  dumpLink(F("RIGHT (Serial2)"), g_right);
  Serial.println(F("--------------------------"));
}

// Runs one read by hand and reports exactly what came back. Blocking, which is
// fine here: probes are a bench-only diagnostic, never part of the control loop.
static void probeRead(WheelDrive& w, uint8_t addr, uint8_t count) {
  labrobot::Rmcs2303& link = w.link();

  // Let any in-flight transaction finish; the caller has already suspended new
  // ones, so the link becomes idle and stays that way.
  const uint32_t settle = millis();
  while (!link.idle() && millis() - settle < 400) {
    w.poll();
  }
  if (link.complete() || link.failed()) link.acknowledge();

  // The driver caps requests at its own kMaxResponseRegs, so a count it refuses
  // is reported rather than silently skipped.
  if (!link.beginRead(addr, count)) {
    Serial.print(F("  addr="));
    Serial.print(addr);
    Serial.print(F(" count="));
    Serial.print(count);
    Serial.println(F("  -> refused by driver (count out of range)"));
    return;
  }

  const uint32_t start = millis();
  while (!link.complete() && !link.failed() && millis() - start < 300) {
    link.poll();
  }

  Serial.print(F("  addr="));
  Serial.print(addr);
  Serial.print(F(" count="));
  Serial.print(count);
  Serial.print(F(" -> '"));
  Serial.print(link.lastFrame());
  Serial.print(F("' regs="));
  Serial.print(link.responseCount());
  Serial.print(' ');
  for (uint8_t i = 0; i < link.responseCount(); ++i) {
    Serial.print(F("0x"));
    Serial.print(link.responseRegister(i), HEX);
    Serial.print(' ');
  }
  Serial.print(labrobot::parseErrorName(link.lastError()));
  Serial.println();

  link.acknowledge();
}

// Determines empirically how the drive interprets the read count field. The
// address space steps by 2, so "count" might mean registers or address units -
// and the 32-bit encoder position can only be read correctly once we know.
static void probeLink(const __FlashStringHelper* label, WheelDrive& w) {
  Serial.println(label);
  w.suspend();
  probeRead(w, labrobot::REG_POS_FB_LSB, 1);  // baseline: one register
  probeRead(w, labrobot::REG_POS_FB_LSB, 2);  // 2 registers, or 2 address units?
  probeRead(w, labrobot::REG_POS_FB_MSB, 1);  // high half on its own
  probeRead(w, labrobot::REG_SPEED_FB, 1);    // speed feedback
  w.resume();
}

static void probeAll() {
  Serial.println(F("---- read-count probe ----"));
  probeLink(F("LEFT  (Serial1)"), g_left);
  probeLink(F("RIGHT (Serial2)"), g_right);
  Serial.println(F("--------------------------"));
}

static void handleCommand(char c) {
  const int16_t v = static_cast<int16_t>(g_test_wheel_rpm);
  switch (c) {
    case 'f': drive(v, v);   Serial.println(F("-> forward")); break;
    case 'b': drive(-v, -v); Serial.println(F("-> reverse")); break;
    // Left turn drives the right wheel forward and the left back.
    case 'l': drive(-v, v);  Serial.println(F("-> spin left")); break;
    case 'r': drive(v, -v);  Serial.println(F("-> spin right")); break;
    case 's': stopMotion(); Serial.println(F("-> brake")); break;
    case 'e':
      g_moving = false;
      g_left.emergencyStop();
      g_right.emergencyStop();
      Serial.println(F("-> E-STOP (motor power cut)"));
      break;
    case 'h':
      g_left.requestSetHome();
      g_right.requestSetHome();
      Serial.println(F("-> encoders zeroed"));
      break;
    case '+':
      if (g_test_wheel_rpm + 5 <= kMaxWheelRpm) g_test_wheel_rpm += 5;
      Serial.print(F("-> speed ")); Serial.println(g_test_wheel_rpm);
      break;
    case '-':
      if (g_test_wheel_rpm >= 10) g_test_wheel_rpm -= 5;
      Serial.print(F("-> speed ")); Serial.println(g_test_wheel_rpm);
      break;
    case '?': printStatus(); break;
    case 'd': dumpBothLinks(); break;
    case 'n':
      g_moving = false;
      g_left.disable();
      g_right.disable();
      Serial.println(F("-> NEUTRAL: motors released, turn a wheel by hand"));
      break;
    case 'p': probeAll(); break;
    default: break;  // ignore stray newlines and unknown keys
  }
}

#else   // LABROBOT_BENCH_MODE

static labrobot::HostLink g_host_link;

#endif  // LABROBOT_BENCH_MODE

void setup() {
#if LABROBOT_BENCH_MODE
  Serial.begin(kConsoleBaud);
#else
  Serial.begin(kHostBaud);
#endif
  Serial1.begin(kMotorBaud);
  Serial2.begin(kMotorBaud);

  g_left.begin(&Serial1, kSlaveAddress, kLinesPerRot, kAcceleration);
  g_right.begin(&Serial2, kSlaveAddress, kLinesPerRot, kAcceleration);
  g_left.setInverted(kInvertLeft);
  g_right.setInverted(kInvertRight);

  // Start braked, never coasting: an unpowered-but-enabled drive lets the base
  // roll freely, and this robot has no parking brake.
  g_left.brake();
  g_right.brake();

#if LABROBOT_BENCH_MODE
  Serial.println();
  Serial.println(F("labrobot firmware - BENCH MODE"));
  Serial.print(F("max "));
  Serial.print(kMaxWheelRpm);
  Serial.print(F(" wheel-rpm = "));
  Serial.print(kMaxBaseRpm);
  Serial.println(F(" base-motor rpm"));
  Serial.println(F("keys: f b l r s +/- h e ? d p n"));
  Serial.println(F("WARNING: no kill switch - wheels clear of the bench."));
#else
  labrobot::HostLink::Config cfg;
  cfg.gear_ratio = kGearRatio;
  cfg.max_wheel_rpm = kMaxWheelRpm;
  cfg.counts_per_wheel_rev = static_cast<uint32_t>(kCountsPerWheelRev);
  // Must exceed the ros2_control period (20 ms at 50 Hz) with margin for USB
  // scheduling jitter, but stay short enough that a dead host stops the robot
  // within a wheel-turn: 0.5 s at 0.26 m/s is 13 cm of travel.
  cfg.command_timeout_ms = 500;
  cfg.status_period_ms = 20;
  g_host_link.begin(&Serial, &g_left, &g_right, cfg);
#endif
}

void loop() {
  // Both drives are pumped every pass. They are on separate UARTs, so their
  // transactions genuinely overlap rather than queueing behind each other.
  g_left.poll();
  g_right.poll();

#if LABROBOT_BENCH_MODE
  while (Serial.available()) {
    handleCommand(static_cast<char>(Serial.read()));
  }

  const uint32_t now = millis();

  if (g_moving && now - g_motion_started_ms >= kAutoBrakeMs) {
    stopMotion();
    Serial.println(F("-> AUTO-BRAKE (no command within timeout)"));
  }

  if (now - g_last_status_ms >= kStatusPeriodMs) {
    g_last_status_ms = now;
    printStatus();
  }
#else
  // Decodes host frames, applies them, publishes status, and brakes if the
  // Jetson goes quiet.
  g_host_link.poll();
#endif
}
