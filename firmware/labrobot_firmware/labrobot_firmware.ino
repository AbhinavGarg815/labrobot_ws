// labrobot motor-control firmware for the Arduino Mega 2560.

// Build mode: 0 = binary protocol to the Jetson, 1 = bench console (flash.sh --bench).
#ifndef LABROBOT_BENCH_MODE
#define LABROBOT_BENCH_MODE 0
#endif

#include "host_link.h"
#include "labrobot_protocol.h"
#include "rmcs2303.h"
#include "wheel_drive.h"

using labrobot::WheelDrive;

// Robot constants. These must agree with labrobot_properties.xacro.

static const uint8_t  kSlaveAddress   = 7;
static const uint16_t kLinesPerRot    = 13;     // RMCS-5030 encoder
static const uint16_t kGearRatio      = 241;    // 1:241
static const uint16_t kAcceleration   = 20000;  // drive reset value (0x4E20)

static const bool kInvertLeft  = true;
static const bool kInvertRight = false;
static const uint16_t kMaxWheelRpm    = 60;     // mechanical ceiling

// 52 quadrature counts per base-motor revolution through the 1:241 gearbox.
static const float kCountsPerWheelRev = 52.0f * 241.0f;  // 12532

static const uint16_t kMaxBaseRpm     = kMaxWheelRpm * kGearRatio;

static const uint32_t kMotorBaud      = 9600;   // fixed in drive firmware
static const uint32_t kConsoleBaud    = 115200;   // bench mode
static const uint32_t kHostBaud       = 500000;   // production mode

static const uint16_t kStatusPeriodMs = 500;

// Bench safety: motion auto-brakes if no further command arrives within this window.
static const uint16_t kAutoBrakeMs = 4000;


static WheelDrive g_left;
static WheelDrive g_right;

#if LABROBOT_BENCH_MODE
static uint16_t g_test_wheel_rpm = 10;  // conservative default
static uint32_t g_last_status_ms = 0;
static uint32_t g_motion_started_ms = 0;
static bool g_moving = false;

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

// Dumps the exact bytes on the wire for one drive.
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

// Runs one read by hand and reports exactly what came back.
static void probeRead(WheelDrive& w, uint8_t addr, uint8_t count) {
  labrobot::Rmcs2303& link = w.link();

  const uint32_t settle = millis();
  while (!link.idle() && millis() - settle < 400) {
    w.poll();
  }
  if (link.complete() || link.failed()) link.acknowledge();

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

// Determines empirically how the drive interprets the read count field.
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
  cfg.command_timeout_ms = 500;
  cfg.status_period_ms = 20;
  g_host_link.begin(&Serial, &g_left, &g_right, cfg);
#endif
}

void loop() {
  // Both drives are pumped every pass.
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
  // Decodes host frames, applies them, publishes status, and brakes if the Jetson goes quiet.
  g_host_link.poll();
#endif
}
