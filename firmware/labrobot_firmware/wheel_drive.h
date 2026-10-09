// One wheel's worth of RMCS-2303 sequencing.
//
// Rmcs2303 handles a single Modbus transaction; this class decides which
// transaction to run next and keeps a cache the host can read at any time.
//
// The Modbus link is the bottleneck (~13-27 transactions/s), so we never waste
// one. Control and speed registers are written only when their value actually
// changes; every other slot goes to reading encoder position. In steady-state
// cruising that means position refreshes at the full link rate.
//
// Encoder position is absolute, so a slow sample rate costs latency but never
// loses distance - which is why odometry stays sound despite the slow link.

#ifndef LABROBOT_WHEEL_DRIVE_H
#define LABROBOT_WHEEL_DRIVE_H

#include "rmcs2303.h"

namespace labrobot {

class WheelDrive {
 public:
  // Consecutive failed transactions before the wheel is reported unhealthy.
  // One-off glitches are normal on a 5 V/3.3 V TTL link; a persistent run of
  // them means the drive is unpowered or unplugged.
  static const uint8_t kFaultThreshold = 5;

  void begin(HardwareSerial* port, uint8_t slave, uint16_t lines_per_rot,
             uint16_t acceleration);

  // Mirror this wheel's sense of rotation.
  //
  // On a differential base the two motors are mounted facing opposite ways, so
  // driving both "clockwise" spins the robot instead of moving it forward. One
  // side must therefore be inverted. This flips the commanded direction bit AND
  // the reported encoder sign together, so everything above this class works in
  // a single convention: positive = this wheel drives the robot forward.
  // Inverting only the command would leave odometry running backwards.
  void setInverted(bool inverted) { inverted_ = inverted; }
  bool inverted() const { return inverted_; }

  // Signed BASE-MOTOR rpm. The drive takes an unsigned magnitude in register 14
  // and its direction from a control bit, so the split happens here. Zero
  // brakes rather than coasting.
  void setTargetRpm(int16_t base_motor_rpm);

  // Brake immediately and refuse further motion until setTargetRpm is called
  // again. Used by the comms watchdog.
  void brake();

  // Cut power to the motor (control 0x0700). Recovery needs re-enabling.
  void emergencyStop();

  // Release the motor so the shaft turns freely. Unlike brake(), which holds
  // position, this lets the wheel be turned by hand - the only way to prove the
  // encoder is mechanically coupled to the wheel rather than reporting a
  // commanded position.
  void disable();

  // Zero the encoder (control 0x0800).
  void requestSetHome();

  void poll();

  int32_t positionCounts() const { return position_counts_; }

  // Wheel velocity in encoder counts per second, differentiated from position.
  //
  // The drive's own speed register is not used: it samples 13 encoder lines
  // over ~10 ms, so its output is quantised to ~465 base-motor rpm - about
  // 1.9 wheel-rpm, or 0.010 m/s. That is far too coarse for odometry. Position
  // is exact, so differentiating it is strictly better, and it frees every
  // Modbus slot for position, doubling the sample rate.
  float velocityCountsPerSec() const { return velocity_cps_; }
  bool healthy() const { return consecutive_faults_ < kFaultThreshold; }
  bool initialised() const { return phase_ != Phase::WRITE_LINES && phase_ != Phase::WRITE_ACCEL; }
  uint16_t timeouts() const { return modbus_.timeouts(); }
  uint16_t crcErrors() const { return modbus_.crcErrors(); }

  // Direct link access, for bench diagnostics only.
  Rmcs2303& link() { return modbus_; }

  // Stop issuing new transactions so a diagnostic can drive the link by hand.
  // poll() still finishes whatever is already in flight, so callers should wait
  // for link().idle() before taking over.
  void suspend() { suspended_ = true; }
  void resume() { suspended_ = false; }
  bool suspended() const { return suspended_; }

  // Raw link diagnostics, for the bench console's 'd' command.
  ParseError lastError() const { return modbus_.lastError(); }
  const char* lastFrame() const { return modbus_.lastFrame(); }
  uint8_t lastFrameLen() const { return modbus_.lastFrameLen(); }
  const char* lastRequest() const { return modbus_.lastRequest(); }

 private:
  enum class Phase : uint8_t {
    WRITE_LINES,  // one-time: encoder lines per rotation
    WRITE_ACCEL,  // one-time: acceleration ramp
    RUNNING,
  };

  // What the current in-flight transaction is doing, so the reply can be
  // routed to the right cache field.
  enum class Action : uint8_t { NONE, CONTROL, SPEED, READ_POS };

  uint16_t desiredControlWord() const;
  // Target in the drive's own sense of rotation, after any inversion.
  int16_t driveTargetRpm() const {
    return inverted_ ? static_cast<int16_t>(-target_rpm_) : target_rpm_;
  }
  void startNextTransaction();

  Rmcs2303 modbus_;
  Phase phase_ = Phase::WRITE_LINES;
  Action action_ = Action::NONE;

  uint16_t lines_per_rot_ = 13;
  uint16_t acceleration_ = 500;

  int16_t target_rpm_ = 0;       // signed, base-motor rpm
  // Position reads happen in SPARE transaction slots, so a fast stream of speed
  // commands can starve them indefinitely - and wheel velocity is differentiated
  // from consecutive position reads, so starving them makes the odometry stale
  // without ever failing. This counter forces a read after at most
  // kMaxWritesBetweenPos writes, bounding feedback staleness no matter how fast
  // the host commands.
  static const uint8_t kMaxWritesBetweenPos = 2;
  uint8_t writes_since_pos_ = 0;

  uint16_t applied_speed_ = 0;   // magnitude last written to register 14
  uint16_t applied_control_ = 0; // last value written to register 2
  bool control_dirty_ = true;
  bool speed_dirty_ = true;
  bool home_pending_ = false;
  bool estopped_ = false;
  bool disabled_ = false;
  bool inverted_ = false;

  int32_t position_counts_ = 0;
  float velocity_cps_ = 0.0f;

  // Baseline for differentiating position. velocity_valid_ suppresses the
  // bogus spike after a set-home, where position steps discontinuously to 0.
  int32_t last_pos_counts_ = 0;
  uint32_t last_pos_ms_ = 0;
  bool velocity_valid_ = false;

  uint8_t consecutive_faults_ = 0;
  bool suspended_ = false;
};

}  // namespace labrobot

#endif  // LABROBOT_WHEEL_DRIVE_H
