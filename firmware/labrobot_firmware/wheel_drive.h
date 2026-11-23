// One wheel's worth of RMCS-2303 sequencing.

#ifndef LABROBOT_WHEEL_DRIVE_H
#define LABROBOT_WHEEL_DRIVE_H

#include "rmcs2303.h"

namespace labrobot {

class WheelDrive {
 public:
  // Consecutive failed transactions before the wheel is reported unhealthy.
  static const uint8_t kFaultThreshold = 5;

  void begin(HardwareSerial* port, uint8_t slave, uint16_t lines_per_rot,
             uint16_t acceleration);

  // Mirror this wheel's sense of rotation.
  void setInverted(bool inverted) { inverted_ = inverted; }
  bool inverted() const { return inverted_; }

  // Signed BASE-MOTOR rpm.
  void setTargetRpm(int16_t base_motor_rpm);

  // Brake immediately and refuse further motion until setTargetRpm is called again.
  void brake();

  // Cut power to the motor (control 0x0700).
  void emergencyStop();

  // Release the motor so the shaft turns freely.
  void disable();

  // Zero the encoder (control 0x0800).
  void requestSetHome();

  void poll();

  int32_t positionCounts() const { return position_counts_; }

  // Wheel velocity in encoder counts per second, differentiated from position.
  float velocityCountsPerSec() const { return velocity_cps_; }
  bool healthy() const { return consecutive_faults_ < kFaultThreshold; }
  bool initialised() const { return phase_ != Phase::WRITE_LINES && phase_ != Phase::WRITE_ACCEL; }
  uint16_t timeouts() const { return modbus_.timeouts(); }
  uint16_t crcErrors() const { return modbus_.crcErrors(); }

  // Direct link access, for bench diagnostics only.
  Rmcs2303& link() { return modbus_; }

  // Stop issuing new transactions so a diagnostic can drive the link by hand.
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

  // Baseline for differentiating position.
  int32_t last_pos_counts_ = 0;
  uint32_t last_pos_ms_ = 0;
  bool velocity_valid_ = false;

  uint8_t consecutive_faults_ = 0;
  bool suspended_ = false;
};

}  // namespace labrobot

#endif  // LABROBOT_WHEEL_DRIVE_H
