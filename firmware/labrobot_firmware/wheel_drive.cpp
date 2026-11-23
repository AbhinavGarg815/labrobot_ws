#include "wheel_drive.h"

namespace labrobot {

void WheelDrive::begin(HardwareSerial* port, uint8_t slave,
                       uint16_t lines_per_rot, uint16_t acceleration) {
  modbus_.begin(port, slave);
  lines_per_rot_ = lines_per_rot;
  acceleration_ = acceleration;
  phase_ = Phase::WRITE_LINES;
  action_ = Action::NONE;
  control_dirty_ = true;
  speed_dirty_ = true;
}

uint16_t WheelDrive::desiredControlWord() const {
  if (estopped_) return CTRL_ESTOP;
  if (disabled_) return CTRL_DISABLE_CW;

  if (target_rpm_ == 0) {
    return applied_control_ == CTRL_ENABLE_CCW || applied_control_ == CTRL_BRAKE_CCW
               ? CTRL_BRAKE_CCW
               : CTRL_BRAKE_CW;
  }
  return driveTargetRpm() > 0 ? CTRL_ENABLE_CW : CTRL_ENABLE_CCW;
}

void WheelDrive::setTargetRpm(int16_t base_motor_rpm) {
  estopped_ = false;
  disabled_ = false;

  if (base_motor_rpm != target_rpm_) {
    target_rpm_ = base_motor_rpm;

    const uint16_t magnitude =
        static_cast<uint16_t>(base_motor_rpm < 0 ? -base_motor_rpm : base_motor_rpm);
    if (magnitude != applied_speed_) speed_dirty_ = true;
    if (desiredControlWord() != applied_control_) control_dirty_ = true;
  }
}

void WheelDrive::disable() {
  disabled_ = true;
  target_rpm_ = 0;
  control_dirty_ = true;
  speed_dirty_ = true;
}

void WheelDrive::brake() {
  disabled_ = false;
  target_rpm_ = 0;
  if (applied_speed_ != 0) speed_dirty_ = true;
  if (desiredControlWord() != applied_control_) control_dirty_ = true;
}

void WheelDrive::emergencyStop() {
  estopped_ = true;
  target_rpm_ = 0;
  control_dirty_ = true;
  speed_dirty_ = true;
}

void WheelDrive::requestSetHome() {
  home_pending_ = true;
  velocity_valid_ = false;
  velocity_cps_ = 0.0f;
}

void WheelDrive::startNextTransaction() {
  switch (phase_) {
    case Phase::WRITE_LINES:
      if (modbus_.beginWrite(REG_LINES_PER_ROT, lines_per_rot_)) {
        action_ = Action::CONTROL;  // reply carries no payload; phase advances
      }
      return;

    case Phase::WRITE_ACCEL:
      if (modbus_.beginWrite(REG_ACCELERATION, acceleration_)) {
        action_ = Action::CONTROL;
      }
      return;

    case Phase::RUNNING:
      break;
  }

  if (home_pending_) {
    if (modbus_.beginWrite(REG_CONTROL, CTRL_SET_HOME)) {
      action_ = Action::CONTROL;
      home_pending_ = false;
      // Re-assert the real control word next, since we just overwrote it.
      control_dirty_ = true;
    }
    return;
  }

  if (control_dirty_) {
    const uint16_t word = desiredControlWord();
    if (modbus_.beginWrite(REG_CONTROL, word)) {
      applied_control_ = word;
      control_dirty_ = false;
      action_ = Action::CONTROL;
      if (writes_since_pos_ < 255) ++writes_since_pos_;
    }
    return;
  }

  if (writes_since_pos_ >= kMaxWritesBetweenPos) {
    if (modbus_.beginRead(REG_POS_FB_LSB, 2)) {
      action_ = Action::READ_POS;
      writes_since_pos_ = 0;
    }
    return;
  }

  if (speed_dirty_) {
    const uint16_t magnitude =
        static_cast<uint16_t>(target_rpm_ < 0 ? -target_rpm_ : target_rpm_);
    if (modbus_.beginWrite(REG_SPEED_CMD, magnitude)) {
      applied_speed_ = magnitude;
      speed_dirty_ = false;
      action_ = Action::SPEED;
      if (writes_since_pos_ < 255) ++writes_since_pos_;
    }
    return;
  }

  // Nothing to write, so spend the slot on position.
  if (modbus_.beginRead(REG_POS_FB_LSB, 2)) {
    action_ = Action::READ_POS;
    writes_since_pos_ = 0;
  }
}

void WheelDrive::poll() {
  modbus_.poll();

  if (modbus_.complete()) {
    consecutive_faults_ = 0;

    switch (action_) {
      case Action::READ_POS: {
        // Registers arrive LSB (addr 20) then MSB (addr 22).
        const uint32_t raw =
            (static_cast<uint32_t>(modbus_.responseRegister(1)) << 16) |
            modbus_.responseRegister(0);
        const int32_t signed_raw = static_cast<int32_t>(raw);
        position_counts_ = inverted_ ? -signed_raw : signed_raw;

        const uint32_t now = millis();
        if (velocity_valid_ && now != last_pos_ms_) {
          const float dt = (now - last_pos_ms_) * 0.001f;
          velocity_cps_ =
              static_cast<float>(position_counts_ - last_pos_counts_) / dt;
        }
        last_pos_counts_ = position_counts_;
        last_pos_ms_ = now;
        velocity_valid_ = true;
        break;
      }
      default:
        break;
    }

    // Advance one-time setup phases once their write is acknowledged.
    if (phase_ == Phase::WRITE_LINES) {
      phase_ = Phase::WRITE_ACCEL;
    } else if (phase_ == Phase::WRITE_ACCEL) {
      phase_ = Phase::RUNNING;
    }

    modbus_.acknowledge();
    action_ = Action::NONE;
  } else if (modbus_.failed()) {
    if (consecutive_faults_ < 255) ++consecutive_faults_;

    if (action_ == Action::CONTROL) control_dirty_ = true;
    if (action_ == Action::SPEED) speed_dirty_ = true;

    modbus_.acknowledge();
    action_ = Action::NONE;
  }

  if (modbus_.idle() && !suspended_) startNextTransaction();
}

}  // namespace labrobot
