#include "labrobot_hardware/rmcs2303_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace labrobot_hardware
{
namespace
{
constexpr double kTwoPi = 6.283185307179586;

rclcpp::Logger logger() { return rclcpp::get_logger("RMCS2303SystemHardware"); }

// Reads a <param> from the URDF's <hardware> block, falling back to a default.
double param_or(
  const hardware_interface::HardwareInfo & info, const std::string & key, double fallback)
{
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      logger(), "parameter '%s' is not a number ('%s'); using %f", key.c_str(),
      it->second.c_str(), fallback);
    return fallback;
  }
}

}  // namespace

hardware_interface::CallbackReturn RMCS2303SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kNumJoints) {
    RCLCPP_FATAL(
      logger(), "expected exactly %zu joints, got %zu", kNumJoints, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate the interface contract up front. Getting this wrong shows up as a
  // silently motionless robot much later, so fail loudly here instead.
  for (size_t i = 0; i < kNumJoints; ++i) {
    const auto & joint = info_.joints[i];
    joint_names_[i] = joint.name;

    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        logger(), "joint '%s' must have exactly one command interface, '%s'",
        joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    const bool has_position = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & si) { return si.name == hardware_interface::HW_IF_POSITION; });
    const bool has_velocity = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & si) { return si.name == hardware_interface::HW_IF_VELOCITY; });

    if (!has_position || !has_velocity) {
      RCLCPP_FATAL(
        logger(), "joint '%s' must export both position and velocity state interfaces",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const auto port_it = info_.hardware_parameters.find("serial_port");
  serial_device_ = port_it != info_.hardware_parameters.end() ? port_it->second
                                                              : "/dev/labrobot_mcu";
  baud_rate_ = static_cast<uint32_t>(param_or(info_, "baud_rate", 500000));
  counts_per_wheel_rev_ = param_or(info_, "counts_per_wheel_rev", 12532.0);

  const double max_wheel_rpm = param_or(info_, "max_wheel_rpm", 60.0);
  max_wheel_rad_s_ = max_wheel_rpm * kTwoPi / 60.0;

  // Tolerate a few times the firmware's 20 ms status period before declaring
  // the link dead, so ordinary USB scheduling jitter is not treated as a fault.
  status_timeout_s_ = param_or(info_, "status_timeout_ms", 500.0) / 1000.0;
  bootloader_wait_s_ = param_or(info_, "bootloader_wait_ms", 2500.0) / 1000.0;

  if (counts_per_wheel_rev_ <= 0.0) {
    RCLCPP_FATAL(logger(), "counts_per_wheel_rev must be positive");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger(), "labrobot hardware: %s @ %u baud, %.0f counts/rev, max %.2f rad/s",
    serial_device_.c_str(), baud_rate_, counts_per_wheel_rev_, max_wheel_rad_s_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RMCS2303SystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string error;
  if (!port_.open(serial_device_, baud_rate_, error)) {
    RCLCPP_FATAL(logger(), "%s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Opening the port asserted DTR and reset the board. Give the bootloader
  // time to hand over to the sketch before writing anything, then discard the
  // bootloader's own chatter. Sleeping here is safe: on_configure is a
  // lifecycle transition, not the real-time loop.
  RCLCPP_INFO(
    logger(), "opened %s; waiting %.1f s for the MCU bootloader",
    serial_device_.c_str(), bootloader_wait_s_);
  rclcpp::sleep_for(
    std::chrono::milliseconds(static_cast<int64_t>(bootloader_wait_s_ * 1000.0)));
  port_.flush_input();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RMCS2303SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Discard anything queued while inactive: acting on a stale velocity frame
  // would move the robot the instant controllers start.
  port_.flush_input();
  parser_.reset();

  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);

  have_status_ = false;
  offset_captured_ = false;
  have_seq_ = false;
  dropped_frames_ = 0;
  status_frames_ = 0;
  write_failures_ = 0;

  if (!send_simple_command(labrobot::protocol::kCmdStop)) {
    RCLCPP_FATAL(logger(), "cannot write to %s", serial_device_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger(), "activated; waiting for first status frame");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RMCS2303SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop explicitly rather than relying on the firmware watchdog: the watchdog
  // is the backstop for a crash, not the normal path for an orderly shutdown.
  send_simple_command(labrobot::protocol::kCmdStop);
  RCLCPP_INFO(
    logger(), "deactivated after %u status frames (%u dropped)", status_frames_,
    dropped_frames_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RMCS2303SystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  send_simple_command(labrobot::protocol::kCmdStop);
  port_.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RMCS2303SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < kNumJoints; ++i) {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
RMCS2303SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < kNumJoints; ++i) {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return interfaces;
}

bool RMCS2303SystemHardware::send_simple_command(uint8_t type)
{
  uint8_t frame[labrobot::protocol::kMaxFrame];
  const size_t n = labrobot::protocol::encodeCommand(type, frame, sizeof(frame));
  return n != 0 && port_.write(frame, n);
}

void RMCS2303SystemHardware::apply_status(const labrobot::protocol::Status & status)
{
  ++status_frames_;

  // Sequence gaps mean status frames were lost on the USB link. They are not
  // fatal - position is absolute, so a gap costs latency, not distance - but a
  // rising count points at a link problem worth investigating.
  if (have_seq_) {
    const uint8_t expected = static_cast<uint8_t>(last_seq_ + 1);
    if (status.seq != expected) {
      dropped_frames_ += static_cast<uint8_t>(status.seq - expected);
    }
  }
  last_seq_ = status.seq;
  have_seq_ = true;

  const int32_t counts[kNumJoints] = {status.left_counts, status.right_counts};

  if (!offset_captured_) {
    count_offset_[kLeft] = counts[kLeft];
    count_offset_[kRight] = counts[kRight];
    offset_captured_ = true;
  }

  const int16_t mrad[kNumJoints] = {status.left_mrad_s, status.right_mrad_s};
  for (size_t i = 0; i < kNumJoints; ++i) {
    const double rev = static_cast<double>(counts[i] - count_offset_[i]) /
                       counts_per_wheel_rev_;
    hw_positions_[i] = rev * kTwoPi;
    hw_velocities_[i] = static_cast<double>(mrad[i]) / 1000.0;
  }

  if (status.flags & labrobot::protocol::kFlagWatchdogTrip) {
    RCLCPP_WARN_THROTTLE(
      logger(), throttle_clock_, 2000,
      "firmware watchdog tripped: the robot braked itself because commands "
      "stopped arriving");
  }
  if (!(status.flags & labrobot::protocol::kFlagLeftHealthy) ||
      !(status.flags & labrobot::protocol::kFlagRightHealthy))
  {
    RCLCPP_WARN_THROTTLE(
      logger(), throttle_clock_, 2000,
      "drive link unhealthy (modbus faults L=%u R=%u)", status.left_faults,
      status.right_faults);
  }
}

void RMCS2303SystemHardware::drain_serial()
{
  uint8_t buffer[512];
  // Bounded so a backlog cannot make read() run long; anything left over is
  // picked up next cycle. At 50 Hz status this drains far faster than it fills.
  for (int pass = 0; pass < 8; ++pass) {
    const int n = port_.read(buffer, sizeof(buffer));
    if (n <= 0) {
      break;
    }
    for (int i = 0; i < n; ++i) {
      if (parser_.feed(buffer[i]) &&
          parser_.type() == labrobot::protocol::kMsgStatus)
      {
        labrobot::protocol::Status status{};
        if (labrobot::protocol::decodeStatus(
              parser_.payload(), parser_.payloadLen(), &status))
        {
          apply_status(status);
          have_status_ = true;
        }
      }
    }
    if (n < static_cast<int>(sizeof(buffer))) {
      break;
    }
  }
}

hardware_interface::return_type RMCS2303SystemHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  const uint32_t before = status_frames_;
  drain_serial();

  if (status_frames_ != before) {
    last_status_time_ = time;
    // Periodic link health. Dropped frames cost latency, not distance (encoder
    // position is absolute), but a steadily rising rate means the USB link is
    // losing data and is worth chasing before it hides a real fault.
    if ((time - last_health_report_).seconds() >= 10.0) {
      const uint32_t recent_frames = status_frames_ - reported_frames_;
      const uint32_t recent_dropped = dropped_frames_ - reported_dropped_;
      const double recent_pct =
        recent_frames + recent_dropped == 0
          ? 0.0
          : 100.0 * static_cast<double>(recent_dropped) /
              static_cast<double>(recent_frames + recent_dropped);
      RCLCPP_INFO(
        logger(), "link: %u frames (%u total), %u dropped in last window (%.2f%%)",
        recent_frames, status_frames_, recent_dropped, recent_pct);
      reported_frames_ = status_frames_;
      reported_dropped_ = dropped_frames_;
      last_health_report_ = time;
    }
    return hardware_interface::return_type::OK;
  }

  if (!have_status_) {
    // Still waiting for the firmware's first frame after activation. Report OK
    // so controllers can start; write() holds the command at zero until the
    // link is up.
    return hardware_interface::return_type::OK;
  }

  if ((time - last_status_time_).seconds() > status_timeout_s_) {
    RCLCPP_ERROR_THROTTLE(
      logger(), throttle_clock_, 1000,
      "no status frame for %.2f s - link to the MCU appears dead",
      (time - last_status_time_).seconds());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RMCS2303SystemHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Do not command motion before the firmware has been heard from: without a
  // status frame we cannot tell a healthy drive from an absent one.
  const bool link_up = have_status_;

  int16_t mrad[kNumJoints] = {0, 0};
  for (size_t i = 0; i < kNumJoints; ++i) {
    double cmd = hw_commands_[i];
    if (!link_up || !std::isfinite(cmd)) {
      cmd = 0.0;
    }
    // Saturate rather than let an over-range request wrap: the RMCS-2303
    // datasheet warns that an unreachable speed gives an "uncertain result".
    cmd = std::clamp(cmd, -max_wheel_rad_s_, max_wheel_rad_s_);
    mrad[i] = static_cast<int16_t>(std::lround(cmd * 1000.0));
  }

  labrobot::protocol::VelocityCommand cmd{mrad[kLeft], mrad[kRight]};
  uint8_t frame[labrobot::protocol::kMaxFrame];
  const size_t n = labrobot::protocol::encodeVelocity(cmd, frame, sizeof(frame));
  if (n == 0) {
    RCLCPP_ERROR(logger(), "could not encode the velocity frame");
    return hardware_interface::return_type::ERROR;
  }
  if (!port_.write(frame, n)) {
    // Returning ERROR makes ros2_control deactivate this hardware for the rest
    // of the run. On 2026-10-31 ONE failed write, while status frames were
    // still arriving at 50 Hz, left the robot unable to move until a full
    // restart. So skip this frame - the next cycle sends a fresh command, and
    // the firmware watchdog brakes if commands really stop - and give up only
    // when writes keep failing as long as read() tolerates a silent MCU.
    if (write_failures_ == 0) {
      first_write_failure_ = time;
    }
    ++write_failures_;
    const int err = port_.last_errno();
    RCLCPP_WARN_THROTTLE(
      logger(), throttle_clock_, 1000, "serial write failed (%s), %u frame(s) in a row",
      err ? std::strerror(err) : "port not open", write_failures_);
    if ((time - first_write_failure_).seconds() > status_timeout_s_) {
      RCLCPP_ERROR(
        logger(), "serial writes failing for %.2f s (%s) - link to the MCU appears dead",
        (time - first_write_failure_).seconds(), err ? std::strerror(err) : "port not open");
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
  if (write_failures_ > 0) {
    RCLCPP_INFO(
      logger(), "serial write recovered after %u failed frame(s)", write_failures_);
    write_failures_ = 0;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace labrobot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  labrobot_hardware::RMCS2303SystemHardware, hardware_interface::SystemInterface)
