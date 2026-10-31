// ros2_control SystemInterface for the labrobot drivetrain.
//
// The Jetson never speaks MODBUS. It exchanges framed binary messages with the
// Arduino Mega, which owns the two RMCS-2303 links and their 9600-baud timing.
// That split is deliberate: a Modbus transaction takes ~37 ms, far longer than
// a control period, so the Mega runs it asynchronously and caches the result
// for this class to read at loop rate.
//
// The wire format comes from the firmware's own header, so there is exactly one
// definition of the protocol shared by both ends.

#ifndef LABROBOT_HARDWARE__RMCS2303_SYSTEM_HPP_
#define LABROBOT_HARDWARE__RMCS2303_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "labrobot_hardware/serial_port.hpp"
#include "labrobot_protocol.h"
// clock.hpp and time.hpp are needed for the Clock and Time MEMBERS below, not
// just for their use in the .cpp: an incomplete rclcpp::Clock here makes the
// member fail to form, and the RCLCPP_*_THROTTLE macros then fail deep inside
// their own lambda capture with an error that names neither this file nor Clock.
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace labrobot_hardware
{

class RMCS2303SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RMCS2303SystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr size_t kLeft = 0;
  static constexpr size_t kRight = 1;
  static constexpr size_t kNumJoints = 2;

  bool send_simple_command(uint8_t type);
  void drain_serial();
  void apply_status(const labrobot::protocol::Status & status);

  // --- configuration, from the <hardware> params in the URDF ---------------
  std::string serial_device_;
  uint32_t baud_rate_ = 500000;
  double counts_per_wheel_rev_ = 12532.0;
  double max_wheel_rad_s_ = 6.283185307179586;
  // How long a lack of status frames is tolerated before read() reports an
  // error. The firmware publishes at 50 Hz, so this is many missed frames.
  double status_timeout_s_ = 0.5;
  // Opening the port asserts DTR, which RESETS the Arduino into its bootloader.
  // Anything written before the bootloader hands over is either swallowed or
  // mistaken for an upload attempt, and the sketch may not start streaming at
  // all. host_client.py waits 2.5 s for exactly this reason; the plugin must
  // too, or first contact is a race it usually loses.
  double bootloader_wait_s_ = 2.5;

  // --- ros2_control storage -------------------------------------------------
  // Parenthesised construction, deliberately. Brace-init on vector<double>
  // prefers the initializer_list constructor, so {kNumJoints, 0.0} would build
  // the two VALUES {2.0, 0.0} rather than two zeros - a left wheel that starts
  // life reporting 2 radians.
  std::vector<std::string> joint_names_ = std::vector<std::string>(kNumJoints);
  std::vector<double> hw_positions_ = std::vector<double>(kNumJoints, 0.0);
  std::vector<double> hw_velocities_ = std::vector<double>(kNumJoints, 0.0);
  std::vector<double> hw_commands_ = std::vector<double>(kNumJoints, 0.0);

  // --- link state -----------------------------------------------------------
  // Owned clock for throttled logging. Passing *rclcpp::Clock::make_shared()
  // instead hands the macro a reference to a temporary whose shared_ptr deleter
  // runs before the throttle logic calls now() - a use-after-free that segfaults
  // the whole controller_manager on the first warning.
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

  SerialPort port_;
  labrobot::protocol::FrameParser parser_;

  bool have_status_ = false;
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_health_report_{0, 0, RCL_ROS_TIME};

  // Encoder counts observed at activation. Reported joint positions are
  // relative to this, so joint_states start near zero on every run instead of
  // inheriting whatever the drives happened to hold.
  int32_t count_offset_[kNumJoints] = {0, 0};
  bool offset_captured_ = false;

  uint8_t last_seq_ = 0;
  bool have_seq_ = false;
  uint32_t dropped_frames_ = 0;
  uint32_t status_frames_ = 0;
  // Consecutive failed command writes, and when the run of failures began.
  uint32_t write_failures_ = 0;
  rclcpp::Time first_write_failure_{0, 0, RCL_ROS_TIME};
  // Snapshot at the last health report, so the log can show drops SINCE then.
  // A cumulative percentage is dominated by the startup burst and would hide a
  // link that started failing an hour into a mission.
  uint32_t reported_dropped_ = 0;
  uint32_t reported_frames_ = 0;
};

}  // namespace labrobot_hardware

#endif  // LABROBOT_HARDWARE__RMCS2303_SYSTEM_HPP_
