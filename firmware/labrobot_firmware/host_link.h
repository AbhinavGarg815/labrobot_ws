// Binds the Jetson wire protocol to the two wheel drives.

#ifndef LABROBOT_HOST_LINK_H
#define LABROBOT_HOST_LINK_H

#include "labrobot_protocol.h"
#include "wheel_drive.h"

namespace labrobot {

class HostLink {
 public:
  struct Config {
    // Brake if no velocity command arrives within this window.
    uint16_t command_timeout_ms = 500;
    uint16_t status_period_ms = 20;  // 50 Hz
    uint8_t frame_gap_ms = 5;
    uint16_t gear_ratio = 241;
    uint16_t max_wheel_rpm = 60;
    uint32_t counts_per_wheel_rev = 12532;
  };

  void begin(HardwareSerial* port, WheelDrive* left, WheelDrive* right,
             const Config& config);

  // Pump the link. Never blocks.
  void poll();

  // Diagnostics.
  bool watchdogTripped() const { return watchdog_tripped_; }
  uint32_t framesReceived() const { return parser_.framesParsed(); }
  uint32_t frameErrors() const { return parser_.errors(); }

  // Unit conversions, exposed for testing.
  int16_t wheelMradSToBaseRpm(int32_t mrad_s) const;
  int16_t countsPerSecToMradS(float counts_per_sec) const;

 private:
  void handleFrame();
  void applyVelocity(const protocol::VelocityCommand& cmd);
  void sendStatus();
  void tripWatchdog();

  HardwareSerial* port_ = nullptr;
  WheelDrive* left_ = nullptr;
  WheelDrive* right_ = nullptr;
  Config cfg_;

  protocol::FrameParser parser_;

  // Single timestamp for the whole poll() call.
  uint32_t now_ms_ = 0;
  uint32_t last_command_ms_ = 0;
  uint32_t last_status_ms_ = 0;
  uint32_t last_byte_ms_ = 0;
  bool watchdog_tripped_ = false;
  bool estopped_ = false;
  bool had_first_command_ = false;
  uint8_t seq_ = 0;
};

}  // namespace labrobot

#endif  // LABROBOT_HOST_LINK_H
