// Binds the Jetson wire protocol to the two wheel drives.
//
// Responsibilities, and nothing else:
//   - decode inbound frames and apply them to the drives
//   - convert between the host's SI-ish units and the drive's base-motor rpm
//   - publish status at a fixed rate
//   - brake if commands stop arriving
//
// Kept separate from the .ino so it can be exercised on the host against a
// simulated drive: the watchdog in particular is safety behaviour, and
// "we'll notice on the bench" is not a test strategy for something whose job is
// to stop a robot that has no kill switch.

#ifndef LABROBOT_HOST_LINK_H
#define LABROBOT_HOST_LINK_H

#include "labrobot_protocol.h"
#include "wheel_drive.h"

namespace labrobot {

class HostLink {
 public:
  struct Config {
    // Brake if no velocity command arrives within this window. Must comfortably
    // exceed the host's control period or normal jitter will trip it.
    uint16_t command_timeout_ms = 500;
    uint16_t status_period_ms = 20;  // 50 Hz
    // Discard a half-received frame after this much silence, so a truncated
    // frame cannot swallow the next one. One frame at 500000 baud is ~0.5 ms.
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

  // Unit conversions, exposed for testing. The host speaks milli-radians per
  // second at the wheel; the drive wants base-motor rpm.
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

  // Single timestamp for the whole poll() call. Re-reading millis() deeper in
  // the call chain let last_command_ms_ end up AHEAD of the loop's `now`, and
  // the unsigned difference then wrapped to ~4.29e9, tripping the watchdog on
  // the very command that should have cleared it.
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
