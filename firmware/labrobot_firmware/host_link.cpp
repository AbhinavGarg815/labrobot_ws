#include "host_link.h"

namespace labrobot {

namespace {
// 60 wheel-rpm = 2*pi rad/s = 6283 mrad/s.
const int32_t kMaxWheelMradS = 6283;
}  // namespace

void HostLink::begin(HardwareSerial* port, WheelDrive* left, WheelDrive* right,
                     const Config& config) {
  port_ = port;
  left_ = left;
  right_ = right;
  cfg_ = config;
  parser_.reset();
  last_command_ms_ = millis();
  last_status_ms_ = last_command_ms_;
  last_byte_ms_ = last_command_ms_;
}

int16_t HostLink::wheelMradSToBaseRpm(int32_t mrad_s) const {
  const int32_t max_base = static_cast<int32_t>(cfg_.max_wheel_rpm) * cfg_.gear_ratio;

  // Clamp before scaling.
  if (mrad_s > kMaxWheelMradS) mrad_s = kMaxWheelMradS;
  if (mrad_s < -kMaxWheelMradS) mrad_s = -kMaxWheelMradS;

  // Integer scaling, rounded half away from zero.
  const int32_t num = mrad_s * max_base;
  const int32_t half = kMaxWheelMradS / 2;
  const int32_t rounded = (num >= 0) ? (num + half) / kMaxWheelMradS
                                     : (num - half) / kMaxWheelMradS;
  return static_cast<int16_t>(rounded);
}

int16_t HostLink::countsPerSecToMradS(float counts_per_sec) const {
  // rad/s = (counts/s / counts_per_rev) * 2*pi -> mrad/s scales by 1000.
  const float mrad = counts_per_sec * static_cast<float>(kMaxWheelMradS) /
                     (static_cast<float>(cfg_.counts_per_wheel_rev) *
                      static_cast<float>(cfg_.max_wheel_rpm) / 60.0f);
  if (mrad > 32767.0f) return 32767;
  if (mrad < -32768.0f) return -32768;
  return static_cast<int16_t>(mrad >= 0 ? mrad + 0.5f : mrad - 0.5f);
}

void HostLink::applyVelocity(const protocol::VelocityCommand& cmd) {
  left_->setTargetRpm(wheelMradSToBaseRpm(cmd.left_mrad_s));
  right_->setTargetRpm(wheelMradSToBaseRpm(cmd.right_mrad_s));
  estopped_ = false;
  watchdog_tripped_ = false;
}

void HostLink::tripWatchdog() {
  if (watchdog_tripped_) return;
  watchdog_tripped_ = true;
  left_->brake();
  right_->brake();
}

void HostLink::handleFrame() {
  switch (parser_.type()) {
    case protocol::kCmdVelocity: {
      protocol::VelocityCommand cmd{};
      if (protocol::decodeVelocity(parser_.payload(), parser_.payloadLen(), &cmd)) {
        applyVelocity(cmd);
        last_command_ms_ = now_ms_;
        had_first_command_ = true;
      }
      break;
    }

    case protocol::kCmdStop:
      left_->brake();
      right_->brake();
      last_command_ms_ = now_ms_;
      had_first_command_ = true;
      watchdog_tripped_ = false;
      break;

    case protocol::kCmdEstop:
      estopped_ = true;
      left_->emergencyStop();
      right_->emergencyStop();
      last_command_ms_ = now_ms_;
      break;

    case protocol::kCmdResetOdom:
      left_->requestSetHome();
      right_->requestSetHome();
      last_command_ms_ = now_ms_;
      break;

    case protocol::kCmdPing: {
      uint8_t frame[protocol::kMaxFrame];
      const size_t n = protocol::encodeCommand(protocol::kMsgPong, frame, sizeof(frame));
      if (n) port_->write(frame, n);
      break;
    }

    default:
      // The parser already rejects unknown types; nothing to do.
      break;
  }
}

void HostLink::sendStatus() {
  protocol::Status s{};
  s.seq = seq_++;

  s.flags = 0;
  if (left_->healthy()) s.flags |= protocol::kFlagLeftHealthy;
  if (right_->healthy()) s.flags |= protocol::kFlagRightHealthy;
  if (left_->initialised() && right_->initialised()) {
    s.flags |= protocol::kFlagInitialised;
  }
  if (estopped_) s.flags |= protocol::kFlagEstopped;
  if (watchdog_tripped_) s.flags |= protocol::kFlagWatchdogTrip;

  s.left_counts = left_->positionCounts();
  s.right_counts = right_->positionCounts();
  s.left_mrad_s = countsPerSecToMradS(left_->velocityCountsPerSec());
  s.right_mrad_s = countsPerSecToMradS(right_->velocityCountsPerSec());
  s.left_faults = static_cast<uint16_t>(left_->timeouts() + left_->crcErrors());
  s.right_faults = static_cast<uint16_t>(right_->timeouts() + right_->crcErrors());
  s.host_frames = static_cast<uint16_t>(parser_.framesParsed());
  s.host_errors = static_cast<uint16_t>(parser_.errors());

  uint8_t frame[protocol::kMaxFrame];
  const size_t n = protocol::encodeStatus(s, frame, sizeof(frame));
  if (n) port_->write(frame, n);
}

void HostLink::poll() {
  if (port_ == nullptr) return;

  // Sampled once and used everywhere below, including inside handleFrame().
  const uint32_t now = millis();
  now_ms_ = now;

  if (static_cast<int32_t>(now - last_byte_ms_) >= static_cast<int32_t>(cfg_.frame_gap_ms)) {
    parser_.reset();
  }

  while (port_->available()) {
    last_byte_ms_ = now;
    if (parser_.feed(static_cast<uint8_t>(port_->read()))) {
      handleFrame();
    }
  }

  // Watchdog.
  const int32_t since_command = static_cast<int32_t>(now - last_command_ms_);
  if (had_first_command_ && since_command >= static_cast<int32_t>(cfg_.command_timeout_ms)) {
    tripWatchdog();
  }

  if (static_cast<int32_t>(now - last_status_ms_) >= static_cast<int32_t>(cfg_.status_period_ms)) {
    last_status_ms_ = now;
    sendStatus();
  }
}

}  // namespace labrobot
