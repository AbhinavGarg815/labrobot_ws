// Wire format for the Jetson <-> Arduino Mega link.
//
// This header is deliberately free of Arduino and ROS dependencies so that the
// firmware, the host-side tests and the ros2_control plugin all compile against
// ONE definition of the protocol. Two hand-kept copies of a wire format drift,
// and the drift shows up as a robot driving on stale commands.
//
// Frame layout:
//
//   +------+------+-----+------+---------+--------+--------+
//   | 0xA5 | 0x5A | LEN | TYPE | payload | CRC_LO | CRC_HI |
//   +------+------+-----+------+---------+--------+--------+
//
//   LEN  - payload length in bytes, excluding TYPE and the CRC.
//   CRC  - CRC-16/CCITT-FALSE over LEN, TYPE and payload (not the sync bytes).
//
// Every multi-byte field is little-endian and packed explicitly, byte by byte.
// We never memcpy a struct onto the wire: the Mega is 8-bit with different
// alignment and padding rules from aarch64, so a shared struct layout is not
// portable between the two ends of this very link.
//
// Design note: LEN is validated against the length the message type REQUIRES,
// not merely against the buffer. A length check that is correct for one message
// type and merely plausible for another is exactly the bug that made every
// RMCS-2303 speed reply get rejected (see docs/PROJECT_REPORT.md, Problem 5).

#ifndef LABROBOT_PROTOCOL_H
#define LABROBOT_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

namespace labrobot {
namespace protocol {

const uint8_t kSync0 = 0xA5;
const uint8_t kSync1 = 0x5A;

// Longest payload is Status at 18 bytes; the cap leaves room to grow.
const uint8_t kMaxPayload = 32;
const uint8_t kFrameOverhead = 6;  // 2 sync + len + type + 2 crc
const uint8_t kMaxFrame = kMaxPayload + kFrameOverhead;

// Host -> MCU have the high bit clear, MCU -> host have it set, so a frame
// echoed back by a loopback or a crossed cable is rejected on type alone.
enum MessageType : uint8_t {
  kCmdVelocity  = 0x01,  // int16 left_mrad_s, int16 right_mrad_s
  kCmdStop      = 0x02,  // no payload - brake both wheels
  kCmdEstop     = 0x03,  // no payload - cut motor power
  kCmdResetOdom = 0x04,  // no payload - zero both encoders
  kCmdPing      = 0x05,  // no payload

  kMsgStatus    = 0x81,
  kMsgPong      = 0x85,
};

// Status flags.
enum StatusFlag : uint8_t {
  kFlagLeftHealthy   = 1 << 0,
  kFlagRightHealthy  = 1 << 1,
  kFlagInitialised   = 1 << 2,  // both drives finished one-time setup
  kFlagEstopped      = 1 << 3,
  kFlagWatchdogTrip  = 1 << 4,  // braked because commands stopped arriving
};

// Velocities are in milli-radians per second at the WHEEL, signed, positive =
// that wheel drives the robot forward. The mechanical ceiling is 60 wheel-rpm =
// 6283 mrad/s, comfortably inside int16, and 1 mrad/s is ~8x finer than the
// encoder can resolve in a control period - so the unit never limits us.
struct VelocityCommand {
  int16_t left_mrad_s;
  int16_t right_mrad_s;
};

struct Status {
  uint8_t seq;            // wraps; lets the host spot dropped frames
  uint8_t flags;
  int32_t left_counts;
  int32_t right_counts;
  int16_t left_mrad_s;
  int16_t right_mrad_s;
  uint16_t left_faults;   // timeouts + rejected frames on that Modbus link
  uint16_t right_faults;
  // Host-link counters. Without these, "the robot ignored my command" cannot be
  // told apart from "the command never arrived" - the two have entirely
  // different causes and there is no other way to see inbound traffic.
  uint16_t host_frames;   // valid frames accepted from the host
  uint16_t host_errors;   // frames rejected (CRC, length, unknown type)
};

const uint8_t kVelocityPayloadLen = 4;
const uint8_t kStatusPayloadLen = 22;

uint16_t crc16(const uint8_t* data, size_t len);

// Writes a complete frame into `out`. Returns the number of bytes written, or 0
// if the payload is too long or the buffer too small.
size_t encodeFrame(uint8_t type, const uint8_t* payload, uint8_t payload_len,
                   uint8_t* out, size_t out_cap);

size_t encodeVelocity(const VelocityCommand& cmd, uint8_t* out, size_t out_cap);
size_t encodeStatus(const Status& status, uint8_t* out, size_t out_cap);
size_t encodeCommand(uint8_t type, uint8_t* out, size_t out_cap);  // no payload

// Decoders return false if the payload length is wrong for the type.
bool decodeVelocity(const uint8_t* payload, uint8_t len, VelocityCommand* out);
bool decodeStatus(const uint8_t* payload, uint8_t len, Status* out);

// Why a byte stream failed to yield a frame. Same reasoning as the Modbus
// driver: one lumped "bad frame" counter cannot tell a wiring fault from noise.
enum class FrameError : uint8_t {
  kNone,
  kBadCrc,
  kBadLength,     // LEN exceeds kMaxPayload
  kUnknownType,
  kWrongPayloadLen,
};

const char* frameErrorName(FrameError e);

// Streaming frame parser. Feed it one byte at a time.
//
// Recovery guarantee, stated precisely because the obvious stronger claim is
// false: a frame that is TRUNCATED mid-payload leaves the parser expecting more
// payload, so it consumes the following frame's header as payload and fails
// that one too. One truncation therefore costs up to TWO frames, after which
// the parser resynchronises on its own. Corruption that preserves length costs
// only the corrupted frame.
//
// Scanning for the sync pattern while inside a payload would not fix this - a
// payload may legitimately contain 0xA5 0x5A - it would just trade lost frames
// for forged ones. The right fix uses time, not bytes: a caller with a clock
// should call reset() when the inter-byte gap exceeds a frame time, which
// discards the stump before the next frame arrives. The firmware does this;
// this class stays clock-free so it is portable and deterministic to test.
class FrameParser {
 public:
  // Returns true when a complete, CRC-valid, correctly-sized frame is ready.
  bool feed(uint8_t byte);

  uint8_t type() const { return type_; }
  const uint8_t* payload() const { return payload_; }
  uint8_t payloadLen() const { return payload_len_; }

  uint32_t framesParsed() const { return frames_parsed_; }
  uint32_t errors() const { return errors_; }
  FrameError lastError() const { return last_error_; }

  void reset();

 private:
  enum class State : uint8_t { kSync0, kSync1, kLen, kType, kPayload, kCrcLo, kCrcHi };

  bool finish();

  State state_ = State::kSync0;
  uint8_t len_ = 0;
  uint8_t type_ = 0;
  uint8_t payload_[kMaxPayload];
  uint8_t payload_len_ = 0;
  uint8_t index_ = 0;
  uint16_t crc_rx_ = 0;

  uint32_t frames_parsed_ = 0;
  uint32_t errors_ = 0;
  FrameError last_error_ = FrameError::kNone;
};

}  // namespace protocol
}  // namespace labrobot

#endif  // LABROBOT_PROTOCOL_H
