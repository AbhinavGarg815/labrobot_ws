#include "labrobot_protocol.h"

namespace labrobot {
namespace protocol {
namespace {

// Little-endian writers. Explicit byte-at-a-time rather than a struct memcpy,
// because this wire crosses an 8-bit AVR and an aarch64 host.
inline void put16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>(v >> 8);
}

inline void put32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

inline uint16_t get16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}

inline uint32_t get32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

// Payload length each type must carry. Returns false for unknown types.
bool requiredPayloadLen(uint8_t type, uint8_t* len) {
  switch (type) {
    case kCmdVelocity:  *len = kVelocityPayloadLen; return true;
    case kCmdStop:
    case kCmdEstop:
    case kCmdResetOdom:
    case kCmdPing:
    case kMsgPong:      *len = 0;                   return true;
    case kMsgStatus:    *len = kStatusPayloadLen;   return true;
    default:                                        return false;
  }
}

}  // namespace

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.
// Bitwise rather than table-driven: 256 entries of PROGMEM would cost more than
// the handful of microseconds this takes on the frames we send.
uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

const char* frameErrorName(FrameError e) {
  switch (e) {
    case FrameError::kNone:            return "none";
    case FrameError::kBadCrc:          return "badcrc";
    case FrameError::kBadLength:       return "badlen";
    case FrameError::kUnknownType:     return "unknowntype";
    case FrameError::kWrongPayloadLen: return "wrongpayloadlen";
  }
  return "?";
}

size_t encodeFrame(uint8_t type, const uint8_t* payload, uint8_t payload_len,
                   uint8_t* out, size_t out_cap) {
  if (payload_len > kMaxPayload) return 0;
  const size_t total = static_cast<size_t>(payload_len) + kFrameOverhead;
  if (out_cap < total) return 0;

  out[0] = kSync0;
  out[1] = kSync1;
  out[2] = payload_len;
  out[3] = type;
  for (uint8_t i = 0; i < payload_len; ++i) out[4 + i] = payload[i];

  // CRC covers LEN, TYPE and payload - not the sync bytes, which are a framing
  // marker rather than content.
  const uint16_t crc = crc16(&out[2], static_cast<size_t>(payload_len) + 2);
  put16(&out[4 + payload_len], crc);
  return total;
}

size_t encodeVelocity(const VelocityCommand& cmd, uint8_t* out, size_t out_cap) {
  uint8_t p[kVelocityPayloadLen];
  put16(&p[0], static_cast<uint16_t>(cmd.left_mrad_s));
  put16(&p[2], static_cast<uint16_t>(cmd.right_mrad_s));
  return encodeFrame(kCmdVelocity, p, kVelocityPayloadLen, out, out_cap);
}

size_t encodeStatus(const Status& s, uint8_t* out, size_t out_cap) {
  uint8_t p[kStatusPayloadLen];
  p[0] = s.seq;
  p[1] = s.flags;
  put32(&p[2], static_cast<uint32_t>(s.left_counts));
  put32(&p[6], static_cast<uint32_t>(s.right_counts));
  put16(&p[10], static_cast<uint16_t>(s.left_mrad_s));
  put16(&p[12], static_cast<uint16_t>(s.right_mrad_s));
  put16(&p[14], s.left_faults);
  put16(&p[16], s.right_faults);
  put16(&p[18], s.host_frames);
  put16(&p[20], s.host_errors);
  return encodeFrame(kMsgStatus, p, kStatusPayloadLen, out, out_cap);
}

size_t encodeCommand(uint8_t type, uint8_t* out, size_t out_cap) {
  return encodeFrame(type, nullptr, 0, out, out_cap);
}

bool decodeVelocity(const uint8_t* payload, uint8_t len, VelocityCommand* out) {
  if (len != kVelocityPayloadLen || out == nullptr) return false;
  out->left_mrad_s = static_cast<int16_t>(get16(&payload[0]));
  out->right_mrad_s = static_cast<int16_t>(get16(&payload[2]));
  return true;
}

bool decodeStatus(const uint8_t* payload, uint8_t len, Status* out) {
  if (len != kStatusPayloadLen || out == nullptr) return false;
  out->seq = payload[0];
  out->flags = payload[1];
  out->left_counts = static_cast<int32_t>(get32(&payload[2]));
  out->right_counts = static_cast<int32_t>(get32(&payload[6]));
  out->left_mrad_s = static_cast<int16_t>(get16(&payload[10]));
  out->right_mrad_s = static_cast<int16_t>(get16(&payload[12]));
  out->left_faults = get16(&payload[14]);
  out->right_faults = get16(&payload[16]);
  out->host_frames = get16(&payload[18]);
  out->host_errors = get16(&payload[20]);
  return true;
}

void FrameParser::reset() {
  state_ = State::kSync0;
  index_ = 0;
  payload_len_ = 0;
}

bool FrameParser::finish() {
  // Recompute over LEN, TYPE, payload exactly as the encoder did.
  uint8_t buf[kMaxPayload + 2];
  buf[0] = len_;
  buf[1] = type_;
  for (uint8_t i = 0; i < len_; ++i) buf[2 + i] = payload_[i];
  const uint16_t crc_calc = crc16(buf, static_cast<size_t>(len_) + 2);

  state_ = State::kSync0;

  if (crc_calc != crc_rx_) {
    ++errors_;
    last_error_ = FrameError::kBadCrc;
    return false;
  }

  // Only accept a length that is right for THIS type. Accepting any length the
  // buffer can hold is how a decoder ends up reading a shorter message as if it
  // were a longer one.
  uint8_t required = 0;
  if (!requiredPayloadLen(type_, &required)) {
    ++errors_;
    last_error_ = FrameError::kUnknownType;
    return false;
  }
  if (required != len_) {
    ++errors_;
    last_error_ = FrameError::kWrongPayloadLen;
    return false;
  }

  payload_len_ = len_;
  ++frames_parsed_;
  last_error_ = FrameError::kNone;
  return true;
}

bool FrameParser::feed(uint8_t byte) {
  switch (state_) {
    case State::kSync0:
      if (byte == kSync0) state_ = State::kSync1;
      return false;

    case State::kSync1:
      // A second 0xA5 means the first was noise and this may be the real start.
      if (byte == kSync1) {
        state_ = State::kLen;
      } else {
        state_ = (byte == kSync0) ? State::kSync1 : State::kSync0;
      }
      return false;

    case State::kLen:
      if (byte > kMaxPayload) {
        ++errors_;
        last_error_ = FrameError::kBadLength;
        state_ = State::kSync0;
        return false;
      }
      len_ = byte;
      state_ = State::kType;
      return false;

    case State::kType:
      type_ = byte;
      index_ = 0;
      state_ = (len_ == 0) ? State::kCrcLo : State::kPayload;
      return false;

    case State::kPayload:
      payload_[index_++] = byte;
      if (index_ >= len_) state_ = State::kCrcLo;
      return false;

    case State::kCrcLo:
      crc_rx_ = byte;
      state_ = State::kCrcHi;
      return false;

    case State::kCrcHi:
      crc_rx_ |= static_cast<uint16_t>(byte) << 8;
      return finish();
  }
  return false;
}

}  // namespace protocol
}  // namespace labrobot
