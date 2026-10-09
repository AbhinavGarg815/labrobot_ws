#include "rmcs2303.h"

namespace labrobot {
namespace {

inline char nibbleToHex(uint8_t n) {
  return static_cast<char>(n < 10 ? '0' + n : 'A' + (n - 10));
}

// Returns 0xFF for a non-hex character so callers can reject the frame.
inline uint8_t hexToNibble(char c) {
  if (c >= '0' && c <= '9') return static_cast<uint8_t>(c - '0');
  if (c >= 'A' && c <= 'F') return static_cast<uint8_t>(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return static_cast<uint8_t>(c - 'a' + 10);
  return 0xFF;
}

// Parses two hex chars into a byte. Sets ok=false on any invalid character;
// ok is never set back to true, so a single bad nibble poisons the whole frame.
inline uint8_t hexByte(const char* p, bool& ok) {
  const uint8_t hi = hexToNibble(p[0]);
  const uint8_t lo = hexToNibble(p[1]);
  if (hi == 0xFF || lo == 0xFF) {
    ok = false;
    return 0;
  }
  return static_cast<uint8_t>((hi << 4) | lo);
}

}  // namespace

const char* parseErrorName(ParseError e) {
  switch (e) {
    case ParseError::NONE:        return "none";
    case ParseError::TIMEOUT:     return "timeout";
    case ParseError::TOO_SHORT:   return "short";
    case ParseError::ODD_LENGTH:  return "oddlen";
    case ParseError::BAD_HEX:     return "badhex";
    case ParseError::BAD_LRC:     return "badlrc";
    case ParseError::WRONG_SLAVE: return "wrongslave";
    case ParseError::WRONG_FC:    return "wrongfc";
    case ParseError::BAD_COUNT:   return "badcount";
  }
  return "?";
}

void Rmcs2303::fail(ParseError reason) {
  ++crc_errors_;
  last_error_ = reason;
  state_ = TxnState::FAILED;
}

void Rmcs2303::begin(HardwareSerial* port, uint8_t slave_addr) {
  port_ = port;
  slave_ = slave_addr;
  state_ = TxnState::IDLE;
  rx_len_ = 0;
}

void Rmcs2303::sendFrame(uint8_t fc, uint8_t addr, uint16_t value) {
  // Payload bytes the LRC is computed over: slave, function code, 16-bit
  // address, 16-bit data. The RMCS-2303 uses an 8-bit address space, so the
  // high address byte is always zero.
  const uint8_t payload[6] = {
      slave_,
      fc,
      0x00,
      addr,
      static_cast<uint8_t>(value >> 8),
      static_cast<uint8_t>(value & 0xFF),
  };

  uint8_t sum = 0;
  char frame[18];
  uint8_t n = 0;
  frame[n++] = ':';
  for (uint8_t i = 0; i < 6; ++i) {
    sum = static_cast<uint8_t>(sum + payload[i]);
    frame[n++] = nibbleToHex(static_cast<uint8_t>(payload[i] >> 4));
    frame[n++] = nibbleToHex(static_cast<uint8_t>(payload[i] & 0x0F));
  }

  // LRC is the two's complement of the running sum.
  const uint8_t lrc = static_cast<uint8_t>(-static_cast<int8_t>(sum));
  frame[n++] = nibbleToHex(static_cast<uint8_t>(lrc >> 4));
  frame[n++] = nibbleToHex(static_cast<uint8_t>(lrc & 0x0F));
  frame[n++] = '\r';
  frame[n++] = '\n';

  // Drop anything stale still sitting in the receive buffer, so a late reply
  // from a previous timed-out transaction cannot be mistaken for this one's.
  while (port_->available()) {
    port_->read();
  }

  const uint8_t keep = n < sizeof(pending_request_) - 1
                           ? n
                           : static_cast<uint8_t>(sizeof(pending_request_) - 1);
  memcpy(pending_request_, frame, keep);
  pending_request_[keep] = '\0';

  port_->write(reinterpret_cast<const uint8_t*>(frame), n);

  rx_len_ = 0;
  resp_reg_count_ = 0;
  expected_fc_ = fc;
  sent_at_ms_ = millis();
  state_ = TxnState::AWAITING;
}

bool Rmcs2303::beginWrite(uint8_t reg_addr, uint16_t value) {
  if (state_ != TxnState::IDLE || port_ == nullptr) return false;
  sendFrame(0x06, reg_addr, value);
  return true;
}

bool Rmcs2303::beginRead(uint8_t reg_addr, uint8_t reg_count) {
  if (state_ != TxnState::IDLE || port_ == nullptr) return false;
  if (reg_count == 0 || reg_count > kMaxResponseRegs) return false;
  sendFrame(0x03, reg_addr, reg_count);
  return true;
}

void Rmcs2303::poll() {
  if (state_ != TxnState::AWAITING || port_ == nullptr) return;

  while (port_->available()) {
    const char c = static_cast<char>(port_->read());

    // Resynchronise on ':' - it can only start a frame, so seeing one mid-way
    // means we lost bytes and should restart rather than parse garbage.
    if (c == ':') {
      rx_len_ = 0;
      rx_[rx_len_++] = c;
      continue;
    }
    if (rx_len_ == 0) continue;  // no frame start seen yet; ignore noise

    if (c == '\n') {
      parseFrame();
      return;
    }
    if (c == '\r') continue;

    if (rx_len_ < kRxBufferSize) {
      rx_[rx_len_++] = c;
    } else {
      // Overlong frame: discard and wait for the next ':'.
      rx_len_ = 0;
    }
  }

  if (millis() - sent_at_ms_ >= kTimeoutMs) {
    ++timeouts_;
    last_error_ = ParseError::TIMEOUT;
    last_frame_len_ = 0;
    last_frame_[0] = '\0';
    memcpy(last_request_, pending_request_, sizeof(last_request_));
    state_ = TxnState::FAILED;
  }
}

void Rmcs2303::parseFrame() {
  // Snapshot the raw reply before any validation, so a rejected frame can
  // still be dumped for diagnosis.
  last_frame_len_ = rx_len_ < kRxBufferSize ? rx_len_ : kRxBufferSize;
  memcpy(last_frame_, rx_, last_frame_len_);
  last_frame_[last_frame_len_] = '\0';
  memcpy(last_request_, pending_request_, sizeof(last_request_));

  // Shortest valid reply is a single-register read:
  //   ':' slave(2) fc(2) bytecount(2) data(4) lrc(2) = 13 chars, CR/LF stripped.
  // A write echo is longer at 15. Using 15 as the floor here silently rejected
  // every real read reply from the drive as "short".
  if (rx_len_ < 13 || rx_[0] != ':') {
    fail(ParseError::TOO_SHORT);
    return;
  }

  // Everything after ':' must be hex pairs.
  const uint8_t hex_chars = static_cast<uint8_t>(rx_len_ - 1);
  if (hex_chars % 2 != 0) {
    fail(ParseError::ODD_LENGTH);
    return;
  }
  const uint8_t n_bytes = static_cast<uint8_t>(hex_chars / 2);

  bool ok = true;
  uint8_t bytes[kRxBufferSize / 2];
  for (uint8_t i = 0; i < n_bytes; ++i) {
    bytes[i] = hexByte(&rx_[1 + i * 2], ok);
  }
  if (!ok) {
    fail(ParseError::BAD_HEX);
    return;
  }

  // Verify the LRC over everything except the trailing LRC byte itself.
  uint8_t sum = 0;
  for (uint8_t i = 0; i < n_bytes - 1; ++i) {
    sum = static_cast<uint8_t>(sum + bytes[i]);
  }
  const uint8_t expected_lrc = static_cast<uint8_t>(-static_cast<int8_t>(sum));
  if (expected_lrc != bytes[n_bytes - 1]) {
    fail(ParseError::BAD_LRC);
    return;
  }

  // A reply from a different slave, or to a different function, is not ours.
  // The drive also signals errors by setting the high bit of the function code.
  if (bytes[0] != slave_) {
    fail(ParseError::WRONG_SLAVE);
    return;
  }
  if (bytes[1] != expected_fc_) {
    fail(ParseError::WRONG_FC);
    return;
  }

  if (expected_fc_ == 0x03) {
    // ':' slave fc bytecount data... lrc
    const uint8_t byte_count = bytes[2];
    // A zero byte count means no payload: treat it as malformed rather than
    // silently reporting a position of 0, which would look like a working
    // encoder that never moves.
    if (byte_count == 0 || byte_count % 2 != 0 ||
        byte_count / 2 > kMaxResponseRegs || n_bytes < 3 + byte_count + 1) {
      fail(ParseError::BAD_COUNT);
      return;
    }
    resp_reg_count_ = static_cast<uint8_t>(byte_count / 2);
    for (uint8_t i = 0; i < resp_reg_count_; ++i) {
      resp_regs_[i] = static_cast<uint16_t>(
          (static_cast<uint16_t>(bytes[3 + i * 2]) << 8) | bytes[4 + i * 2]);
    }
  } else {
    // Write replies echo the request; no payload to extract.
    resp_reg_count_ = 0;
  }

  last_error_ = ParseError::NONE;
  state_ = TxnState::COMPLETE;
}

uint16_t Rmcs2303::responseRegister(uint8_t index) const {
  if (index >= resp_reg_count_) return 0;
  return resp_regs_[index];
}

}  // namespace labrobot
