// Non-blocking MODBUS ASCII master for the Rhino RMCS-2303 DC servo drive.

#ifndef LABROBOT_RMCS2303_H
#define LABROBOT_RMCS2303_H

#include <Arduino.h>

namespace labrobot {

// Register data addresses.
enum Rmcs2303Register : uint8_t {
  REG_EEPROM_SLAVE   = 0,   // write 0xNNFF to store params / set slave id NN
  REG_CONTROL        = 2,   // MODE[15:8] | CTRL[7:0]
  REG_POS_P_GAIN     = 4,
  REG_POS_I_GAIN     = 6,
  REG_VEL_FF_GAIN    = 8,
  REG_LINES_PER_ROT  = 10,  // 13 for the RMCS-5030; speed feedback scales off it
  REG_ACCELERATION   = 12,
  REG_SPEED_CMD      = 14,  // BASE-MOTOR rpm, unsigned; direction is a CTRL bit
  REG_POS_CMD_LSB    = 16,
  REG_POS_CMD_MSB    = 18,
  REG_POS_FB_LSB     = 20,  // 32-bit position, read as one 2-register txn
  REG_POS_FB_MSB     = 22,
  REG_SPEED_FB       = 24,  // base-motor rpm
};

// Values for REG_CONTROL.
enum Rmcs2303Control : uint16_t {
  CTRL_ENABLE_CW    = 0x0101,
  CTRL_ENABLE_CCW   = 0x0109,
  CTRL_DISABLE_CW   = 0x0100,
  CTRL_DISABLE_CCW  = 0x0108,
  CTRL_BRAKE_CW     = 0x0104,
  CTRL_BRAKE_CCW    = 0x010C,
  CTRL_ESTOP        = 0x0700,  // cuts power to the motor
  CTRL_STOP_AND_HOLD= 0x0701,
  CTRL_SET_HOME     = 0x0800,  // zeroes the encoder
  CTRL_RESTART      = 0x0900,
};

enum class TxnState : uint8_t {
  IDLE,      // nothing in flight, ready for a new request
  AWAITING,  // request sent, collecting the reply
  COMPLETE,  // reply parsed successfully; result readable
  FAILED,    // timeout or malformed reply; retryable
};

// Why the last transaction was rejected.
enum class ParseError : uint8_t {
  NONE,
  TIMEOUT,      // no reply at all
  TOO_SHORT,    // terminator arrived before a full frame
  ODD_LENGTH,   // not a whole number of hex pairs
  BAD_HEX,      // non-hex character inside the frame
  BAD_LRC,      // checksum mismatch - line noise or wrong baud
  WRONG_SLAVE,  // a different drive answered
  WRONG_FC,     // function code mismatch, or an exception reply
  BAD_COUNT,    // byte count inconsistent with the frame
};

const char* parseErrorName(ParseError e);

class Rmcs2303 {
 public:
  // The drive's reply must arrive within this window.
  static const uint16_t kTimeoutMs = 120;

  void begin(HardwareSerial* port, uint8_t slave_addr);

  // Pump the receive state machine.
  void poll();

  bool idle() const { return state_ == TxnState::IDLE; }
  bool complete() const { return state_ == TxnState::COMPLETE; }
  bool failed() const { return state_ == TxnState::FAILED; }

  // Clear a COMPLETE/FAILED result and return to IDLE.
  void acknowledge() { state_ = TxnState::IDLE; }

  // Start a transaction. Returns false if one is already in flight.
  bool beginWrite(uint8_t reg_addr, uint16_t value);
  bool beginRead(uint8_t reg_addr, uint8_t reg_count);

  // Valid only while complete().
  uint16_t responseRegister(uint8_t index) const;
  uint8_t responseCount() const { return resp_reg_count_; }

  // Diagnostics.
  uint16_t timeouts() const { return timeouts_; }
  uint16_t crcErrors() const { return crc_errors_; }
  ParseError lastError() const { return last_error_; }

  // The most recent reply as received, valid or not, NUL-terminated.
  const char* lastFrame() const { return last_frame_; }
  uint8_t lastFrameLen() const { return last_frame_len_; }

  // The request that produced lastFrame(), NUL-terminated.
  const char* lastRequest() const { return last_request_; }

 private:
  static const uint8_t kRxBufferSize = 48;
  static const uint8_t kMaxResponseRegs = 2;

  void sendFrame(uint8_t fc, uint8_t addr, uint16_t value);
  void parseFrame();
  void fail(ParseError reason);

  HardwareSerial* port_ = nullptr;
  uint8_t slave_ = 7;

  TxnState state_ = TxnState::IDLE;
  uint32_t sent_at_ms_ = 0;
  uint8_t expected_fc_ = 0;

  char rx_[kRxBufferSize];
  uint8_t rx_len_ = 0;

  uint16_t resp_regs_[kMaxResponseRegs] = {0, 0};
  uint8_t resp_reg_count_ = 0;

  uint16_t timeouts_ = 0;
  uint16_t crc_errors_ = 0;
  ParseError last_error_ = ParseError::NONE;

  char last_frame_[kRxBufferSize + 1] = {0};
  uint8_t last_frame_len_ = 0;
  char last_request_[20] = {0};   // paired with last_frame_
  char pending_request_[20] = {0};  // in flight, promoted on parse/timeout
};

}  // namespace labrobot

#endif  // LABROBOT_RMCS2303_H
