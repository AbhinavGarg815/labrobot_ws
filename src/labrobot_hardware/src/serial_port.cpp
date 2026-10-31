#include "labrobot_hardware/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <map>

namespace labrobot_hardware
{
namespace
{

// Only the rates we actually use. 500000 is exact on a 16 MHz ATmega2560
// (U2X, UBRR=3, 0% error); 460800 and 921600 carry several percent of error
// there and corrupt frames, so they are deliberately absent.
speed_t to_speed(uint32_t baud)
{
  static const std::map<uint32_t, speed_t> kRates = {
    {9600, B9600}, {19200, B19200}, {38400, B38400}, {57600, B57600},
    {115200, B115200}, {230400, B230400}, {500000, B500000}, {1000000, B1000000},
  };
  const auto it = kRates.find(baud);
  return it == kRates.end() ? B0 : it->second;
}

}  // namespace

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string & device, uint32_t baud, std::string & error)
{
  close();

  const speed_t speed = to_speed(baud);
  if (speed == B0) {
    error = "unsupported baud rate " + std::to_string(baud);
    return false;
  }

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    error = "cannot open " + device + ": " + std::strerror(errno);
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    error = "tcgetattr failed: " + std::string(std::strerror(errno));
    close();
    return false;
  }

  // Raw mode: no canonical processing, echo, flow control or character mapping.
  // Anything less and the driver would eat or translate bytes of our frames.
  cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD;   // ignore modem lines, enable receiver
  tty.c_cflag &= ~CRTSCTS;         // no hardware flow control
  tty.c_cflag &= ~CSTOPB;          // one stop bit
  tty.c_cflag &= ~PARENB;          // no parity
  tty.c_cc[VMIN] = 0;              // fully non-blocking: return immediately
  tty.c_cc[VTIME] = 0;

  if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
    error = "cannot set baud rate";
    close();
    return false;
  }

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    error = "tcsetattr failed: " + std::string(std::strerror(errno));
    close();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

int SerialPort::read(uint8_t * buffer, size_t max)
{
  if (fd_ < 0) {
    return -1;
  }
  const ssize_t n = ::read(fd_, buffer, max);
  if (n < 0) {
    // Nothing buffered is the normal case on a non-blocking port, not an error.
    return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
  }
  return static_cast<int>(n);
}

bool SerialPort::write(const uint8_t * data, size_t len)
{
  last_errno_ = 0;
  if (fd_ < 0) {
    return false;
  }
  size_t written = 0;
  while (written < len) {
    const ssize_t n = ::write(fd_, data + written, len - written);
    if (n < 0) {
      // Never retry here: spinning would stall the whole controller loop. The
      // caller decides whether a failure is transient (see write() in
      // rmcs2303_system.cpp).
      last_errno_ = errno;
      return false;
    }
    written += static_cast<size_t>(n);
  }
  return true;
}

void SerialPort::flush_input()
{
  if (fd_ >= 0) {
    tcflush(fd_, TCIFLUSH);
  }
}

}  // namespace labrobot_hardware
