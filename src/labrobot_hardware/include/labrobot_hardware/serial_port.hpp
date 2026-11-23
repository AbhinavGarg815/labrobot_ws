// Minimal non-blocking POSIX serial port.

#ifndef LABROBOT_HARDWARE__SERIAL_PORT_HPP_
#define LABROBOT_HARDWARE__SERIAL_PORT_HPP_

#include <cstdint>
#include <string>

namespace labrobot_hardware
{

class SerialPort
{
public:
  ~SerialPort();

  // Opens and configures the port as raw 8N1 at `baud`.
  bool open(const std::string & device, uint32_t baud, std::string & error);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // Reads whatever is already available, up to `max`.
  int read(uint8_t * buffer, size_t max);

  // Writes `len` bytes.
  bool write(const uint8_t * data, size_t len);
  int last_errno() const { return last_errno_; }

  void flush_input();

private:
  int fd_ = -1;
  int last_errno_ = 0;
};

}  // namespace labrobot_hardware

#endif  // LABROBOT_HARDWARE__SERIAL_PORT_HPP_
