// Minimal non-blocking POSIX serial port.
//
// ros2_control's read()/write() run inside the controller_manager update loop,
// so neither may ever block: a stalled write would stall every controller. This
// wrapper therefore opens the port O_NONBLOCK and drains only what is already
// buffered.

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

  // Opens and configures the port as raw 8N1 at `baud`. On failure returns
  // false and fills `error`.
  bool open(const std::string & device, uint32_t baud, std::string & error);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // Reads whatever is already available, up to `max`. Returns bytes read, 0 if
  // nothing is pending, or -1 on a real error (EAGAIN is reported as 0).
  int read(uint8_t * buffer, size_t max);

  // Writes `len` bytes. Returns true only if all of them were accepted.
  bool write(const uint8_t * data, size_t len);

  // Discards buffered input, so a restart never acts on frames queued while the
  // controller was inactive.
  void flush_input();

private:
  int fd_ = -1;
};

}  // namespace labrobot_hardware

#endif  // LABROBOT_HARDWARE__SERIAL_PORT_HPP_
