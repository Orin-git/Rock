#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "xw_chassis/serial_protocol.hpp"

namespace xw_chassis {

/** Thin POSIX UART wrapper (no DTR/RTS toggle on open). */
class PosixSerial
{
public:
  PosixSerial(
    std::string port, int baudrate = 115200,
    std::vector<std::string> fallback_ports = {}, double timeout_sec = 0.02);
  ~PosixSerial();

  PosixSerial(const PosixSerial &) = delete;
  PosixSerial & operator=(const PosixSerial &) = delete;

  bool is_open() const;
  const std::string & active_port() const { return active_port_; }
  const std::string & port() const { return port_; }
  int baudrate() const { return baudrate_; }
  const std::vector<std::string> & fallback_ports() const { return fallback_ports_; }
  FrameParser & parser() { return parser_; }

  void close();
  /** Try primary then fallbacks. Throws std::runtime_error on total failure after attempts. */
  bool open();

  void write(const uint8_t * data, size_t len);
  void write(const std::vector<uint8_t> & data);

  std::vector<uint8_t> read_available();
  ParsedFrames drain();

private:
  std::vector<std::string> candidates() const;
  bool configure_port(int fd) const;

  std::string port_;
  int baudrate_;
  std::vector<std::string> fallback_ports_;
  double timeout_sec_;
  int fd_{-1};
  std::string active_port_;
  FrameParser parser_;
};

/** Wait until one of paths exists and is R/W accessible. */
std::string wait_for_port(const std::vector<std::string> & paths, double timeout_sec);

}  // namespace xw_chassis
