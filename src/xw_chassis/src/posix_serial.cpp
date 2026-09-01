#include "xw_chassis/posix_serial.hpp"

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace xw_chassis {
namespace {

speed_t baud_to_flag(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B115200;
  }
}

}  // namespace

PosixSerial::PosixSerial(
  std::string port, int baudrate, std::vector<std::string> fallback_ports, double timeout_sec)
: port_(std::move(port)),
  baudrate_(baudrate),
  fallback_ports_(std::move(fallback_ports)),
  timeout_sec_(timeout_sec)
{
}

PosixSerial::~PosixSerial()
{
  close();
}

bool PosixSerial::is_open() const
{
  return fd_ >= 0;
}

std::vector<std::string> PosixSerial::candidates() const
{
  std::vector<std::string> out;
  auto add = [&](const std::string & p) {
    if (p.empty()) {
      return;
    }
    for (const auto & e : out) {
      if (e == p) {
        return;
      }
    }
    out.push_back(p);
  };
  add(port_);
  for (const auto & p : fallback_ports_) {
    add(p);
  }
  return out;
}

void PosixSerial::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  active_port_.clear();
  parser_.reset();
}

bool PosixSerial::configure_port(int fd) const
{
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    return false;
  }
  cfmakeraw(&tty);
  const speed_t speed = baud_to_flag(baudrate_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;  // non-blocking reads; we poll via FIONREAD
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    return false;
  }
  // Avoid USB-CDC reset on open (clear DTR/RTS)
  int status = 0;
  if (ioctl(fd, TIOCMGET, &status) == 0) {
    status &= ~(TIOCM_DTR | TIOCM_RTS);
    ioctl(fd, TIOCMSET, &status);
  }
  tcflush(fd, TCIOFLUSH);
  return true;
}

bool PosixSerial::open()
{
  close();
  std::runtime_error last_err("serial open failed");
  bool had_err = false;
  for (const auto & path : candidates()) {
    const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      had_err = true;
      last_err = std::runtime_error(std::string("open ") + path + ": " + std::strerror(errno));
      continue;
    }
    // Drop O_NONBLOCK for steadier write semantics after configure
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
      fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    }
    if (!configure_port(fd)) {
      had_err = true;
      last_err = std::runtime_error(std::string("configure ") + path + " failed");
      ::close(fd);
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    fd_ = fd;
    active_port_ = path;
    return true;
  }
  if (had_err) {
    throw last_err;
  }
  return false;
}

void PosixSerial::write(const uint8_t * data, size_t len)
{
  if (!is_open()) {
    throw std::runtime_error("serial not open");
  }
  size_t off = 0;
  while (off < len) {
    const ssize_t n = ::write(fd_, data + off, len - off);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::runtime_error(std::string("serial write: ") + std::strerror(errno));
    }
    off += static_cast<size_t>(n);
  }
}

void PosixSerial::write(const std::vector<uint8_t> & data)
{
  write(data.data(), data.size());
}

std::vector<uint8_t> PosixSerial::read_available()
{
  std::vector<uint8_t> out;
  if (!is_open()) {
    return out;
  }
  int avail = 0;
  if (ioctl(fd_, FIONREAD, &avail) != 0 || avail <= 0) {
    return out;
  }
  out.resize(static_cast<size_t>(avail));
  const ssize_t n = ::read(fd_, out.data(), out.size());
  if (n <= 0) {
    out.clear();
    return out;
  }
  out.resize(static_cast<size_t>(n));
  return out;
}

ParsedFrames PosixSerial::drain()
{
  const auto raw = read_available();
  return parser_.feed(raw.data(), raw.size());
}

std::string wait_for_port(const std::vector<std::string> & paths, double timeout_sec)
{
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::duration<double>(timeout_sec);
  while (clock::now() < deadline) {
    for (const auto & p : paths) {
      if (p.empty()) {
        continue;
      }
      if (::access(p.c_str(), R_OK | W_OK) == 0) {
        return p;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return {};
}

}  // namespace xw_chassis
