// C++ port of xw_sensors/wt901_imu_node (Python -> C++).
// WT901C485 (RS-485 Modbus RTU) -> /imu/data (frame_id=imu_link).
// Logic, topics and node names identical to the Python version (backup kept).
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace {

using std::chrono::steady_clock;

constexpr uint8_t REG_START = 0x34;
constexpr uint8_t REG_COUNT = 12;
constexpr size_t RESP_EXPECT = 5 + 2 * REG_COUNT;  // addr func len data crc

uint16_t crc16_modbus(const std::vector<uint8_t> & data) {
  uint16_t crc = 0xFFFF;
  for (uint8_t b : data) {
    crc ^= b;
    for (int i = 0; i < 8; ++i) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

geometry_msgs::msg::Quaternion euler_to_quat(double roll, double pitch, double yaw) {
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

speed_t baud_to_speed(int baud) {
  switch (baud) {
    case 1200: return B1200;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return B9600;
  }
}

class SerialPort {
 public:
  ~SerialPort() { close(); }

  bool open(const std::string & path, int baud) {
    close();
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      return false;
    }
    struct termios tio {};
    if (tcgetattr(fd_, &tio) != 0) {
      close();
      return false;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD | CS8;
    tio.c_cflag &= ~(CSTOPB | PARENB);
    tio.c_cflag &= ~(CSIZE);
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    const speed_t sp = baud_to_speed(baud);
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      close();
      return false;
    }
    // exclusive avoids silent multi-open failures on CH340
    ioctl(fd_, TIOCEXCL);
    port_ = path;
    return true;
  }

  void close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool is_open() const { return fd_ >= 0; }
  std::string port() const { return port_; }

  void reset_input_buffer() {
    if (fd_ >= 0) {
      tcflush(fd_, TCIOFLUSH);
    }
  }

  bool write(const std::vector<uint8_t> & data) {
    if (fd_ < 0) {
      return false;
    }
    return ::write(fd_, data.data(), data.size()) == static_cast<ssize_t>(data.size());
  }

  // Replicates pyserial read(n, timeout=0.02): loop until n bytes or deadline.
  size_t read_exact(std::vector<uint8_t> & buf, size_t n,
                    steady_clock::time_point deadline) {
    while (buf.size() < n && steady_clock::now() < deadline) {
      fd_set rf;
      FD_ZERO(&rf);
      FD_SET(fd_, &rf);
      timeval tv {};
      tv.tv_usec = 20000;  // 20 ms
      int r = ::select(fd_ + 1, &rf, nullptr, nullptr, &tv);
      if (r > 0) {
        uint8_t tmp[512];
        ssize_t got = ::read(fd_, tmp, sizeof(tmp));
        if (got > 0) {
          for (ssize_t i = 0; i < got; ++i) {
            buf.push_back(tmp[i]);
          }
        } else if (got < 0) {
          break;
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
    return buf.size();
  }

 private:
  int fd_ = -1;
  std::string port_;
};

}  // namespace

class Wt901ImuNode : public rclcpp::Node {
 public:
  Wt901ImuNode() : Node("xw_wt901_imu") {
    this->declare_parameter<std::string>("port", "/dev/imu");
    this->declare_parameter<std::string>("port_fallback", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud", 9600);
    this->declare_parameter<int>("slave_id", 0x50);
    this->declare_parameter<std::string>("frame_id", "imu_link");
    this->declare_parameter<double>("rate", 15.0);
    this->declare_parameter<double>("timeout_s", 0.12);
    // Subtract from raw gz. Must be measured on THIS unit while idle (mean raw wz).
    // Do NOT reuse an old value — wrong sign/magnitude causes EKF yaw crawl.
    this->declare_parameter<double>("gyro_z_bias_rad_s", 0.0);
    // After bias: clamp tiny residual to 0 so stationary EKF does not integrate.
    this->declare_parameter<double>("gyro_z_deadband_rad_s", 0.005);

    frame_id_ = this->get_parameter("frame_id").as_string();
    slave_ = static_cast<uint8_t>(this->get_parameter("slave_id").as_int() & 0xFF);
    baud_ = this->get_parameter("baud").as_int();
    timeout_s_ = this->get_parameter("timeout_s").as_double();
    gyro_z_bias_ = this->get_parameter("gyro_z_bias_rad_s").as_double();
    gyro_z_deadband_ = this->get_parameter("gyro_z_deadband_rad_s").as_double();
    const double rate = std::max(1.0, this->get_parameter("rate").as_double());

    port_candidates_ = {this->get_parameter("port").as_string(),
                        this->get_parameter("port_fallback").as_string()};

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    open_serial();
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate), [this]() { this->tick(); });
    RCLCPP_INFO(this->get_logger(),
                "WT901C485 Modbus IMU port=%s baud=%d slave=0x%02x rate=%.1fHz "
                "bias_z=%.5f deadband=%.5f -> /imu/data",
                ser_.is_open() ? ser_.port().c_str() : "?", baud_, slave_, rate,
                gyro_z_bias_, gyro_z_deadband_);

    param_cb_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & params) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto & p : params) {
            if (p.get_name() == "gyro_z_bias_rad_s") {
              gyro_z_bias_ = p.as_double();
              RCLCPP_INFO(this->get_logger(), "gyro_z_bias_rad_s -> %.6f", gyro_z_bias_);
            } else if (p.get_name() == "gyro_z_deadband_rad_s") {
              gyro_z_deadband_ = std::max(0.0, p.as_double());
              RCLCPP_INFO(this->get_logger(), "gyro_z_deadband_rad_s -> %.6f",
                          gyro_z_deadband_);
            }
          }
          return result;
        });
  }

  ~Wt901ImuNode() override {}

 private:
  void open_serial() {
    ser_.close();
    std::string last_err;
    for (const auto & p : port_candidates_) {
      if (ser_.open(p, baud_)) {
        RCLCPP_INFO(this->get_logger(), "opened %s", p.c_str());
        return;
      }
      last_err = p;
    }
    RCLCPP_ERROR(this->get_logger(), "IMU serial open failed: %s", last_err.c_str());
  }

  std::vector<uint8_t> build_read(uint8_t start, uint8_t count) {
    std::vector<uint8_t> req = {
        slave_, 0x03, static_cast<uint8_t>((start >> 8) & 0xFF),
        static_cast<uint8_t>(start & 0xFF),
        static_cast<uint8_t>((count >> 8) & 0xFF),
        static_cast<uint8_t>(count & 0xFF)};
    const uint16_t crc = crc16_modbus(req);
    req.push_back(static_cast<uint8_t>(crc & 0xFF));
    req.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    return req;
  }

  bool read_regs(std::vector<int> & out) {
    if (!ser_.is_open()) {
      open_serial();
      if (!ser_.is_open()) {
        return false;
      }
    }
    const auto req = build_read(REG_START, REG_COUNT);
    try {
      ser_.reset_input_buffer();
      if (!ser_.write(req)) {
        open_serial();
        return false;
      }
      // RS-485 DE/RE turnaround
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      std::vector<uint8_t> resp;
      const auto deadline = steady_clock::now() +
          std::chrono::duration_cast<steady_clock::duration>(
              std::chrono::duration<double>(timeout_s_));
      const size_t got = ser_.read_exact(resp, RESP_EXPECT, deadline);
      if (got < RESP_EXPECT) {
        return false;
      }
      if (resp[0] != slave_ || resp[1] != 0x03 || resp[2] != 2 * REG_COUNT) {
        return false;
      }
      const uint16_t crc_calc = crc16_modbus(
          std::vector<uint8_t>(resp.begin(), resp.end() - 2));
      const uint16_t crc_rx = resp[resp.size() - 2] | (resp[resp.size() - 1] << 8);
      if (crc_calc != crc_rx) {
        return false;
      }
      out.clear();
      for (size_t i = 3; i + 1 < resp.size() - 2; i += 2) {
        out.push_back(static_cast<int16_t>((resp[i] << 8) | resp[i + 1]));
      }
      return true;
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "IMU IO error: %s", e.what());
      open_serial();
      return false;
    }
  }

  void tick() {
    std::vector<int> regs;
    if (!read_regs(regs) || regs.size() < 12) {
      ++fail_streak_;
      if (fail_streak_ == 1 || fail_streak_ == 10 || fail_streak_ == 50 ||
          fail_streak_ == 200) {
        RCLCPP_WARN(this->get_logger(), "IMU Modbus read fail streak=%d", fail_streak_);
      }
      if (fail_streak_ % 30 == 0) {
        open_serial();
      }
      return;
    }
    if (fail_streak_ > 0) {
      RCLCPP_INFO(this->get_logger(), "IMU Modbus recovered after %d fails", fail_streak_);
    }
    fail_streak_ = 0;
    ++ok_count_;

    const double ax = regs[0] / 32768.0 * 16.0 * 9.80665;
    const double ay = regs[1] / 32768.0 * 16.0 * 9.80665;
    const double az = regs[2] / 32768.0 * 16.0 * 9.80665;
    const double gx = regs[3] / 32768.0 * 2000.0 * M_PI / 180.0;
    const double gy = regs[4] / 32768.0 * 2000.0 * M_PI / 180.0;
    const double gz = regs[5] / 32768.0 * 2000.0 * M_PI / 180.0;
    const double roll = regs[9] / 32768.0 * 180.0 * M_PI / 180.0;
    const double pitch = regs[10] / 32768.0 * 180.0 * M_PI / 180.0;
    const double yaw = regs[11] / 32768.0 * 180.0 * M_PI / 180.0;

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.orientation = euler_to_quat(roll, pitch, yaw);
    msg.orientation_covariance = {0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.1};
    msg.angular_velocity.x = gx;
    msg.angular_velocity.y = gy;
    double gz_corr = gz - gyro_z_bias_;
    if (std::abs(gz_corr) < gyro_z_deadband_) {
      gz_corr = 0.0;
    }
    msg.angular_velocity.z = gz_corr;
    msg.angular_velocity_covariance = {0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02};
    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;
    msg.linear_acceleration_covariance = {0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.08};
    pub_->publish(msg);

    if (ok_count_ == 1 || ok_count_ % 150 == 0) {
      RCLCPP_INFO(this->get_logger(), "IMU ok#%d az=%.2f yaw_deg=%.1f", ok_count_,
                  az, yaw * 180.0 / M_PI);
    }
  }

  std::string frame_id_;
  uint8_t slave_;
  int baud_;
  double timeout_s_;
  double gyro_z_bias_;
  double gyro_z_deadband_{0.005};
  std::vector<std::string> port_candidates_;
  SerialPort ser_;
  int fail_streak_ = 0;
  int ok_count_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Wt901ImuNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
