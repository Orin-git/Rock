#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "xw_interfaces/msg/power_state.hpp"

#include "xw_chassis/bms_protocol.hpp"
#include "xw_chassis/posix_serial.hpp"
#include "xw_chassis/serial_protocol.hpp"

using namespace std::chrono_literals;

namespace {

geometry_msgs::msg::Quaternion yaw_to_quat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double wall_now()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

std::string bytes_to_hex(const std::vector<uint8_t> & data)
{
  static const char * kHex = "0123456789abcdef";
  std::string out;
  out.resize(data.size() * 2);
  for (size_t i = 0; i < data.size(); ++i) {
    out[2 * i] = kHex[(data[i] >> 4) & 0xF];
    out[2 * i + 1] = kHex[data[i] & 0xF];
  }
  return out;
}

}  // namespace

class ChassisNode : public rclcpp::Node
{
public:
  ChassisNode()
  : Node("xw_chassis")
  {
    declare_parameter("use_sim_hw", true);
    declare_parameter("publish_tf", true);
    declare_parameter("publish_odom_tf", true);
    declare_parameter("odom_topic", std::string("odom"));
    declare_parameter("base_frame", std::string("base_link"));
    declare_parameter("odom_frame", std::string("odom"));
    declare_parameter("wheel_separation", 0.35);
    declare_parameter("max_linear", 0.5);
    declare_parameter("max_angular", 1.0);
    declare_parameter("serial_port", std::string("/dev/chassis"));
    declare_parameter("serial_baud_rate", 115200);
    declare_parameter("serial_fallback", std::string("/dev/ttyACM0"));
    declare_parameter("cmd_timeout_sec", 0.5);
    declare_parameter("serial_no_frame_reopen_sec", 2.0);
    declare_parameter("charge_current_dock_min", 0.05);
    declare_parameter("lock_motion_when_docked", true);
    declare_parameter("bms_byte_order", std::string("big"));
    declare_parameter("bms_comm_ok_value", 1);
    declare_parameter("bms_raw_frame_topic", std::string("/bms/raw_frame"));

    use_sim_ = get_parameter("use_sim_hw").as_bool();
    publish_tf_ = get_parameter("publish_tf").as_bool() &&
      get_parameter("publish_odom_tf").as_bool();
    odom_topic_ = get_parameter("odom_topic").as_string();
    if (!odom_topic_.empty() && odom_topic_.front() == '/') {
      odom_topic_.erase(odom_topic_.begin());
    }
    if (odom_topic_.empty()) {
      odom_topic_ = "odom";
    }
    base_frame_ = get_parameter("base_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    cmd_timeout_ = get_parameter("cmd_timeout_sec").as_double();
    no_frame_reopen_ = get_parameter("serial_no_frame_reopen_sec").as_double();
    bms_byte_order_ = get_parameter("bms_byte_order").as_string();
    bms_comm_ok_ = get_parameter("bms_comm_ok_value").as_int();
    if (bms_byte_order_ != "little" && bms_byte_order_ != "big") {
      throw std::runtime_error("bms_byte_order must be 'little' or 'big'");
    }

    last_odom_wall_ = wall_now();

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&ChassisNode::on_cmd, this, std::placeholders::_1));
    charge_mode_sub_ = create_subscription<std_msgs::msg::Int8>(
      "/xw/chassis/charge_mode", 10,
      std::bind(&ChassisNode::on_charge_mode, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    power_pub_ = create_publisher<xw_interfaces::msg::PowerState>("/xw/power", 10);
    motor_disabled_pub_ = create_publisher<std_msgs::msg::Bool>("/xw/chassis/motor_disabled", 10);
    bms_raw_pub_ = create_publisher<std_msgs::msg::ByteMultiArray>(
      get_parameter("bms_raw_frame_topic").as_string(), 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (use_sim_) {
      timer_ = create_wall_timer(50ms, std::bind(&ChassisNode::tick_sim, this));
      RCLCPP_INFO(get_logger(), "chassis started (use_sim_hw=True) [C++]");
    } else {
      const auto port = get_parameter("serial_port").as_string();
      const int baud = get_parameter("serial_baud_rate").as_int();
      const auto fallback = get_parameter("serial_fallback").as_string();
      std::vector<std::string> fallbacks;
      if (!fallback.empty()) {
        fallbacks.push_back(fallback);
      }
      serial_ = std::make_unique<xw_chassis::PosixSerial>(port, baud, fallbacks);
      timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / 30.0),
        std::bind(&ChassisNode::tick_serial, this));
      try_open_serial(true);
      RCLCPP_INFO(
        get_logger(),
        "chassis started (use_sim_hw=False port=%s baud=%d bms_byte_order=%s) [C++]",
        port.c_str(), baud, bms_byte_order_.c_str());
    }

    power_timer_ = create_wall_timer(1s, std::bind(&ChassisNode::publish_power, this));
  }

  ~ChassisNode() override
  {
    if (serial_) {
      try {
        send_speed(0.0, 0.0, 0.0);
      } catch (...) {
      }
      serial_->close();
    }
  }

private:
  std::tuple<double, double, double> clamp_cmd(const geometry_msgs::msg::Twist & msg)
  {
    const double max_lin = get_parameter("max_linear").as_double();
    const double max_ang = get_parameter("max_angular").as_double();
    const double vx = std::clamp(msg.linear.x, -max_lin, max_lin);
    const double vy = std::clamp(msg.linear.y, -max_lin, max_lin);
    const double wz = std::clamp(msg.angular.z, -max_ang, max_ang);
    return {vx, vy, wz};
  }

  void on_charge_mode(const std_msgs::msg::Int8::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    charge_mode_ = static_cast<int>(msg->data) & 0xFF;
  }

  std::tuple<double, double, double> effective_cmd_unlocked()
  {
    const bool lock = get_parameter("lock_motion_when_docked").as_bool();
    const bool charging = saw_bms_frame_ ? bms_charging_ : charging_;
    if (lock && charging && charge_mode_ == 0) {
      return {0.0, 0.0, 0.0};
    }
    return {cmd_vx_, cmd_vy_, cmd_wz_};
  }

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto [vx, vy, wz] = clamp_cmd(*msg);
    double evx, evy, ewz;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd_vx_ = vx;
      cmd_vy_ = vy;
      cmd_wz_ = wz;
      last_cmd_wall_ = wall_now();
      std::tie(evx, evy, ewz) = effective_cmd_unlocked();
      if (use_sim_) {
        meas_vx_ = evx;
        meas_wz_ = ewz;
      }
    }
    if (!use_sim_) {
      send_speed(evx, evy, ewz);
    }
  }

  bool try_open_serial(bool initial)
  {
    if (!serial_) {
      return false;
    }
    std::vector<std::string> paths = {serial_->port()};
    for (const auto & p : serial_->fallback_ports()) {
      paths.push_back(p);
    }
    const auto ready = xw_chassis::wait_for_port(paths, initial ? 5.0 : 0.5);
    if (ready.empty()) {
      RCLCPP_ERROR(get_logger(), "chassis serial not ready");
      return false;
    }
    try {
      const bool ok = serial_->open();
      if (ok) {
        last_frame_wall_ = wall_now();
        RCLCPP_INFO(
          get_logger(), "chassis serial open %s @ %d",
          serial_->active_port().c_str(), serial_->baudrate());
      }
      return ok;
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "chassis serial open failed: %s", ex.what());
      return false;
    }
  }

  void send_speed(double vx, double vy, double wz)
  {
    if (!serial_ || !serial_->is_open()) {
      return;
    }
    try {
      int mode = 0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        mode = charge_mode_;
      }
      auto payload = xw_chassis::pack_speed(vx, vy, wz, mode);
      serial_->write(payload);
      std::lock_guard<std::mutex> lock(mutex_);
      ++tx_count_;
      last_tx_hex_ = bytes_to_hex(payload);
    } catch (const std::exception & ex) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++write_errors_;
      RCLCPP_ERROR(get_logger(), "chassis serial write failed: %s", ex.what());
      serial_->close();
    }
  }

  void publish_motor_disabled(bool disabled, bool force = false)
  {
    const double now = wall_now();
    const bool changed = !last_motor_pub_valid_ || disabled != last_motor_pub_;
    const bool period = (now - last_motor_pub_wall_) >= 1.0;
    if (!force && !changed && !period) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = disabled;
    motor_disabled_pub_->publish(msg);
    last_motor_pub_ = disabled;
    last_motor_pub_valid_ = true;
    last_motor_pub_wall_ = now;
  }

  void publish_odom()
  {
    double x, y, yaw, mvx, mvy, mwz;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      x = x_;
      y = y_;
      yaw = yaw_;
      mvx = meas_vx_;
      mvy = meas_vy_;
      mwz = meas_wz_;
    }
    const auto now = get_clock()->now();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = yaw_to_quat(yaw);
    odom.twist.twist.linear.x = mvx;
    odom.twist.twist.linear.y = mvy;
    odom.twist.twist.angular.z = mwz;
    odom.pose.covariance[0] = 0.05;
    odom.pose.covariance[7] = 0.05;
    odom.pose.covariance[35] = 0.1;
    odom.twist.covariance[0] = 0.02;
    odom.twist.covariance[7] = 0.05;
    odom.twist.covariance[35] = 0.15;
    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now;
      t.header.frame_id = odom_frame_;
      t.child_frame_id = base_frame_;
      t.transform.translation.x = x;
      t.transform.translation.y = y;
      t.transform.rotation = yaw_to_quat(yaw);
      tf_broadcaster_->sendTransform(t);
    }
  }

  void handle_bms_frames(const std::vector<std::vector<uint8_t>> & bms_frames)
  {
    if (bms_frames.empty()) {
      return;
    }
    for (const auto & raw : bms_frames) {
      std_msgs::msg::ByteMultiArray msg;
      msg.data.assign(raw.begin(), raw.end());
      bms_raw_pub_->publish(msg);

      auto sample = xw_chassis::parse_battery_frame(raw, bms_byte_order_);
      if (!sample.has_value()) {
        const double now = wall_now();
        if (now - last_bms_invalid_log_ >= 5.0) {
          RCLCPP_WARN(get_logger(), "Dropped invalid BMS payload");
          last_bms_invalid_log_ = now;
        }
        continue;
      }
      if (sample->comm_status != bms_comm_ok_) {
        const double now = wall_now();
        if (now - last_bms_invalid_log_ >= 5.0) {
          RCLCPP_WARN(
            get_logger(), "BMS communication status=%d (expected %d)",
            sample->comm_status, bms_comm_ok_);
          last_bms_invalid_log_ = now;
        }
        continue;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      battery_percent_ = sample->soc_percent;
      battery_voltage_ = sample->voltage;
      bms_current_ = sample->current;
      bms_charging_ =
        (sample->state == xw_chassis::ChargeState::CHARGING) &&
        (sample->protection_bits == 0);
      saw_bms_frame_ = true;
      ++bms_rx_count_;
      if (saw_charge_frame_) {
        const double imin = get_parameter("charge_current_dock_min").as_double();
        docked_ = bms_charging_ && charging_current_ >= imin;
      }
    }
  }

  void tick_sim()
  {
    constexpr double dt = 0.05;
    double vx, vy, wz;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      std::tie(vx, vy, wz) = effective_cmd_unlocked();
      yaw_ += wz * dt;
      x_ += vx * std::cos(yaw_) * dt;
      y_ += vx * std::sin(yaw_) * dt;
      meas_vx_ = vx;
      meas_wz_ = wz;
    }
    publish_odom();
  }

  void tick_serial()
  {
    if (!serial_) {
      return;
    }
    const double now = wall_now();

    if (!serial_->is_open()) {
      try_open_serial(false);
      return;
    }

    double evx, evy, ewz;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (last_cmd_wall_ > 0.0 && (now - last_cmd_wall_) > cmd_timeout_) {
        cmd_vx_ = 0.0;
        cmd_vy_ = 0.0;
        cmd_wz_ = 0.0;
      }
      std::tie(evx, evy, ewz) = effective_cmd_unlocked();
    }
    send_speed(evx, evy, ewz);

    xw_chassis::ParsedFrames parsed;
    try {
      parsed = serial_->drain();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "chassis serial read failed: %s", ex.what());
      serial_->close();
      return;
    }

    handle_bms_frames(parsed.bms);

    if (!parsed.charge.empty()) {
      const auto & ch = parsed.charge.back();
      std::lock_guard<std::mutex> lock(mutex_);
      saw_charge_frame_ = true;
      charging_current_ = ch.current;
      ir_red_ = ch.red;
      charging_ = ch.charging;
      charge_set_state_ = ch.charge_set_state;
      const double imin = get_parameter("charge_current_dock_min").as_double();
      const bool charging_now = saw_bms_frame_ ? bms_charging_ : charging_;
      docked_ = charging_now && charging_current_ >= imin;
    }

    if (!parsed.motion.empty()) {
      const auto & frame = parsed.motion.back();
      double dt;
      int flag_stop;
      uint64_t tx_count;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_frame_wall_ = now;
        rx_count_ += parsed.motion.size();
        dt = now - last_odom_wall_;
        if (dt < 1e-4) {
          dt = 1e-4;
        }
        if (dt > 0.2) {
          dt = 0.2;
        }
        meas_vx_ = frame.vx;
        meas_vy_ = frame.vy;
        // MCU reports yaw rate with opposite sign to ROS convention (CCW+).
        // Bench-verified 2026-09-01: wheel odom yaw read -30 deg while the
        // robot physically turned left +90 (imu +93, ekf/motion command +90).
        meas_wz_ = -frame.wz;
        const double meas_mag = std::abs(meas_vx_) + std::abs(meas_vy_) + std::abs(meas_wz_);
        double ivx, ivy, iwz;
        if (meas_mag > 1e-3) {
          ivx = meas_vx_;
          ivy = meas_vy_;
          iwz = meas_wz_;
        } else {
          ivx = cmd_vx_;
          ivy = cmd_vy_;
          iwz = cmd_wz_;
        }
        x_ += (ivx * std::cos(yaw_) - ivy * std::sin(yaw_)) * dt;
        y_ += (ivx * std::sin(yaw_) + ivy * std::cos(yaw_)) * dt;
        yaw_ += iwz * dt;
        last_odom_wall_ = now;
        if (meas_mag <= 1e-3) {
          meas_vx_ = ivx;
          meas_vy_ = ivy;
          meas_wz_ = iwz;
        }
        flag_stop_ = frame.flag_stop;
        flag_stop = flag_stop_;
        tx_count = tx_count_;
      }
      const bool disabled = (flag_stop == 0);
      if (disabled && (tx_count % 90 == 1)) {
        RCLCPP_WARN(
          get_logger(),
          "MCU Flag_Stop=%d → motor_disabled=true (cannot drive)", flag_stop);
      }
      publish_motor_disabled(disabled);
      publish_odom();
    } else {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        double dt = now - last_odom_wall_;
        if (dt >= 0.02 &&
          (std::abs(cmd_vx_) + std::abs(cmd_vy_) + std::abs(cmd_wz_) > 1e-3))
        {
          if (dt > 0.2) {
            dt = 0.2;
          }
          x_ += (cmd_vx_ * std::cos(yaw_) - cmd_vy_ * std::sin(yaw_)) * dt;
          y_ += (cmd_vx_ * std::sin(yaw_) + cmd_vy_ * std::cos(yaw_)) * dt;
          yaw_ += cmd_wz_ * dt;
          meas_vx_ = cmd_vx_;
          meas_vy_ = cmd_vy_;
          meas_wz_ = cmd_wz_;
          last_odom_wall_ = now;
        }
      }
      publish_odom();
      if (last_frame_wall_ > 0.0 && (now - last_frame_wall_) > no_frame_reopen_) {
        RCLCPP_WARN(get_logger(), "chassis serial no frame, reopening");
        serial_->close();
      }
    }
  }

  void publish_power()
  {
    xw_interfaces::msg::PowerState p;
    p.stamp = get_clock()->now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      p.ir_red = static_cast<uint8_t>(ir_red_ & 0xFF);
      p.charging_current = static_cast<float>(charging_current_);
      if (use_sim_) {
        p.battery_percent = 88.0f;
        p.voltage = 24.5f;
        p.charging = charging_;
        p.docked = docked_;
        p.detail = "mock charge_mode=" + std::to_string(charge_mode_);
      } else {
        p.battery_percent = static_cast<float>(battery_percent_);
        p.voltage = static_cast<float>(battery_voltage_);
        p.charging = saw_bms_frame_ ? bms_charging_ : charging_;
        p.docked = docked_;
        const std::string port = serial_ ? serial_->active_port() : "";
        std::ostringstream oss;
        oss << "serial:" << (port.empty() ? "closed" : port)
            << " tx=" << tx_count_ << " rx=" << rx_count_
            << " bms_rx=" << bms_rx_count_ << " err=" << write_errors_
            << " flag_stop=" << flag_stop_
            << (saw_charge_frame_ ? " 0x7c" : " no-0x7c")
            << (saw_bms_frame_ ? " bms" : " no-bms")
            << " mode=" << charge_mode_ << " ir=" << ir_red_
            << " I=" << charging_current_ << "A Ibms=" << bms_current_
            << "A SOC=" << battery_percent_ << "% V=" << battery_voltage_
            << " last=" << last_tx_hex_
            << " cmd=(" << cmd_vx_ << "," << cmd_wz_ << ")"
            << " meas=(" << meas_vx_ << "," << meas_wz_ << ")";
        p.detail = oss.str();
      }
    }
    power_pub_->publish(p);
  }

  bool use_sim_{true};
  bool publish_tf_{true};
  std::string odom_topic_;
  std::string base_frame_;
  std::string odom_frame_;
  double cmd_timeout_{0.5};
  double no_frame_reopen_{2.0};
  std::string bms_byte_order_{"big"};
  int bms_comm_ok_{1};

  std::mutex mutex_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double meas_vx_{0.0};
  double meas_vy_{0.0};
  double meas_wz_{0.0};
  double cmd_vx_{0.0};
  double cmd_vy_{0.0};
  double cmd_wz_{0.0};
  int flag_stop_{0};
  double last_cmd_wall_{0.0};
  double last_frame_wall_{0.0};
  bool last_motor_pub_{false};
  bool last_motor_pub_valid_{false};
  double last_motor_pub_wall_{0.0};
  double last_odom_wall_{0.0};
  uint64_t tx_count_{0};
  uint64_t rx_count_{0};
  std::string last_tx_hex_;
  uint64_t write_errors_{0};
  int charge_mode_{0};
  bool charging_{false};
  double charging_current_{0.0};
  int ir_red_{0};
  int charge_set_state_{0};
  bool docked_{false};
  bool saw_charge_frame_{false};
  double battery_percent_{0.0};
  double battery_voltage_{0.0};
  double bms_current_{0.0};
  bool bms_charging_{false};
  bool saw_bms_frame_{false};
  uint64_t bms_rx_count_{0};
  double last_bms_invalid_log_{0.0};

  std::unique_ptr<xw_chassis::PosixSerial> serial_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr charge_mode_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<xw_interfaces::msg::PowerState>::SharedPtr power_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_disabled_pub_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr bms_raw_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr power_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ChassisNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    fprintf(stderr, "xw_chassis fatal: %s\n", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
