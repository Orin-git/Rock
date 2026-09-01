// C++ port of bms_receiver/node.py + protocol.py.
// Decode controller-forwarded battery telemetry frames (30-byte BMS frame).
// Topics, node name and behavior identical to the Python version.
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include <std_msgs/msg/byte_multi_array.hpp>

namespace {

constexpr size_t FRAME_SIZE = 30;

enum ChargeState : uint8_t {
  STANDBY = 0,
  CHARGING = 1,
  DISCHARGING = 2,
};

struct BatterySample {
  double voltage = 0.0;
  double current = 0.0;
  double soc_percent = 0.0;
  double soh_percent = 0.0;
  uint8_t state = STANDBY;
  double mos_temperature = 0.0;
  double env_temperature = 0.0;
  uint32_t warning_bits = 0;
  uint32_t protection_bits = 0;
  uint8_t comm_status = 0;
};

uint8_t xor_bcc(const std::vector<uint8_t> & data) {
  uint8_t result = 0;
  for (uint8_t v : data) {
    result ^= v;
  }
  return result;
}

uint32_t decode(const std::vector<uint8_t> & frame, size_t start, size_t size,
                bool big, bool is_signed) {
  uint32_t value = 0;
  if (big) {
    for (size_t i = 0; i < size; ++i) {
      value = (value << 8) | frame[start + i];
    }
  } else {
    for (size_t i = 0; i < size; ++i) {
      value |= static_cast<uint32_t>(frame[start + i]) << (8 * i);
    }
  }
  if (is_signed && size < 4 && (value >> (size * 8 - 1)) & 1u) {
    // sign-extend
    value |= (~0u) << (size * 8);
  }
  return value;
}

bool decode_frame(const std::vector<uint8_t> & raw, bool big,
                  BatterySample & out, std::string & err) {
  if (raw.size() != FRAME_SIZE) {
    err = "expected 30 bytes, got " + std::to_string(raw.size());
    return false;
  }
  if (raw[0] != 0xFB) { err = "invalid frame header"; return false; }
  if (raw[1] != 0x01) { err = "invalid battery frame type"; return false; }
  if (raw[2] != 0x19) { err = "invalid payload length"; return false; }
  if (raw[29] != 0xFD) { err = "invalid frame tail"; return false; }
  const std::vector<uint8_t> head(raw.begin(), raw.begin() + 28);
  if (xor_bcc(head) != raw[28]) { err = "battery frame BCC mismatch"; return false; }

  const auto s16 = [&](size_t start) {
    return static_cast<int16_t>(decode(raw, start, 2, big, true));
  };
  out.voltage = decode(raw, 3, 4, big, false) * 0.001;
  out.current = static_cast<double>(static_cast<int32_t>(decode(raw, 7, 4, big, false))) * 0.001;
  out.soc_percent = decode(raw, 11, 2, big, false) * 0.1;
  out.soh_percent = static_cast<double>(raw[13]);
  if (raw[14] > DISCHARGING) {
    err = "invalid charge state: " + std::to_string(raw[14]);
    return false;
  }
  out.state = raw[14];
  out.mos_temperature = s16(15) * 0.1;
  out.env_temperature = s16(17) * 0.1;
  out.warning_bits = (decode(raw, 19, 2, big, false) << 16) |
                     decode(raw, 21, 2, big, false);
  out.protection_bits = (decode(raw, 23, 2, big, false) << 16) |
                        decode(raw, 25, 2, big, false);

  if (out.voltage < 5.0 || out.voltage > 100.0) {
    err = "implausible voltage: " + std::to_string(out.voltage);
    return false;
  }
  if (std::abs(out.current) > 500.0) {
    err = "implausible current";
    return false;
  }
  if (out.soc_percent < 0.0 || out.soc_percent > 100.0) {
    err = "implausible SOC";
    return false;
  }
  if (out.soh_percent < 0.0 || out.soh_percent > 100.0) {
    err = "implausible SOH";
    return false;
  }
  if (out.mos_temperature < -60.0 || out.mos_temperature > 150.0) {
    err = "implausible MOS temperature";
    return false;
  }
  if (out.env_temperature < -60.0 || out.env_temperature > 150.0) {
    err = "implausible environment temperature";
    return false;
  }
  return true;
}

}  // namespace

class BmsReceiverNode : public rclcpp::Node {
 public:
  BmsReceiverNode() : Node("bms_receiver_node") {
    this->declare_parameter<std::string>("byte_order", "big");
    this->declare_parameter<double>("stale_timeout_sec", 3.0);
    this->declare_parameter<double>("min_voltage_v", 15.0);
    this->declare_parameter<double>("max_voltage_v", 35.0);
    this->declare_parameter<double>("max_abs_current_a", 200.0);
    this->declare_parameter<double>("min_temperature_c", -40.0);
    this->declare_parameter<double>("max_temperature_c", 100.0);
    this->declare_parameter<int>("comm_ok_value", 1);
    this->declare_parameter<std::string>("raw_frame_topic", "/bms/raw_frame");
    this->declare_parameter<std::string>("battery_state_topic", "/battery_state");
    this->declare_parameter<std::string>("power_voltage_topic", "/PowerVoltage");
    this->declare_parameter<std::string>("charger_status_topic", "/charger_status");
    this->declare_parameter<std::string>("charging_flag_topic", "/robot_charging_flag");
    this->declare_parameter<std::string>(
        "charging_current_topic", "/robot_charging_current");

    byte_order_ = this->get_parameter("byte_order").as_string();
    stale_timeout_sec_ = this->get_parameter("stale_timeout_sec").as_double();
    min_voltage_v_ = this->get_parameter("min_voltage_v").as_double();
    max_voltage_v_ = this->get_parameter("max_voltage_v").as_double();
    max_abs_current_a_ = this->get_parameter("max_abs_current_a").as_double();
    min_temperature_c_ = this->get_parameter("min_temperature_c").as_double();
    max_temperature_c_ = this->get_parameter("max_temperature_c").as_double();
    comm_ok_value_ = this->get_parameter("comm_ok_value").as_int();

    battery_state_pub_ =
        this->create_publisher<sensor_msgs::msg::BatteryState>(
            this->get_parameter("battery_state_topic").as_string(), 10);
    power_voltage_pub_ =
        this->create_publisher<std_msgs::msg::Float32>(
            this->get_parameter("power_voltage_topic").as_string(), 10);
    charging_flag_pub_ =
        this->create_publisher<std_msgs::msg::Bool>(
            this->get_parameter("charging_flag_topic").as_string(), 10);
    charging_current_pub_ =
        this->create_publisher<std_msgs::msg::Float32>(
            this->get_parameter("charging_current_topic").as_string(), 10);
    // NOTE: /charger_status (ox_battery_util) intentionally skipped — the
    // message package is not available on Gen2 (Python version skips too).

    raw_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
        this->get_parameter("raw_frame_topic").as_string(), 10,
        [this](const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
          this->on_raw_frame(*msg);
        });

    started_at_ = steady_now();
    watchdog_ = this->create_wall_timer(
        std::chrono::seconds(1), [this]() { this->check_stale(); });
    RCLCPP_INFO(this->get_logger(), "BMS receiver ready: byte_order=%s, "
                "stale_timeout=%.1fs", byte_order_.c_str(), stale_timeout_sec_);
  }

 private:
  void on_raw_frame(const std_msgs::msg::ByteMultiArray & msg) {
    byte_order_ = this->get_parameter("byte_order").as_string();
    comm_ok_value_ = this->get_parameter("comm_ok_value").as_int();

    BatterySample s;
    std::string err;
    if (!decode_frame(msg.data, byte_order_ == "big", s, err)) {
      const double now = steady_now();
      if (now - last_invalid_log_at_ >= 5.0) {
        RCLCPP_WARN(this->get_logger(), "Dropped invalid BMS frame: %s", err.c_str());
        last_invalid_log_at_ = now;
      }
      return;
    }
    // configured limits
    if (!(min_voltage_v_ <= s.voltage && s.voltage <= max_voltage_v_)) {
      const double now = steady_now();
      if (now - last_invalid_log_at_ >= 5.0) {
        RCLCPP_WARN(this->get_logger(),
                    "Dropped invalid BMS frame: voltage outside configured limits");
        last_invalid_log_at_ = now;
      }
      return;
    }
    if (std::abs(s.current) > max_abs_current_a_ ||
        s.mos_temperature < min_temperature_c_ ||
        s.mos_temperature > max_temperature_c_ ||
        s.env_temperature < min_temperature_c_ ||
        s.env_temperature > max_temperature_c_) {
      const double now = steady_now();
      if (now - last_invalid_log_at_ >= 5.0) {
        RCLCPP_WARN(this->get_logger(),
                    "Dropped invalid BMS frame: outside configured limits");
        last_invalid_log_at_ = now;
      }
      return;
    }
    if (s.comm_status != comm_ok_value_) {
      const double now = steady_now();
      if (now - last_invalid_log_at_ >= 5.0) {
        RCLCPP_WARN(this->get_logger(),
                    "Dropped invalid BMS frame: lower controller reports BMS "
                    "communication status %d", s.comm_status);
        last_invalid_log_at_ = now;
      }
      return;
    }

    const bool charging = s.state == CHARGING && s.comm_status == comm_ok_value_ &&
                          s.protection_bits == 0;

    auto battery = sensor_msgs::msg::BatteryState();
    battery.header.stamp = this->now();
    battery.header.frame_id = "battery";
    battery.voltage = s.voltage;
    battery.temperature = s.env_temperature;
    battery.current = s.current;
    battery.charge = std::numeric_limits<double>::quiet_NaN();
    battery.capacity = std::numeric_limits<double>::quiet_NaN();
    battery.design_capacity = std::numeric_limits<double>::quiet_NaN();
    battery.percentage = s.soc_percent / 100.0;
    if (s.state == CHARGING) {
      battery.power_supply_status =
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    } else if (s.state == DISCHARGING) {
      battery.power_supply_status =
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    } else {
      battery.power_supply_status =
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
    const bool has_fault =
        s.comm_status != comm_ok_value_ || s.warning_bits || s.protection_bits;
    battery.power_supply_health = has_fault
        ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
        : sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    battery.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    battery.present = s.comm_status == comm_ok_value_;

    battery_state_pub_->publish(battery);

    std_msgs::msg::Float32 pv;
    pv.data = static_cast<float>(s.soc_percent);
    power_voltage_pub_->publish(pv);

    std_msgs::msg::Bool flag;
    flag.data = charging;
    charging_flag_pub_->publish(flag);

    std_msgs::msg::Float32 cc;
    cc.data = charging ? static_cast<float>(std::abs(s.current)) : 0.0f;
    charging_current_pub_->publish(cc);

    last_valid_at_ = steady_now();
  }

  void check_stale() {
    const double now = steady_now();
    const double reference = last_valid_at_ > 0.0 ? last_valid_at_ : started_at_;
    if (now - reference < stale_timeout_sec_) {
      return;
    }
    if (now - last_stale_log_at_ >= stale_timeout_sec_) {
      RCLCPP_WARN(this->get_logger(), "No valid BMS telemetry for %.1fs",
                  now - reference);
      last_stale_log_at_ = now;
    }
  }

  static double steady_now() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  std::string byte_order_;
  double stale_timeout_sec_ = 3.0;
  double min_voltage_v_ = 15.0;
  double max_voltage_v_ = 35.0;
  double max_abs_current_a_ = 200.0;
  double min_temperature_c_ = -40.0;
  double max_temperature_c_ = 100.0;
  int comm_ok_value_ = 1;

  double started_at_ = 0.0;
  double last_valid_at_ = 0.0;
  double last_invalid_log_at_ = 0.0;
  double last_stale_log_at_ = 0.0;

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr power_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charging_flag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr charging_current_pub_;
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr raw_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BmsReceiverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
