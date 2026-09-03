// C++ port of xw_safety_gate (Python -> C++).
// Logic, topics, node name and parameters kept identical.
// Original Python: python_legacy/xw_safety_gate/safety_gate_node.py (kept as backup).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "xw_interfaces/msg/ultrasonic_array.hpp"

namespace {

inline double ang_diff(double a, double b)
{
  // Match Python: (a - b + pi) % (2*pi) - pi  (non-negative modulo)
  double d = std::fmod(a - b + M_PI, 2.0 * M_PI);
  if (d < 0.0) {
    d += 2.0 * M_PI;
  }
  return d - M_PI;
}

inline std::string to_lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

const std::unordered_set<std::string> kTeleopSources = {"teleop", "motion"};
const std::unordered_set<std::string> kNavSources = {"nav", "follow"};
const std::unordered_set<std::string> kRechargeSources = {"recharge"};

struct SectorInfo {
  std::string name;
  bool blocked{false};
  std::optional<double> range_m;
  std::string source;  // empty if none
  double stop_m{0.0};
};

}  // namespace

class SafetyGateNode : public rclcpp::Node
{
public:
  SafetyGateNode()
  : Node("xw_safety_gate")
  {
    declare_parameter<double>("safety_distance", 0.35);
    declare_parameter<double>("nav_safety_distance", 0.28);
    declare_parameter<double>("turn_safety_distance", 0.25);
    declare_parameter<double>("front_angle_deg", 40.0);
    declare_parameter<double>("sector_angle_deg", 40.0);
    declare_parameter<double>("lidar_yaw_offset_rad", 3.141592653589793);
    declare_parameter<double>("lidar_ignore_below_m", 0.20);
    declare_parameter<double>("ultrasonic_stop_m", 0.25);
    declare_parameter<bool>("use_lidar", true);
    declare_parameter<bool>("use_ultrasonic", true);
    declare_parameter<bool>("use_depth", false);
    declare_parameter<std::string>("depth_topic", "/camera/front_up/depth/image_raw");
    declare_parameter<double>("depth_stop_m", 0.40);
    declare_parameter<double>("depth_roi_frac", 0.35);
    declare_parameter<double>("depth_min_valid_m", 0.05);
    declare_parameter<double>("depth_max_valid_m", 4.0);
    declare_parameter<double>("depth_scale", 0.001);
    declare_parameter<int>("depth_min_hits", 40);
    declare_parameter<bool>("enable_teleop_oa", true);
    declare_parameter<double>("avoid_turn_speed", 0.35);
    declare_parameter<double>("avoid_back_speed", 0.18);
    declare_parameter<double>("max_linear_speed", 0.45);
    declare_parameter<double>("max_angular_speed", 0.55);
    declare_parameter<bool>("enable_recharge_pass_through", true);
    declare_parameter<double>("recharge_pass_linear_max", 0.08);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/xw/cmd/gated", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_cmd_ = *msg;
      });

    source_sub_ = create_subscription<std_msgs::msg::String>(
      "/xw/cmd/active_source", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        active_source_ = to_lower(msg->data);
        // trim
        while (!active_source_.empty() &&
          (active_source_.front() == ' ' || active_source_.front() == '\t'))
        {
          active_source_.erase(active_source_.begin());
        }
        while (!active_source_.empty() &&
          (active_source_.back() == ' ' || active_source_.back() == '\t'))
        {
          active_source_.pop_back();
        }
      });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::QoS(10),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        scan_ = *msg;
        have_scan_ = true;
      });

    ultra_sub_ = create_subscription<xw_interfaces::msg::UltrasonicArray>(
      "/ultrasonic_array", rclcpp::QoS(10),
      [this](const xw_interfaces::msg::UltrasonicArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        ultra_ = *msg;
        have_ultra_ = true;
      });

    if (get_parameter("use_depth").as_bool()) {
      const auto topic = get_parameter("depth_topic").as_string();
      rclcpp::QoS depth_qos(1);
      depth_qos.best_effort();
      depth_qos.keep_last(1);
      depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        topic, depth_qos,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          auto z = roi_min_depth(*msg);
          std::lock_guard<std::mutex> lock(mutex_);
          depth_min_ = z;
        });
      RCLCPP_INFO(get_logger(), "depth safety enabled on %s", topic.c_str());
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    safe_pub_ = create_publisher<std_msgs::msg::Bool>("safety_status", 10);
    obs_pub_ = create_publisher<std_msgs::msg::String>("obstacle_status", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() { tick(); });

    RCLCPP_INFO(get_logger(), "safety gate ready (mode-aware teleop/nav, C++)");
  }

private:
  std::optional<double> roi_min_depth(const sensor_msgs::msg::Image & msg)
  {
    const int w = static_cast<int>(msg.width);
    const int h = static_cast<int>(msg.height);
    if (w < 8 || h < 8) {
      return std::nullopt;
    }

    double frac = get_parameter("depth_roi_frac").as_double();
    frac = std::max(0.1, std::min(0.9, frac));
    const int rw = std::max(1, static_cast<int>(w * frac));
    const int rh = std::max(1, static_cast<int>(h * frac));
    const int x0 = (w - rw) / 2;
    const int y0 = (h - rh) / 2;
    const double scale = get_parameter("depth_scale").as_double();
    const double zmin = get_parameter("depth_min_valid_m").as_double();
    const double zmax = get_parameter("depth_max_valid_m").as_double();
    const int need = get_parameter("depth_min_hits").as_int();

    std::string enc = to_lower(msg.encoding);
    const auto & data = msg.data;
    const int step = static_cast<int>(msg.step);

    int hits = 0;
    double best = std::numeric_limits<double>::infinity();

    try {
      if (enc == "16uc1" || enc == "mono16") {
        for (int y = y0; y < y0 + rh; ++y) {
          const int row = y * step;
          for (int x = x0; x < x0 + rw; ++x) {
            const size_t off = static_cast<size_t>(row + x * 2);
            if (off + 1 >= data.size()) {
              continue;
            }
            const uint16_t raw =
              static_cast<uint16_t>(data[off]) |
              (static_cast<uint16_t>(data[off + 1]) << 8);
            if (raw == 0) {
              continue;
            }
            const double z = static_cast<double>(raw) * scale;
            if (z > zmin && z < zmax) {
              ++hits;
              best = std::min(best, z);
            }
          }
        }
      } else if (enc == "32fc1") {
        for (int y = y0; y < y0 + rh; ++y) {
          const int row = y * step;
          for (int x = x0; x < x0 + rw; ++x) {
            const size_t off = static_cast<size_t>(row + x * 4);
            if (off + 3 >= data.size()) {
              continue;
            }
            float zf = 0.0f;
            std::memcpy(&zf, data.data() + off, sizeof(float));
            if (!std::isfinite(zf) || zf <= 0.0f) {
              continue;
            }
            double z = static_cast<double>(zf);
            if (z > 20.0) {
              z *= scale;
            }
            if (z > zmin && z < zmax) {
              ++hits;
              best = std::min(best, z);
            }
          }
        }
      } else {
        std::lock_guard<std::mutex> lock(mutex_);
        return depth_min_;
      }
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      return depth_min_;
    }

    if (hits < need) {
      return std::nullopt;
    }
    return best;
  }

  std::optional<double> sector_min_lidar(
    const sensor_msgs::msg::LaserScan & scan,
    double center_rad, double half_rad) const
  {
    if (!get_parameter("use_lidar").as_bool()) {
      return std::nullopt;
    }
    const double ignore_below = get_parameter("lidar_ignore_below_m").as_double();
    double best = std::numeric_limits<double>::infinity();
    bool any = false;
    double angle = scan.angle_min;
    for (float r : scan.ranges) {
      if (std::abs(ang_diff(angle, center_rad)) <= half_rad) {
        const double lo = std::max(static_cast<double>(scan.range_min), ignore_below);
        if (std::isfinite(r) && r > lo && r < scan.range_max) {
          best = std::min(best, static_cast<double>(r));
          any = true;
        }
      }
      angle += scan.angle_increment;
    }
    if (!any) {
      return std::nullopt;
    }
    return best;
  }

  std::optional<double> ultra_min_for(
    const xw_interfaces::msg::UltrasonicArray & ultra,
    const std::vector<std::string> & keys) const
  {
    if (!get_parameter("use_ultrasonic").as_bool() || ultra.ranges.empty()) {
      return std::nullopt;
    }
    double best = std::numeric_limits<double>::infinity();
    bool any = false;
    for (size_t i = 0; i < ultra.ranges.size(); ++i) {
      std::string label;
      if (i < ultra.labels.size()) {
        label = to_lower(ultra.labels[i]);
      }
      bool match = false;
      for (const auto & k : keys) {
        if (label.find(k) != std::string::npos) {
          match = true;
          break;
        }
      }
      if (!match) {
        continue;
      }
      const float r = ultra.ranges[i];
      if (std::isfinite(r) && r > 0.0f) {
        best = std::min(best, static_cast<double>(r));
        any = true;
      }
    }
    if (!any) {
      return std::nullopt;
    }
    return best;
  }

  static std::pair<std::optional<double>, std::string> pick_range(
    const std::vector<std::pair<std::optional<double>, std::string>> & candidates)
  {
    std::optional<double> best;
    std::string src;
    for (const auto & c : candidates) {
      if (!c.first.has_value()) {
        continue;
      }
      if (!best.has_value() || *c.first < *best) {
        best = c.first;
        src = c.second;
      }
    }
    return {best, src};
  }

  SectorInfo make_sector(
    const std::string & name,
    const std::optional<double> & lidar_m,
    const std::optional<double> & ultra_m,
    const std::optional<double> & depth_m,
    std::optional<double> stop_override,
    double stop_lidar, double stop_ultra, double stop_depth, double turn_stop) const
  {
    auto [dist, src] = pick_range({
      {lidar_m, "lidar"},
      {ultra_m, "ultra"},
      {depth_m, "depth"},
    });
    double stop = stop_lidar;
    if (stop_override.has_value()) {
      stop = *stop_override;
    } else if (src == "ultra") {
      stop = stop_ultra;
    } else if (src == "depth") {
      stop = stop_depth;
    } else {
      stop = (name == "front") ? stop_lidar : turn_stop;
      if (name == "rear") {
        stop = stop_lidar;
      }
    }
    SectorInfo s;
    s.name = name;
    s.range_m = dist;
    s.source = src;
    s.stop_m = stop;
    s.blocked = dist.has_value() && *dist < stop;
    return s;
  }

  struct Sectors {
    SectorInfo front;
    SectorInfo rear;
    SectorInfo left;
    SectorInfo right;
    std::optional<double> depth_m;
  };

  Sectors build_sectors(
    const std::optional<sensor_msgs::msg::LaserScan> & scan,
    const std::optional<xw_interfaces::msg::UltrasonicArray> & ultra,
    const std::optional<double> & depth_min) const
  {
    const double stop_lidar = get_parameter("safety_distance").as_double();
    const double stop_ultra = get_parameter("ultrasonic_stop_m").as_double();
    const double stop_depth = get_parameter("depth_stop_m").as_double();
    const double turn_stop = get_parameter("turn_safety_distance").as_double();
    const double half = get_parameter("sector_angle_deg").as_double() * M_PI / 180.0;
    const double front_half = get_parameter("front_angle_deg").as_double() * M_PI / 180.0;
    const double yaw_off = get_parameter("lidar_yaw_offset_rad").as_double();

    std::optional<double> lidar_front, lidar_left, lidar_right, lidar_rear;
    if (scan.has_value()) {
      lidar_front = sector_min_lidar(*scan, 0.0 + yaw_off, front_half);
      lidar_left = sector_min_lidar(*scan, M_PI / 2.0 + yaw_off, half);
      lidar_right = sector_min_lidar(*scan, -M_PI / 2.0 + yaw_off, half);
      lidar_rear = sector_min_lidar(*scan, M_PI + yaw_off, half);
    }

    std::optional<double> ultra_front, ultra_rear, ultra_left, ultra_right;
    if (ultra.has_value()) {
      ultra_front = ultra_min_for(*ultra, {"front", "f", "前"});
      ultra_rear = ultra_min_for(*ultra, {"rear", "back", "aft", "后"});
      ultra_left = ultra_min_for(*ultra, {"left", "l", "左"});
      ultra_right = ultra_min_for(*ultra, {"right", "r", "右"});
    }

    std::optional<double> d_depth;
    if (get_parameter("use_depth").as_bool()) {
      d_depth = depth_min;
    }

    Sectors out;
    out.front = make_sector(
      "front", lidar_front, ultra_front, d_depth, std::nullopt,
      stop_lidar, stop_ultra, stop_depth, turn_stop);
    out.rear = make_sector(
      "rear", lidar_rear, ultra_rear, std::nullopt, std::nullopt,
      stop_lidar, stop_ultra, stop_depth, turn_stop);
    out.left = make_sector(
      "left", lidar_left, ultra_left, std::nullopt, turn_stop,
      stop_lidar, stop_ultra, stop_depth, turn_stop);
    out.right = make_sector(
      "right", lidar_right, ultra_right, std::nullopt, turn_stop,
      stop_lidar, stop_ultra, stop_depth, turn_stop);
    out.depth_m = d_depth;
    return out;
  }

  geometry_msgs::msg::Twist clamp_speed(geometry_msgs::msg::Twist cmd) const
  {
    const double vmax = get_parameter("max_linear_speed").as_double();
    const double wmax = get_parameter("max_angular_speed").as_double();
    cmd.linear.x = std::max(-vmax, std::min(vmax, cmd.linear.x));
    cmd.angular.z = std::max(-wmax, std::min(wmax, cmd.angular.z));
    return cmd;
  }

  std::pair<geometry_msgs::msg::Twist, bool> apply_teleop(
    const geometry_msgs::msg::Twist & cmd, const Sectors & sectors)
  {
    geometry_msgs::msg::Twist out;
    out.linear.x = cmd.linear.x;
    out.angular.z = cmd.angular.z;
    out = clamp_speed(out);

    const bool front_b = sectors.front.blocked;
    const bool rear_b = sectors.rear.blocked;
    const bool left_b = sectors.left.blocked;
    const bool right_b = sectors.right.blocked;
    const double turn_spd = get_parameter("avoid_turn_speed").as_double();
    const double back_spd = get_parameter("avoid_back_speed").as_double();
    bool ok = true;

    if (rear_b && out.linear.x < 0.0) {
      out.linear.x = 0.0;
      ok = false;
    }
    if (left_b && out.angular.z > 0.0) {
      out.angular.z = 0.0;
      ok = false;
    }
    if (right_b && out.angular.z < 0.0) {
      out.angular.z = 0.0;
      ok = false;
    }

    if (!get_parameter("enable_teleop_oa").as_bool()) {
      if (front_b && out.linear.x > 0.0) {
        out.linear.x = 0.0;
        ok = false;
      }
      return {out, ok};
    }

    if (front_b && out.linear.x > 0.0) {
      out.linear.x = 0.0;
      ok = false;
      const bool can_l = !left_b;
      const bool can_r = !right_b;
      if (can_l && !can_r) {
        out.angular.z = std::abs(turn_spd);
      } else if (can_r && !can_l) {
        out.angular.z = -std::abs(turn_spd);
      } else if (can_l && can_r) {
        if (std::abs(cmd.angular.z) > 1e-3) {
          out.angular.z = std::copysign(std::abs(turn_spd), cmd.angular.z);
        } else {
          out.angular.z = prefer_turn_sign_ * std::abs(turn_spd);
          prefer_turn_sign_ *= -1.0;
        }
      } else if (!rear_b) {
        out.linear.x = -std::abs(back_spd);
        out.angular.z = 0.0;
      } else {
        out.linear.x = 0.0;
        out.angular.z = 0.0;
      }
    }
    return {out, ok};
  }

  std::pair<geometry_msgs::msg::Twist, bool> apply_nav(
    const geometry_msgs::msg::Twist & cmd,
    const Sectors & sectors,
    bool follow_mode = false) const
  {
    geometry_msgs::msg::Twist out;
    out.linear.x = cmd.linear.x;
    out.angular.z = cmd.angular.z;

    const double nav_stop = get_parameter("nav_safety_distance").as_double();
    bool front_b = false;
    if (sectors.front.range_m.has_value() && *sectors.front.range_m < nav_stop) {
      front_b = true;
    }
    if (sectors.front.blocked) {
      front_b = true;
    }

    const bool rear_b = sectors.rear.blocked;
    const bool left_b = sectors.left.blocked;
    const bool right_b = sectors.right.blocked;
    bool ok = true;

    if (front_b && out.linear.x > 0.0) {
      out.linear.x = 0.0;
      ok = false;
    }
    if (out.linear.x < 0.0 && rear_b) {
      out.linear.x = 0.0;
      ok = false;
    }
    // Body-follow: person standing beside the robot trips left/right sectors.
    // Killing yaw toward them makes "see person on edge → never turn" — skip for follow.
    if (!follow_mode) {
      if (left_b && out.angular.z > 0.0) {
        out.angular.z = 0.0;
      }
      if (right_b && out.angular.z < 0.0) {
        out.angular.z = 0.0;
      }
    }
    return {out, ok};
  }

  std::pair<geometry_msgs::msg::Twist, bool> apply_recharge(
    const geometry_msgs::msg::Twist & cmd, const Sectors & sectors) const
  {
    geometry_msgs::msg::Twist out;
    out.linear.x = cmd.linear.x;
    out.angular.z = cmd.angular.z;
    const bool rear_b = sectors.rear.blocked;
    const bool front_b = sectors.front.blocked;
    bool ok = true;
    if (rear_b && out.linear.x < 0.0) {
      out.linear.x = 0.0;
      ok = false;
    }
    if (front_b && out.linear.x > 0.0) {
      if (get_parameter("enable_recharge_pass_through").as_bool()) {
        const double cap = get_parameter("recharge_pass_linear_max").as_double();
        out.linear.x = std::min(out.linear.x, cap);
      } else {
        out.linear.x = 0.0;
        ok = false;
      }
    }
    return {out, ok};
  }

  static nlohmann::json sector_to_json(const SectorInfo & s)
  {
    nlohmann::json j;
    j["name"] = s.name;
    j["blocked"] = s.blocked;
    if (s.range_m.has_value()) {
      j["range_m"] = std::round(*s.range_m * 1000.0) / 1000.0;
    } else {
      j["range_m"] = nullptr;
    }
    if (s.source.empty()) {
      j["source"] = nullptr;
    } else {
      j["source"] = s.source;
    }
    j["stop_m"] = std::round(s.stop_m * 1000.0) / 1000.0;
    return j;
  }

  void tick()
  {
    geometry_msgs::msg::Twist cmd;
    std::string src;
    std::optional<sensor_msgs::msg::LaserScan> scan;
    std::optional<xw_interfaces::msg::UltrasonicArray> ultra;
    std::optional<double> depth_min;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd = last_cmd_;
      src = active_source_;
      if (have_scan_) {
        scan = scan_;
      }
      if (have_ultra_) {
        ultra = ultra_;
      }
      depth_min = depth_min_;
    }

    auto sectors = build_sectors(scan, ultra, depth_min);
    const auto d_depth = sectors.depth_m;

    geometry_msgs::msg::Twist out;
    bool ok = true;
    if (kTeleopSources.count(src)) {
      std::tie(out, ok) = apply_teleop(cmd, sectors);
    } else if (kNavSources.count(src)) {
      std::tie(out, ok) = apply_nav(cmd, sectors, src == "follow");
    } else if (kRechargeSources.count(src)) {
      std::tie(out, ok) = apply_recharge(cmd, sectors);
    } else {
      std::tie(out, ok) = apply_nav(cmd, sectors, false);
    }

    safety_ok_ = ok;
    cmd_pub_->publish(out);

    std_msgs::msg::Bool st;
    st.data = safety_ok_;
    safe_pub_->publish(st);

    const bool blocked = sectors.front.blocked;
    std::string reason = "clear";
    if (blocked) {
      const std::string s =
        sectors.front.source.empty() ? "front" : sectors.front.source;
      if (sectors.front.range_m.has_value()) {
        char buf[64];
        std::snprintf(
          buf, sizeof(buf), "%s:%.2f", s.c_str(), *sectors.front.range_m);
        reason = buf;
      } else {
        reason = s;
      }
    }

    nlohmann::json payload;
    payload["blocked"] = blocked;
    payload["any_sector_blocked"] =
      sectors.front.blocked || sectors.rear.blocked ||
      sectors.left.blocked || sectors.right.blocked;
    payload["safety_ok"] = safety_ok_;
    payload["reason"] = reason;
    if (src.empty()) {
      payload["active_source"] = nullptr;
    } else {
      payload["active_source"] = src;
    }
    if (d_depth.has_value()) {
      payload["depth_m"] = std::round(*d_depth * 1000.0) / 1000.0;
    } else {
      payload["depth_m"] = nullptr;
    }
    payload["sectors"] = {
      {"front", sector_to_json(sectors.front)},
      {"rear", sector_to_json(sectors.rear)},
      {"left", sector_to_json(sectors.left)},
      {"right", sector_to_json(sectors.right)},
    };

    std_msgs::msg::String obs;
    obs.data = payload.dump();
    obs_pub_->publish(obs);
  }

  std::mutex mutex_;
  geometry_msgs::msg::Twist last_cmd_;
  std::string active_source_;
  sensor_msgs::msg::LaserScan scan_;
  bool have_scan_{false};
  xw_interfaces::msg::UltrasonicArray ultra_;
  bool have_ultra_{false};
  std::optional<double> depth_min_;
  bool safety_ok_{true};
  double prefer_turn_sign_{-1.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr source_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<xw_interfaces::msg::UltrasonicArray>::SharedPtr ultra_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safe_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obs_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SafetyGateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
