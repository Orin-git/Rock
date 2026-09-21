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

// Why the depth source is, or is not, contributing. Reported verbatim in
// /obstacle_status so that "the gate cannot see" is never again confusable with
// "the gate sees nothing" -- those two collapsed into the same signal, and on
// 2026-09-20 189 ran for hours in the second state while reporting the first.
//
//   ok                 usable min depth in the ROI
//   no_data            no depth frame has ever arrived (or the sub was never made)
//   stale              last frame is older than `depth_ttl_sec`
//   insufficient_hits  fresh frame, but fewer than `depth_min_hits` valid pixels
//   bad_encoding       encoding is neither 16UC1/MONO16 nor 32FC1
//   bad_image          width or height below 8 px
//   exception          the scan loop threw
//   disabled           `use_depth` is false
struct DepthReading {
  std::optional<double> m;
  std::string state;
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
    // ★ Not a calibrated number: the bridge publishes at ~10 fps, so 0.5 s is a
    // five-frame grace margin. Tune it live with
    //   ros2 param set /xw_safety_gate depth_ttl_sec <s>
    // (0 disables the check) -- apply_nav/tick re-read every parameter each
    // tick, so no restart is needed and the check can be exercised
    // non-destructively.
    declare_parameter<double>("depth_ttl_sec", 0.5);
    // ★ Also uncalibrated defaults. The band is measured from the sector's own
    // STOP threshold, not from nav_safety_distance (see apply_nav), so the
    // slowdown starts at stop_m + band and reaches full speed there.
    declare_parameter<double>("nav_slowdown_band_m", 0.60);
    declare_parameter<double>("nav_slowdown_floor_ratio", 0.25);
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
          // ★ The arrival stamp is taken here, on EVERY frame, whatever
          // roi_min_depth decided about its contents. "A frame arrived whose
          // ROI was unusable" (insufficient_hits) and "no frame has arrived for
          // a minute" (stale) are different facts and must not share a clock.
          //
          // roi_min_depth is lock-free now that its two fail-open branches are
          // gone, so this is the only lock the callback takes.
          const DepthReading reading = roi_min_depth(*msg);
          const double stamp = now().seconds();
          std::lock_guard<std::mutex> lock(mutex_);
          depth_min_ = reading.m;
          depth_state_ = reading.state;
          depth_stamp_sec_ = stamp;
          depth_have_ = true;
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
  // Lock-free: the two branches that used to re-read `depth_min_` under
  // mutex_ are gone (they were the reason this could not be const).
  DepthReading roi_min_depth(const sensor_msgs::msg::Image & msg) const
  {
    const int w = static_cast<int>(msg.width);
    const int h = static_cast<int>(msg.height);
    if (w < 8 || h < 8) {
      return {std::nullopt, "bad_image"};
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
        // ★ Fail CLOSED. This used to `return depth_min_`, i.e. hand the caller
        // the PREVIOUS frame's reading and let it pass for a fresh one. An
        // unrecognised encoding therefore pinned the depth contribution to a
        // value of unknown age for as long as it lasted -- the gate would keep
        // acting on a number it could no longer justify. Dropping the frame
        // instead costs nothing the lidar and ultrasonics do not already cover,
        // and `bad_encoding` below makes the condition loud instead of silent.
        return {std::nullopt, "bad_encoding"};
      }
    } catch (...) {
      // ★ Fail CLOSED, for the same reason.
      return {std::nullopt, "exception"};
    }

    if (hits < need) {
      return {std::nullopt, "insufficient_hits"};
    }
    return {best, "ok"};
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
      // Valid module range starts ~0.30 m; skip NaN / lost / blind ghosts.
      if (std::isfinite(r) && r >= 0.15f) {
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
      ultra_front = ultra_min_for(*ultra, {"front", "前"});
      ultra_rear = ultra_min_for(*ultra, {"rear", "back", "aft", "后"});
      ultra_left = ultra_min_for(*ultra, {"left", "左"});
      ultra_right = ultra_min_for(*ultra, {"right", "右"});
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

  // The predicate apply_nav actually uses to kill forward motion, kept in ONE
  // place so the telemetry field `nav_blocked` cannot drift away from the
  // behaviour. That drift is how `blocked` -- keyed on the WINNING source's
  // stop_m -- came to read false while the published command was already zero:
  // in the band between `nav_safety_distance` and `stop_m` the command is
  // stopped (this predicate) but `sectors.front.blocked` is not yet set.
  bool nav_front_blocked(const Sectors & sectors) const
  {
    if (sectors.front.blocked) {
      return true;
    }
    const double nav_stop = get_parameter("nav_safety_distance").as_double();
    return sectors.front.range_m.has_value() && *sectors.front.range_m < nav_stop;
  }

  struct NavResult {
    geometry_msgs::msg::Twist cmd;
    bool ok{true};
    bool slowdown_applied{false};
    double slowdown_ratio{1.0};
  };

  NavResult apply_nav(
    const geometry_msgs::msg::Twist & cmd,
    const Sectors & sectors,
    bool follow_mode = false) const
  {
    NavResult res;
    geometry_msgs::msg::Twist & out = res.cmd;
    out.linear.x = cmd.linear.x;
    out.angular.z = cmd.angular.z;

    const bool front_b = nav_front_blocked(sectors);

    const bool rear_b = sectors.rear.blocked;
    const bool left_b = sectors.left.blocked;
    const bool right_b = sectors.right.blocked;

    if (front_b && out.linear.x > 0.0) {
      out.linear.x = 0.0;
      res.ok = false;
    }
    if (out.linear.x < 0.0 && rear_b) {
      out.linear.x = 0.0;
      res.ok = false;
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

    // ★ Progress-preserving slowdown band.
    //
    // Keyed on the sector's own STOP threshold -- the threshold of the source
    // that actually won the sector -- and deliberately NOT on
    // `nav_safety_distance`. A band hung off nav_stop (0.28) would sit entirely
    // inside the region where `front_b` is already true, so the block would
    // always win and the band could never produce a non-zero output. That is
    // the "eternal full stop" complaint: prior to this, the ONLY thing between
    // full speed and a hard zero was nothing at all.
    //
    // It only ever SCALES linear.x down. It never sets res.ok = false and never
    // zeroes: /safety_status must keep meaning "am I stopping?", and "slowing"
    // must be a separate signal. The floor is
    // floor_ratio * max_linear_speed = 0.25 * 0.45 = 0.1125 m/s by default,
    // which is two orders of magnitude above xw_cmd_arbiter's isActive() eps of
    // 1e-3 -- so a slowdown can never be mistaken for "this source is gone" and
    // silently erased from the arbiter's source table.
    if (!front_b && sectors.front.range_m.has_value() && out.linear.x > 0.0) {
      const double band = get_parameter("nav_slowdown_band_m").as_double();
      const double floor_ratio =
        std::max(0.0, std::min(1.0, get_parameter("nav_slowdown_floor_ratio").as_double()));
      const double d = *sectors.front.range_m;
      const double stop = sectors.front.stop_m;
      if (band > 0.0 && d < stop + band) {
        const double r = std::max(0.0, std::min(1.0, (d - stop) / band));
        res.slowdown_ratio = floor_ratio + (1.0 - floor_ratio) * r;
        res.slowdown_applied = res.slowdown_ratio < 1.0;
        out.linear.x *= res.slowdown_ratio;
      }
    }
    return res;
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
    std::string depth_state_raw;
    // ★ A plain double, deliberately, NOT rclcpp::Time. `now()` here returns
    // RCL_SYSTEM_TIME (this node does not set use_sim_time), and subtracting a
    // Time of a different clock type throws inside rcl_time_point_subtract --
    // which, in a 20 Hz tick, would take the whole gate down. Both ends of this
    // subtraction come from the same node clock, so seconds-since-epoch is
    // sufficient and has no failure mode.
    double depth_stamp_sec = -1.0;
    bool depth_have = false;
    bool have_scan = false;
    bool have_ultra = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd = last_cmd_;
      src = active_source_;
      have_scan = have_scan_;
      have_ultra = have_ultra_;
      if (have_scan_) {
        scan = scan_;
      }
      if (have_ultra_) {
        ultra = ultra_;
      }
      depth_min = depth_min_;
      depth_state_raw = depth_state_;
      depth_stamp_sec = depth_stamp_sec_;
      depth_have = depth_have_;
    }

    // ── Depth liveness gate (A1) ─────────────────────────────────────────────
    // `depth_min_` used to be a bare optional with no time attached to it: once
    // set it stayed "valid" for ever. Together with the two fail-open returns
    // that roi_min_depth used to have, "I stopped receiving depth frames" and
    // "there is nothing in front of me" collapsed into one signal -- and the
    // gate resolved it in favour of the second. 189 ran exactly that way for
    // hours on 2026-09-20 (the bridge's callbacks had been dead since
    // 1789867268, while /obstacle_status still carried a depth_m as if fresh).
    //
    // Now a reading whose frame is older than `depth_ttl_sec` is discarded
    // outright, and which of the two situations we are in is named in
    // /obstacle_status (`depth.state`, `degraded`). Discarding is safe: the
    // lidar at safety_distance 0.40 and the ultrasonics at 0.25 still guard the
    // same sector, and both fail independently of the camera stack.
    const bool use_depth = get_parameter("use_depth").as_bool();
    const double depth_ttl = get_parameter("depth_ttl_sec").as_double();
    std::string depth_state;
    std::optional<double> depth_used;
    double depth_age = -1.0;
    if (!use_depth) {
      depth_state = "disabled";
    } else if (!depth_have) {
      depth_state = "no_data";
    } else {
      depth_age = now().seconds() - depth_stamp_sec;
      if (depth_ttl > 0.0 && depth_age > depth_ttl) {
        depth_state = "stale";
      } else {
        depth_state = depth_state_raw;
      }
      if (depth_state == "ok") {
        depth_used = depth_min;
      }
    }

    auto sectors = build_sectors(scan, ultra, depth_used);
    // What the gate ACTED on. Distinct from `depth.m` below, which is what the
    // camera last actually said, fresh or not.
    const auto d_depth = sectors.depth_m;
    const bool nav_blocked = nav_front_blocked(sectors);

    geometry_msgs::msg::Twist out;
    bool ok = true;
    NavResult nav_res;
    bool nav_mode = false;
    if (kTeleopSources.count(src)) {
      std::tie(out, ok) = apply_teleop(cmd, sectors);
    } else if (kNavSources.count(src)) {
      nav_res = apply_nav(cmd, sectors, src == "follow");
      nav_mode = true;
    } else if (kRechargeSources.count(src)) {
      std::tie(out, ok) = apply_recharge(cmd, sectors);
    } else {
      nav_res = apply_nav(cmd, sectors, false);
      nav_mode = true;
    }
    if (nav_mode) {
      out = nav_res.cmd;
      ok = nav_res.ok;
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
    // ★ `nav_blocked` reports the predicate apply_nav ACTUALLY used. `blocked`
    // above is left exactly as it was (sectors.front.blocked, keyed on the
    // winning source's stop_m) -- this addition exists so the two can be
    // compared rather than silently disagreeing in the band between
    // nav_safety_distance and stop_m. Note it is a property of the sectors, so
    // it is reported for every mode, including teleop.
    payload["nav_blocked"] = nav_blocked;
    if (d_depth.has_value()) {
      payload["depth_m"] = std::round(*d_depth * 1000.0) / 1000.0;
    } else {
      payload["depth_m"] = nullptr;
    }
    // ★ The whole point of A1: make "cannot see" distinguishable from "sees
    // nothing". `depth.m` is the last thing the camera actually said, with its
    // age; `depth.used_m` duplicates the effective `depth_m` so a reader never
    // has to work out which of the two the gate obeyed.
    {
      nlohmann::json dj;
      if (depth_have && depth_min.has_value()) {
        dj["m"] = std::round(*depth_min * 1000.0) / 1000.0;
      } else {
        dj["m"] = nullptr;
      }
      if (depth_have) {
        dj["age_sec"] = std::round(depth_age * 10.0) / 10.0;
      } else {
        dj["age_sec"] = nullptr;
      }
      dj["ttl_sec"] = depth_ttl;
      dj["valid"] = (depth_state == "ok");
      dj["state"] = depth_state;
      if (d_depth.has_value()) {
        dj["used_m"] = std::round(*d_depth * 1000.0) / 1000.0;
      } else {
        dj["used_m"] = nullptr;
      }
      payload["depth"] = dj;
    }
    payload["nav_slowdown"] = {
      {"applied", nav_mode && nav_res.slowdown_applied},
      {"ratio", std::round(nav_res.slowdown_ratio * 1000.0) / 1000.0},
      {"band_m", get_parameter("nav_slowdown_band_m").as_double()},
    };
    // Sources that are enabled but are not currently contributing anything
    // usable. ⚠️ `have_scan_` / `have_ultra_` are sticky -- they are never
    // aged, so this catches "never arrived" but NOT "arrived and then stopped".
    // Only the depth leg has a liveness clock (depth_ttl_sec); giving the lidar
    // and ultrasonics one is a separate change and must not be assumed here.
    {
      nlohmann::json degraded = nlohmann::json::array();
      if (get_parameter("use_lidar").as_bool() && !have_scan) {
        degraded.push_back("lidar:no_data");
      }
      if (get_parameter("use_ultrasonic").as_bool() && !have_ultra) {
        degraded.push_back("ultrasonic:no_data");
      }
      if (use_depth && depth_state != "ok") {
        degraded.push_back("depth:" + depth_state);
      }
      payload["degraded"] = degraded;
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
  std::string depth_state_;
  double depth_stamp_sec_{-1.0};
  bool depth_have_{false};
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
