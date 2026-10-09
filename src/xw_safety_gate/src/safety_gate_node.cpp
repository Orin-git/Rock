// C++ port of xw_safety_gate (Python -> C++).
// Logic, topics, node name and parameters kept identical.
// Original Python: python_legacy/xw_safety_gate/safety_gate_node.py (kept as backup).
#include <algorithm>
#include <cmath>
#include <cstdint>
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
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
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

// Why the depth evidence is, or is not, contributing. Reported verbatim in
// /obstacle_status so that "the gate cannot see" is never again confusable with
// "the gate sees nothing" -- those two collapsed into the same signal, and on
// 2026-09-20 189 ran for hours in the second state while reporting the first.
//
// ★ 2026-09-21: the source moved from front_up's height band to the RAW
// front_down cloud, and the test from "inside a fixed height band" to "above the
// measured FLOOR PLANE". band_min_range() carries the full account; in one line:
// front_up looks level, so its floor only enters the frame 1.64 m out and a
// fixed band that excludes the floor must also exclude everything shorter than
// ~0.27 m -- which is the whole point. front_down looks DOWN 27 deg, so the floor
// is in view from 0.37 m, every frame re-measures it, and an obstacle is simply a
// point standing above that plane.
//
// The states are about the FLOOR MODEL first and about the STREAM second:
//
//   ok                  >= `depth_min_points` points stand more than
//                       `depth_floor_h_min` above the floor; `m` is the nearest
//                       range among them
//   clear               nothing stands above the floor. ★ NOT a failure: in an
//                       empty corridor this is the correct reading, and it is
//                       trustworthy precisely because the raw stream is alive.
//   insufficient_points 1..depth_min_points-1 such points -- too few to act on,
//                       but not nothing. Reported, never acted on.
//   no_floor_model      ★ NEW. A frame arrived, but the floor plane could not be
//                       solved (too few points inside the fit window). Without a
//                       floor there is no height scale and therefore NO READING:
//                       this is emphatically not "clear". Measured 2026-09-21,
//                       the fit fails when the robot pitch moves ~15 deg away
//                       from the calibration prior while the first fit pass uses
//                       a narrow window.
//   no_data             no frame has ever arrived (or `use_depth` was false when
//                       the subscription was created)
//   stale               frames have stopped: none for `depth_ttl_sec`
//   no_camera_data      frames arrive but carry almost nothing: the cloud is
//                       empty because the camera chain died, and must not be
//                       read as "clear"
//   disabled            `use_depth` is false
struct Band {
  std::optional<double> min_range;
  int points{0};                  // points standing above the floor
  uint64_t raw_points{0};         // valid points in the frame (liveness + scale)
  bool floor_ok{false};           // did the floor plane solve?
  double floor_a{0.0};            // y = a + b*z, optical frame (Y down-positive)
  double floor_b{0.0};
  long floor_fit_points{0};       // points the fit actually used
  bool truncated{false};          // short buffer: the frame is not whole
};

// Everything band_min_range() needs from the parameter server, read once per
// frame in the callback and passed in. Keeping it a value (rather than reading
// parameters inside the scan) is what lets that function stay lock-free and
// allocation-free while looping over every point in the cloud.
struct FloorCfg {
  double a0{0.3489};        // ★ CALIBRATED prior, see safety_gate.yaml
  double b0{-0.5126};
  double h_min{0.05};
  double clip_wide{0.25};
  double clip_narrow{0.06};
  int min_fit_points{2000};
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
    // ★ 2026-10-08: front-group ultrasonic non-finite readings are treated
    // as BLOCKED instead of being silently dropped (see ultra_min_for and
    // build_sectors). Live-tunable escape hatch; false restores the old
    // drop-the-reading behaviour.
    declare_parameter<bool>("ultrasonic_deadzone_is_blocked", true);
    declare_parameter<bool>("use_lidar", true);
    declare_parameter<bool>("use_ultrasonic", true);
    // ── Depth evidence: the floor plane of the DOWNWARD camera ──────────────
    // 2026-09-21. band_min_range() carries the full account; in one line: the
    // image ROI selected by depression angle and for a level camera every floor
    // pixel shares the same optical Y, and a fixed height band on that same
    // level camera cannot admit anything shorter than ~0.27 m. front_down looks
    // 27 deg down, so the floor is a live, re-measurable reference surface.
    declare_parameter<bool>("use_depth", false);
    // ★ The RAW front_down cloud, deliberately not xw_pc_nav_filter's output.
    //   pc_nav_filter caps output at max_points_out (2500) after a voxel filter,
    //   which can drop the very returns the low-obstacle test depends on, and it
    //   is shared with the costmap, so its death blinds both at once.
    declare_parameter<std::string>(
      "depth_pc_topic", "/camera/front_down/depth/points");
    // ★ Liveness floor, not a judgement threshold. Measured 2026-09-21: the
    // front_down cloud carries ~275000 valid points in a corridor (of 307200
    // pixels), so 1000 is 0.4% of normal -- it catches "the camera chain is
    // dead", and nothing subtler. The TTL is the primary defence.
    declare_parameter<int>("depth_raw_floor_points", 1000);
    // ── Floor-plane model, y = a + b*z in the optical frame (Y down-positive) ─
    // ★★ ALL SIX CALIBRATED on 189, 2026-09-21, over 6+ independent frames.
    //    Measured plane: H = 0.3097..0.3112 m, pitch = -27.10..-27.19 deg.
    //      a0 = H / cos(pitch)  = 0.3105 / cos(27.14 deg) = 0.3489
    //      b0 = -tan(pitch)     = -0.5126
    //    ★ The sign of b0 is negative and is NOT a typo: Y is down-positive, so
    //      a camera pitched down sees the floor's y DECREASE with z. Cross-check:
    //      at z = 0.368 m this plane gives y = +0.160, and the measured global
    //      y_max of the cloud is +0.162.
    declare_parameter<double>("depth_floor_a0", 0.3489);
    declare_parameter<double>("depth_floor_b0", -0.5126);
    // ★ How far above the floor a point must stand to count as an obstacle.
    //   Measured noise floor 2026-09-21 on an empty corridor: h > 0.020 -> 0.158%
    //   of points; h > 0.030 -> 0.020%; h > 0.050 -> EXACTLY ZERO. A 10 cm box at
    //   0.565 m gave 10545 points. 0.05 sits at the noise floor with ~2x margin.
    declare_parameter<double>("depth_floor_h_min", 0.05);
    // ★ Two-stage fit window, in metres of perpendicular distance to the CURRENT
    //   estimate of the plane. The wide first pass is what buys pitch tolerance:
    //   measured 2026-09-21 on real clouds, a narrow-only schedule [0.06, 0.06]
    //   loses the floor entirely at +15 deg of pitch error and biases H by
    //   +3.6 cm (dangerous: it lifts the plane, so floor points read as
    //   obstacles) at -10 deg. [0.25, 0.06] tracked every case to within 0.1 cm.
    //   The wide window is safe against large obstacles for a geometric reason:
    //   it admits only the bottom 25 cm of a wall, and a wall's bottom edge lies
    //   ON the floor plane, so it cannot tilt the fit. Measured with a synthetic
    //   wall covering 30% of the cloud: [0.25, 0.06] -> H off by 0.3 cm, while
    //   plain iterative least squares -> off by 3.6 cm.
    declare_parameter<double>("depth_floor_clip_wide_m", 0.25);
    declare_parameter<double>("depth_floor_clip_narrow_m", 0.06);
    // ★ Below this many in-window points the frame yields no floor model, and
    //   the reading becomes `no_floor_model` rather than `clear`. Measured: a
    //   healthy frame puts ~265000 points inside the wide window.
    declare_parameter<int>("depth_floor_min_fit_points", 2000);
    // ★ NOT CALIBRATED -- 0 leaves the discrepancy test off. Set it from the
    // measured approach profile, live:
    //   ros2 param set /xw_safety_gate depth_low_obstacle_m <m>
    declare_parameter<double>("depth_low_obstacle_m", 0.0);
    // ★ NOT CALIBRATED either. Only consulted when depth_low_obstacle_m > 0.
    declare_parameter<double>("depth_lidar_margin_m", 0.15);
    // ★ NOT CALIBRATED. Floor on what counts as a real target, so a couple of
    // stray returns cannot drive the gate. Measured 2026-09-20: a 0.35 m box
    // 1.5 m ahead gave 25-37 points in the band.
    declare_parameter<int>("depth_min_points", 5);
    declare_parameter<double>("depth_stop_m", 0.40);
    // ★ Placeholder, NOT measured. The source changed from the image (asumed
    // ~10 fps) to points_nav, measured at 4.31 Hz MEAN on 2026-09-20; the worst
    // inter-frame gap was not measured. Derive this from that gap before
    // trusting it. 0 disables the check. Tune live with
    //   ros2 param set /xw_safety_gate depth_ttl_sec <s>
    declare_parameter<double>("depth_ttl_sec", 0.7);
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
      const auto pc_topic = get_parameter("depth_pc_topic").as_string();
      rclcpp::QoS pc_qos(1);
      pc_qos.best_effort();
      pc_qos.keep_last(1);
      depth_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pc_topic, pc_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          // ★ The arrival stamp is taken here, on EVERY frame, whatever the
          // frame turned out to contain. "A frame arrived whose floor held
          // nothing" (clear) and "no frame has arrived for a second" (stale)
          // are different facts and must not share a clock.
          //
          // ★ ONE subscription now carries both jobs. The same walk that solves
          // the floor also counts every valid point, so `raw_points` is the
          // liveness heartbeat that used to need a second subscription -- an
          // empty CLEARING and a dead CAMERA stay distinguishable without
          // paying for a second 3.3 MB deserialisation of the same message.
          //
          // ★ The floor parameters are read HERE, per frame, rather than once at
          // construction. They are the calibration knobs (see the plan's 11.4),
          // and reading them per frame is what makes them tunable live with
          // `ros2 param set` instead of requiring a restart. Six parameter reads
          // at 8 Hz is not a cost worth trading that for.
          FloorCfg cfg;
          cfg.a0 = get_parameter("depth_floor_a0").as_double();
          cfg.b0 = get_parameter("depth_floor_b0").as_double();
          cfg.h_min = get_parameter("depth_floor_h_min").as_double();
          cfg.clip_wide = get_parameter("depth_floor_clip_wide_m").as_double();
          cfg.clip_narrow = get_parameter("depth_floor_clip_narrow_m").as_double();
          cfg.min_fit_points =
            get_parameter("depth_floor_min_fit_points").as_int();
          // band_min_range takes no lock, so this is the only lock the callback
          // takes.
          const Band band = band_min_range(*msg, cfg);
          const double stamp = now().seconds();
          std::lock_guard<std::mutex> lock(mutex_);
          depth_min_ = band.min_range;
          depth_points_ = band.points;
          depth_raw_points_ = band.raw_points;
          depth_floor_ok_ = band.floor_ok;
          depth_floor_a_ = band.floor_a;
          depth_floor_b_ = band.floor_b;
          depth_floor_fit_points_ = band.floor_fit_points;
          depth_truncated_ = band.truncated;
          depth_stamp_sec_ = stamp;
          depth_have_ = true;
        });
      RCLCPP_INFO(
        get_logger(), "depth evidence: raw cloud %s (floor-plane test)",
        pc_topic.c_str());
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
  // Lock-free (no locks; the few parameters it needs arrive by value) and
  // allocation-free.
  //
  // ★ Why this parses the DOWNWARD camera's raw cloud (2026-09-21).
  //
  // The history is worth keeping, because each earlier attempt failed for a
  // STRUCTURAL reason rather than a tuning reason, and the same trap is still
  // one parameter away:
  //
  //   1. A centred rectangle of the depth IMAGE. Dead on arrival on 189.
  //      Measured 2026-09-20, two independent samples of
  //      /camera/front_up/depth/image_raw: rows 0-319 carry exactly zero valid
  //      pixels; rows 320-479 carry 40081, z in 1.285..2.607 m. A centred
  //      depth_roi_frac 0.35 lands on rows 156-323 and meets that window on four
  //      rows, so `hits` never reached depth_min_hits and the depth leg degraded
  //      silently to lidar+ultrasonic. It had never once fired, and it could not
  //      have: for a LEVEL camera every floor pixel shares the same optical Y,
  //      so an image rectangle selects by depression angle and mixes floor with
  //      obstacle at every distance.
  //
  //   2. A fixed HEIGHT BAND on that same level camera (xw_pc_nav_filter's
  //      y in [-0.80, 0.40]). This one does separate floor from obstacle, and it
  //      works -- but excluding the floor requires the band to sit below the
  //      camera (measured 0.626 m), and the floor only enters a level camera's
  //      view at 1.64 m. Measured 2026-09-21: admitting a 0.10 m obstacle at
  //      1.64 m needs y_max >= 0.66, while the floor's own y there starts at
  //      0.666 -- about a centimetre of separation, against centimetres of floor
  //      relief. So that band sees >= 0.27 m objects beyond 1.64 m, which is the
  //      range the lidar already covers best, and nothing shorter.
  //
  //   3. THIS: the DOWNWARD camera's raw cloud, with the floor treated as a
  //      reference SURFACE rather than as something to exclude. front_down is
  //      pitched 27 deg down and mounted 0.311 m up, so the floor is in view
  //      from 0.368 m out and every frame re-measures it to an rms of 4.8 mm.
  //      An obstacle is then simply a point standing above that plane -- and the
  //      near field, the only place a low obstacle matters, is where the data is
  //      cleanest (z < 1.0 m: 220262 points, h in -0.013..+0.011 m).
  //
  // ★ The measured noise floor, which is what makes a 5 cm threshold defensible
  //   rather than hopeful. Empty corridor, 2026-09-21: h > 0.020 -> 0.158% of
  //   points, h > 0.030 -> 0.020%, h > 0.050 -> EXACTLY ZERO. A 0.10 m box at
  //   0.565 m produced 10545 points. Two orders of magnitude of separation.
  //
  // ★ Why the fit is seeded from a CALIBRATED prior and clipped, instead of
  //   being an unweighted least squares over the whole cloud. Measured
  //   2026-09-21 on real frames with a synthetic wall injected over 30% of the
  //   points: plain iterative least squares is dragged 3.6 cm off (H 0.3099 ->
  //   0.3462); this seed-and-clip schedule stays within 0.3 cm. The mechanism is
  //   geometric -- the wide window admits only the bottom 25 cm of a wall, and a
  //   wall's bottom edge lies ON the floor plane.
  //
  // ★ And why the FIRST pass is WIDE. A single narrow window [0.06, 0.06] is
  //   what "just trust the prior" suggests, and it is wrong: measured, it loses
  //   the floor entirely at +15 deg of pitch error, and biases H by +3.6 cm at
  //   -10 deg. That bias is the dangerous direction -- it lifts the plane, so
  //   the floor itself starts reading as an obstacle. [0.25, 0.06] tracked every
  //   case to within 0.1 cm.
  //
  // The range returned is the 3-D norm, chosen so that it is directly
  // comparable with the lidar's range in the discrepancy test in tick().

  // Where x/y/z live in the buffer, and whether the buffer can hold them.
  // Resolved from the message's own field table: point_step is 12 today (xyz
  // only), but a vendor change must not silently turn this into garbage.
  struct PcGeom {
    int ox{-1};
    int oy{-1};
    int oz{-1};
    size_t step{0};
    size_t row_step{0};
    size_t need{0};
    bool ok{false};
  };

  static PcGeom resolve_geom(const sensor_msgs::msg::PointCloud2 & msg)
  {
    PcGeom g;
    if (msg.point_step < sizeof(float) || msg.data.empty()) {
      return g;
    }
    for (const auto & f : msg.fields) {
      if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) {
        continue;
      }
      if (f.name == "x") {
        g.ox = static_cast<int>(f.offset);
      } else if (f.name == "y") {
        g.oy = static_cast<int>(f.offset);
      } else if (f.name == "z") {
        g.oz = static_cast<int>(f.offset);
      }
    }
    if (g.ox < 0 || g.oy < 0 || g.oz < 0) {
      return g;
    }
    g.need =
      static_cast<size_t>(std::max(std::max(g.ox, g.oy), g.oz)) + sizeof(float);
    if (g.need > msg.point_step) {
      return g;
    }
    g.step = msg.point_step;
    g.row_step =
      msg.row_step ? static_cast<size_t>(msg.row_step) : g.step * msg.width;
    g.ok = true;
    return g;
  }

  // Visit every finite xyz in the frame. Returns false when the frame is
  // TRUNCATED: a short buffer means the frame is not whole, and half a frame is
  // not a frame for something a safety gate decides on.
  template<typename Fn>
  static bool each_point(
    const sensor_msgs::msg::PointCloud2 & msg, const PcGeom & g, Fn && fn)
  {
    for (uint32_t row = 0; row < msg.height; ++row) {
      const size_t rbase = static_cast<size_t>(row) * g.row_step;
      for (uint32_t col = 0; col < msg.width; ++col) {
        const size_t off = rbase + static_cast<size_t>(col) * g.step;
        if (off + g.need > msg.data.size()) {
          return false;
        }
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        std::memcpy(
          &x, msg.data.data() + off + static_cast<size_t>(g.ox), sizeof(float));
        std::memcpy(
          &y, msg.data.data() + off + static_cast<size_t>(g.oy), sizeof(float));
        std::memcpy(
          &z, msg.data.data() + off + static_cast<size_t>(g.oz), sizeof(float));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        fn(x, y, z);
      }
    }
    return true;
  }

  Band band_min_range(
    const sensor_msgs::msg::PointCloud2 & msg, const FloorCfg & cfg) const
  {
    Band out;
    const PcGeom g = resolve_geom(msg);
    if (!g.ok) {
      return out;
    }

    // ── Passes 1 and 2: solve the floor plane from the calibrated prior ───────
    // Each pass is a clipped least-squares over the points lying within `clip`
    // metres of the CURRENT estimate. The first clip is deliberately WIDE: it is
    // what tolerates the robot's pitch moving away from the calibration pose,
    // and it is safe against large obstacles because it admits only the bottom
    // slice of anything standing on the floor.
    double a = cfg.a0;
    double b = cfg.b0;
    const double clips[2] = {cfg.clip_wide, cfg.clip_narrow};
    for (int pass = 0; pass < 2; ++pass) {
      const double clip = clips[pass];
      const double sect = std::sqrt(1.0 + b * b);
      double sz = 0.0;
      double sy = 0.0;
      double szz = 0.0;
      double szy = 0.0;
      long n = 0;
      const bool whole = each_point(
        msg, g,
        [&](float /*xf*/, float yf, float zf) {
          const double zd = static_cast<double>(zf);
          if (zd <= 0.0) {
            return;
          }
          const double yd = static_cast<double>(yf);
          if (std::fabs((a + b * zd - yd) / sect) >= clip) {
            return;
          }
          ++n;
          sz += zd;
          sy += yd;
          szz += zd * zd;
          szy += zd * yd;
        });
      out.floor_fit_points = n;
      if (!whole) {
        out.truncated = true;
        out.floor_ok = false;
        return out;
      }
      if (n < cfg.min_fit_points) {
        // No floor => no height scale => NO READING. floor_ok stays false and
        // the caller reports `no_floor_model`; this must never fall through to
        // `clear`, which would mean "nothing ahead" on the strength of nothing
        // at all.
        out.floor_ok = false;
        return out;
      }
      const double dn = static_cast<double>(n);
      const double det = dn * szz - sz * sz;
      if (std::fabs(det) < 1e-9) {
        out.floor_ok = false;
        return out;
      }
      const double nb = (dn * szy - sz * sy) / det;
      const double na = (sy - nb * sz) / dn;
      if (!std::isfinite(na) || !std::isfinite(nb)) {
        out.floor_ok = false;
        return out;
      }
      a = na;
      b = nb;
    }
    out.floor_ok = true;
    out.floor_a = a;
    out.floor_b = b;

    // ── Pass 3: how far each point stands above the floor ────────────────────
    // h = (a + b*z - y) / sqrt(1 + b^2) is the perpendicular distance, positive
    // above the plane. `raw_points` is counted HERE rather than by a second
    // subscription: an empty CLEARING and a dead CAMERA have to stay
    // distinguishable, and this walk already touches every point.
    //
    // h_min is floored at 1 cm. At exactly 0 the floor's own noise crosses the
    // test -- measured p50 of h is -0.0002 m -- and the gate would brake on the
    // floor for ever.
    const double h_min = std::max(0.01, cfg.h_min);
    const double sect = std::sqrt(1.0 + b * b);
    const bool whole = each_point(
      msg, g,
      [&](float xf, float yf, float zf) {
        const double zd = static_cast<double>(zf);
        if (zd <= 0.0) {
          return;
        }
        ++out.raw_points;
        const double yd = static_cast<double>(yf);
        if ((a + b * zd - yd) / sect <= h_min) {
          return;
        }
        ++out.points;
        const double xd = static_cast<double>(xf);
        const double r = std::sqrt(xd * xd + yd * yd + zd * zd);
        if (!out.min_range.has_value() || r < *out.min_range) {
          out.min_range = r;
        }
      });
    if (!whole) {
      out.truncated = true;
      out.floor_ok = false;
      out.points = 0;
      out.min_range.reset();
    }
    return out;
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
    const std::vector<std::string> & keys,
    bool * invalid_seen = nullptr) const
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
      // ★ 2026-10-08 fail-closed fix. The ultrasonic driver encodes THREE
      // distinct facts as a non-finite reading: a measured value below 30 cm,
      // a probe-lost status (0x00), and the 0x01 blind zone. The old line here
      // ("skip NaN / lost / blind ghosts") silently dropped all three, so
      // once an obstacle entered the <30 cm dead zone the front group produced
      // no reading at all and the sector fell back to the lidar's 0.40 m --
      // which at the bumper is only ~0.15 m -- opening a legal-pass gap
      // (measured on 189, 2026-10-08, 807.958 s). A non-finite front-group
      // reading is now reported through `invalid_seen` and build_sectors
      // treats the front sector as BLOCKED. No distance is forged.
      if (std::isfinite(r) && r >= 0.15f) {
        best = std::min(best, static_cast<double>(r));
        any = true;
      } else {
        if (invalid_seen) *invalid_seen = true;
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
    // The lidar's OWN front reading, kept separate from `front.range_m` (which
    // is the minimum over every source). The discrepancy test in tick() has to
    // ask what the LIDAR saw, not what the sector settled on -- once depth has
    // won the sector, `front.range_m` is the depth value and asking it whether
    // the lidar agrees would be circular.
    std::optional<double> lidar_front_m;
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
    // ★ Front group only: a non-finite reading there is a failure to
    // exclude, see the fail-closed note in ultra_min_for(). rear/left/right
    // keep the old behaviour (no flag passed).
    bool ultra_front_invalid = false;
    if (ultra.has_value()) {
      ultra_front = ultra_min_for(*ultra, {"front", "前"}, &ultra_front_invalid);
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
    // ★ 2026-10-08: front fail-closed. A non-finite ultrasonic reading in
    // the front group means the obstacle is inside the <30 cm dead zone, the
    // probe is lost, or the module reports its blind zone -- in every case the
    // front sector is forced BLOCKED with no forged distance. stop_m is the
    // ultrasonic stop threshold so telemetry and the slowdown band stay keyed
    // to the source that caused the block. Disable live with
    //   ros2 param set /xw_safety_gate ultrasonic_deadzone_is_blocked false
    if (ultra_front_invalid && get_parameter("ultrasonic_deadzone_is_blocked").as_bool()) {
      out.front.blocked = true;
      out.front.source = "ultra";
      out.front.range_m = std::nullopt;
      out.front.stop_m = stop_ultra;
    }
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
    out.lidar_front_m = lidar_front;
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
    int depth_points = 0;
    // ★ A plain double, deliberately, NOT rclcpp::Time. `now()` here returns
    // RCL_SYSTEM_TIME (this node does not set use_sim_time), and subtracting a
    // Time of a different clock type throws inside rcl_time_point_subtract --
    // which, in a 20 Hz tick, would take the whole gate down. Both ends of this
    // subtraction come from the same node clock, so seconds-since-epoch is
    // sufficient and has no failure mode.
    double depth_stamp_sec = -1.0;
    bool depth_have = false;
    uint64_t depth_raw_points = 0;
    bool depth_floor_ok = false;
    double depth_floor_a = 0.0;
    double depth_floor_b = 0.0;
    long depth_floor_fit_points = 0;
    bool depth_truncated = false;
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
      depth_points = depth_points_;
      depth_stamp_sec = depth_stamp_sec_;
      depth_have = depth_have_;
      depth_raw_points = depth_raw_points_;
      depth_floor_ok = depth_floor_ok_;
      depth_floor_a = depth_floor_a_;
      depth_floor_b = depth_floor_b_;
      depth_floor_fit_points = depth_floor_fit_points_;
      depth_truncated = depth_truncated_;
    }

    // ── Depth liveness gate (A1) ─────────────────────────────────────────────
    // `depth_min_` used to be a bare optional with no time attached to it: once
    // set it stayed "valid" for ever. Together with the two fail-open returns
    // that the old image reader had, "I stopped receiving depth frames" and
    // "there is nothing in front of me" collapsed into one signal -- and the
    // gate resolved it in favour of the second. 189 ran exactly that way for
    // hours on 2026-09-20 (the bridge's callbacks had been dead since
    // 1789867268, while /obstacle_status still carried a depth_m as if fresh).
    //
    // ★ The fail-closed anchor MOVED on 2026-09-21, and this is the one thing
    // about this rewrite that must not be read the old way. Under the image-ROI
    // reading, "0 usable pixels" meant "I cannot see" and was distrusted, so
    // the anchor was a COUNT (`hits < depth_min_hits` => unusable). Under the
    // floor-plane reading it is the opposite: 0 points standing above the floor
    // means the corridor ahead is clear, and that is the single most trustworthy
    // statement this camera makes -- the floor is not excluded by a filter that
    // could also drop the obstacle, it IS the ruler the test is measured
    // against, so "nothing stands above it" is a positive reading about the
    // space rather than missing data. Distrust therefore attaches to the FRAME,
    // and there are now THREE distinct ways a frame can fail to inform:
    //   * no frame within `depth_ttl_sec`                      -> stale
    //   * a frame arrived but no floor could be solved from it  -> no_floor_model
    //   * the frame carries essentially no points              -> no_camera_data
    // The middle one is new and is the reason this comment had to change: a
    // frame whose fit failed is not a frame that said "clear", it is a frame
    // that said NOTHING, and conflating the two is precisely the failure this
    // whole family of rewrites exists to prevent.
    // Discarding is safe: the lidar at safety_distance 0.40 and the ultrasonics
    // at 0.25 still guard the same sector, and both fail independently of the
    // camera stack.
    const bool use_depth = get_parameter("use_depth").as_bool();
    const double depth_ttl = get_parameter("depth_ttl_sec").as_double();
    const int depth_min_points = get_parameter("depth_min_points").as_int();
    const int raw_floor = get_parameter("depth_raw_floor_points").as_int();
    const double now_sec = now().seconds();
    const auto fresh = [depth_ttl, now_sec](double stamp) {
        return depth_ttl <= 0.0 || (now_sec - stamp) <= depth_ttl;
      };

    std::string depth_state;
    std::optional<double> depth_used;
    double depth_age = -1.0;
    if (!use_depth) {
      depth_state = "disabled";
    } else if (!depth_have) {
      depth_state = "no_data";
    } else {
      depth_age = now_sec - depth_stamp_sec;
      if (!fresh(depth_stamp_sec)) {
        depth_state = "stale";
      } else if (!depth_floor_ok) {
        // ★ ORDER MATTERS. A frame arrived but yielded no floor, so there is no
        // height scale and therefore no reading at all. This must be tested
        // BEFORE the liveness branch below, because a failed fit leaves
        // raw_points at 0 -- the count is taken in the pass that never ran --
        // and testing liveness first would report a pitch or geometry problem
        // as `no_camera_data`, a diagnosis that sounds like hardware and would
        // send the next reader after the wrong thing entirely.
        depth_state = "no_floor_model";
      } else if (depth_raw_points <
        static_cast<uint64_t>(std::max(0, raw_floor)))
      {
        depth_state = "no_camera_data";
      } else if (depth_points >= depth_min_points) {
        depth_state = "ok";
      } else if (depth_points == 0) {
        depth_state = "clear";
      } else {
        depth_state = "insufficient_points";
      }
      if (depth_state == "ok") {
        depth_used = depth_min;
      }
    }

    auto sectors = build_sectors(scan, ultra, depth_used);

    // ── Discrepancy test: an obstacle that only the camera can see ───────────
    // The lidar scans a horizontal plane at roughly 0.22 m; anything shorter
    // than that is invisible to it and equally invisible to the ultrasonics
    // (0.25 m range, and soft or sloped surfaces often return nothing at all).
    // The height band does see it. So: the camera reports something inside the
    // danger distance AND the lidar, looking down the same bearing, either sees
    // nothing or sees something meaningfully farther away => the two disagree,
    // and the disagreement is the finding. The camera is not being trusted over
    // the lidar here -- a bare "depth < 0.40" threshold would fire on the floor
    // and on anything the lidar already knows about; requiring the lidar to
    // disagree is what makes this specific to the blind spot.
    //
    // `depth_low_obstacle_m <= 0` disables the whole test, which is the shipped
    // default until the distance is calibrated on the robot (see the plan's
    // 11.4: it must not be set from a single measurement).
    bool low_obstacle = false;
    const double low_m = get_parameter("depth_low_obstacle_m").as_double();
    if (low_m > 0.0 && depth_state == "ok" && depth_used.has_value() &&
      get_parameter("use_lidar").as_bool())
    {
      const double margin =
        std::max(0.0, get_parameter("depth_lidar_margin_m").as_double());
      const bool lidar_blind =
        !sectors.lidar_front_m.has_value() ||
        (*sectors.lidar_front_m > *depth_used + margin);
      low_obstacle = (*depth_used < low_m) && lidar_blind;
    }
    if (low_obstacle) {
      // Winning the sector outright is what turns the finding into a brake:
      // `blocked`, `range_m` and the `stop_m` that apply_nav keys on are all
      // read from these fields. `stop_m` itself needs no write -- it is already
      // the depth source's `depth_stop_m`, because `source` is being set to
      // "depth" right here.
      sectors.front.blocked = true;
      sectors.front.range_m = depth_used;
      sectors.front.source = "depth";
    }

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
    if (low_obstacle && depth_used.has_value()) {
      // Named distinctly from the source-tagged form below, because this stop
      // is a camera-only finding. A reader of /obstacle_status must be able to
      // tell "the lidar saw something" apart from "only the camera did" --
      // plan 11.4/11.5 calibrate and accept on this exact string, so it is
      // load-bearing, not a label.
      char buf[64];
      std::snprintf(
        buf, sizeof(buf), "low_obstacle_depth_only:%.2f", *depth_used);
      reason = buf;
    } else if (blocked) {
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
    // U9 残余「进遥测」：前向盲区公示字段（静态、带口径标签，不参与任何判决）。
    // 数字口径 = 2026-09-15 实测姿态；出处 FINDING_u9_u10_2026-10-08.md §U9-6/T5。
    payload["blind_zone"] = {
      {"convention", "measured-2026-09-15"},
      {"D_max_m", 1.18},
      {"h_lo_m", 0.19},
      {"h_hi_m", 0.41},
      {"lidar_in_zone_D", {0.53, 0.98}},
      {"ultrasonic", "unmeasured"},
      {"measured_on", "2026-10-08"},
    };
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
      // ★ `points` is how much stands above the floor and `raw_points` is the
      // whole frame. They answer different questions and both are needed to read
      // `state`: points==0 with a healthy frame is `clear` (empty corridor),
      // points==0 with a near-empty frame is `no_camera_data`. Together with the
      // floor block below, this is what makes the fail-closed anchor auditable
      // from outside the node.
      dj["points"] = depth_points;
      dj["raw_points"] = depth_raw_points;
      payload["depth"] = dj;
      // ★ The floor model itself. Without this the calibration cannot be
      // audited: "did the fit work, and on what plane" would only be answerable
      // from inside. H and pitch are the calibrated form of (a, b) -- they are
      // what the operator compares against the stand-off measurement -- and
      // fit_points is what says whether the frame carried enough geometry to be
      // believed at all.
      {
        nlohmann::json fj;
        fj["ok"] = depth_floor_ok;
        fj["a"] = std::round(depth_floor_a * 10000.0) / 10000.0;
        fj["b"] = std::round(depth_floor_b * 10000.0) / 10000.0;
        const double sect = std::sqrt(1.0 + depth_floor_b * depth_floor_b);
        fj["H_m"] = std::round(depth_floor_a / sect * 10000.0) / 10000.0;
        fj["pitch_deg"] =
          std::round(std::asin(depth_floor_b / sect) * 18000.0 / M_PI) / 100.0;
        fj["fit_points"] = depth_floor_fit_points;
        fj["h_min_m"] =
          std::max(0.01, get_parameter("depth_floor_h_min").as_double());
        fj["truncated"] = depth_truncated;
        payload["floor"] = fj;
      }
    }
    payload["low_obstacle_only"] = low_obstacle;
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
      // ★ `clear` is NOT degraded -- it is a healthy reading of an empty
      // corridor. Listing it here would make the normal case look like a fault
      // and train the reader to ignore the field.
      if (use_depth && depth_state != "ok" && depth_state != "clear") {
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
  int depth_points_{0};
  double depth_stamp_sec_{-1.0};
  bool depth_have_{false};
  uint64_t depth_raw_points_{0};
  bool depth_floor_ok_{false};
  double depth_floor_a_{0.0};
  double depth_floor_b_{0.0};
  long depth_floor_fit_points_{0};
  bool depth_truncated_{false};
  bool safety_ok_{true};
  double prefer_turn_sign_{-1.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr source_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<xw_interfaces::msg::UltrasonicArray>::SharedPtr ultra_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
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
