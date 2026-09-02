// C++ port of xw_sensors/xw_sensors/pc_nav_filter_node.py.
// Pipeline (same order as Python / PCL stacks):
//   1. Pass-through / ROI crop (optical frame: Z forward, X right, Y down)
//   2. Voxel downsample (first-point-wins)
//   3. Statistical outlier removal (SOR)
//   4. Radius outlier removal
// Node name "xw_pc_nav_filter", topics/parameters identical to the Python node.
// Logic matches the Python reference 1:1 (including per-stream overrides and
// the rate cap), but without numpy/PCL deps.

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"

namespace
{

// Voxel grid key.
struct Key3
{
  int64_t ix, iy, iz;
  bool operator==(const Key3 & o) const
  {
    return ix == o.ix && iy == o.iy && iz == o.iz;
  }
};

struct Key3Hash
{
  size_t operator()(const Key3 & k) const
  {
    uint64_t h = static_cast<uint64_t>(k.ix) * 73856093ull;
    h ^= static_cast<uint64_t>(k.iy) * 19349663ull;
    h ^= static_cast<uint64_t>(k.iz) * 83492791ull;
    h ^= h >> 33;
    h *= 0xff51afd7ed558ccdULL;
    h ^= h >> 33;
    return static_cast<size_t>(h);
  }
};

inline float read_f32(const uint8_t * data, uint32_t off)
{
  float v;
  std::memcpy(&v, data + off, sizeof(float));  // little-endian host (same as Python '<f')
  return v;
}

// Python _find_xyz_offsets: FLOAT32 x/y/z fields -> (x_off, y_off, z_off, point_step).
bool find_xyz_offsets(
  const sensor_msgs::msg::PointCloud2 & msg,
  int & x_off, int & y_off, int & z_off, int & step)
{
  bool fx = false, fy = false, fz = false;
  for (const auto & f : msg.fields) {
    if (f.name == "x" && f.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      x_off = f.offset; fx = true;
    } else if (f.name == "y" && f.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      y_off = f.offset; fy = true;
    } else if (f.name == "z" && f.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      z_off = f.offset; fz = true;
    }
  }
  if (!(fx && fy && fz)) {
    return false;
  }
  step = msg.point_step;
  return true;
}

// Python _pack_xyz_cloud: xyz-only PointCloud2, header preserved as-is.
sensor_msgs::msg::PointCloud2 pack_xyz_cloud(
  const std_msgs::msg::Header & header,
  const std::vector<float> & pts)  // 3*N contiguous
{
  sensor_msgs::msg::PointCloud2 out;
  out.header = header;
  out.height = 1;
  out.width = pts.size() / 3u;
  out.is_bigendian = false;
  out.is_dense = true;
  out.point_step = 12;
  out.row_step = out.point_step * out.width;
  sensor_msgs::msg::PointField f;
  f.datatype = sensor_msgs::msg::PointField::FLOAT32;
  f.count = 1;
  f.name = "x"; f.offset = 0;
  out.fields.push_back(f);
  f.name = "y"; f.offset = 4;
  out.fields.push_back(f);
  f.name = "z"; f.offset = 8;
  out.fields.push_back(f);
  if (!pts.empty()) {
    const auto * begin = reinterpret_cast<const uint8_t *>(pts.data());
    out.data.assign(begin, begin + pts.size() * sizeof(float));
  }
  return out;
}

// Python statistical_outlier_removal: mean of nearest-(k+1)[1..k] distances,
// keep if <= mu + stddev_mul * sigma.
void statistical_outlier_removal(
  std::vector<float> & pts,   // 3*N contiguous xyz
  int mean_k,
  float stddev_mul)
{
  const size_t n = pts.size() / 3u;
  const size_t k = static_cast<size_t>(std::max(mean_k, 1));
  if (n <= k + 1u) {
    return;
  }
  std::vector<float> d2(n);
  std::vector<float> mean_dist(n);
  for (size_t i = 0; i < n; ++i) {
    const float xi = pts[3 * i], yi = pts[3 * i + 1], zi = pts[3 * i + 2];
    for (size_t j = 0; j < n; ++j) {
      const float dx = xi - pts[3 * j], dy = yi - pts[3 * j + 1], dz = zi - pts[3 * j + 2];
      d2[j] = dx * dx + dy * dy + dz * dz;
    }
    // Partial sort to the k-th smallest; positions 1..k are the k smallest
    // non-self neighbors (numpy: partition [:, 1:k+1] -> k smallest after
    // the first; boundary ties are value-identical so order is immaterial).
    std::nth_element(d2.begin(), d2.begin() + static_cast<std::ptrdiff_t>(k), d2.end());
    double acc = 0.0;
    for (size_t j = 1; j <= k; ++j) {
      acc += std::sqrt(d2[j]);
    }
    mean_dist[i] = static_cast<float>(acc / static_cast<double>(k));
  }
  double mu = 0.0, sig = 0.0;
  for (float md : mean_dist) {
    mu += static_cast<double>(md);
  }
  mu /= static_cast<double>(n);
  for (float md : mean_dist) {
    const double d = static_cast<double>(md) - mu;
    sig += d * d;
  }
  sig = std::sqrt(sig / static_cast<double>(n));
  const float thresh = static_cast<float>(mu + static_cast<double>(stddev_mul) * sig);
  std::vector<float> out;
  out.reserve(pts.size());
  for (size_t i = 0; i < n; ++i) {
    if (mean_dist[i] <= thresh) {
      out.push_back(pts[3 * i]);
      out.push_back(pts[3 * i + 1]);
      out.push_back(pts[3 * i + 2]);
    }
  }
  pts.swap(out);
}

// Python radius_outlier_removal: keep points with >= min_neighbors other points
// within radius.
void radius_outlier_removal(
  std::vector<float> & pts,   // 3*N contiguous xyz
  float radius,
  int min_neighbors)
{
  const size_t n = pts.size() / 3u;
  if (n == 0) {
    return;
  }
  const float r2 = radius * radius;
  const int need = std::max(min_neighbors, 1);
  std::vector<float> out;
  out.reserve(pts.size());
  for (size_t i = 0; i < n; ++i) {
    const float xi = pts[3 * i], yi = pts[3 * i + 1], zi = pts[3 * i + 2];
    int cnt = 0;
    for (size_t j = 0; j < n; ++j) {
      if (j == i) {
        continue;
      }
      const float dx = xi - pts[3 * j], dy = yi - pts[3 * j + 1], dz = zi - pts[3 * j + 2];
      if (dx * dx + dy * dy + dz * dz <= r2 && ++cnt >= need) {
        break;
      }
    }
    if (cnt >= need) {
      out.push_back(pts[3 * i]);
      out.push_back(pts[3 * i + 1]);
      out.push_back(pts[3 * i + 2]);
    }
  }
  pts.swap(out);
}

}  // namespace

class PcNavFilterNode : public rclcpp::Node
{
public:
  PcNavFilterNode()
  : Node("xw_pc_nav_filter")
  {
    // ---- parameters (defaults identical to pc_nav_filter_node.py) ----
    declare_parameter<std::vector<std::string>>("input_topics",
      {"/camera/front_up/depth/points", "/camera/front_down/depth/points"});
    declare_parameter<std::vector<std::string>>("output_topics",
      {"/camera/front_up/depth/points_nav", "/camera/front_down/depth/points_nav"});
    // Optical frame ROI (PassThrough / CropBox equivalent).
    declare_parameter<double>("z_min", 0.20);
    declare_parameter<double>("z_max", 2.50);
    declare_parameter<double>("abs_x_max", 1.20);
    declare_parameter<double>("y_min", -0.80);
    declare_parameter<double>("y_max", 0.40);
    declare_parameter<double>("voxel_leaf", 0.06);
    declare_parameter<double>("max_rate_hz", 5.0);
    declare_parameter<int>("max_points_out", 2500);
    declare_parameter<int>("stride", 4);
    // Statistical outlier removal (after voxel).
    declare_parameter<bool>("sor_enable", true);
    declare_parameter<int>("sor_mean_k", 8);
    declare_parameter<double>("sor_stddev_mul", 1.0);
    // Radius outlier removal (after SOR).
    declare_parameter<bool>("radius_enable", true);
    declare_parameter<double>("radius_search", 0.12);
    declare_parameter<int>("radius_min_neighbors", 5);
    // Optional per-stream overrides (same order as input_topics).
    declare_parameter<std::vector<double>>("stream_y_min", {-0.80, -1.20});
    declare_parameter<std::vector<double>>("stream_y_max", {0.40, 1.00});
    declare_parameter<std::vector<int>>("stream_stride", {4, 2});
    declare_parameter<std::vector<bool>>("stream_sor_enable", {true, false});
    declare_parameter<std::vector<bool>>("stream_radius_enable", {true, false});

    const auto inputs = get_parameter("input_topics").as_string_array();
    const auto outputs = get_parameter("output_topics").as_string_array();
    if (inputs.size() != outputs.size()) {
      throw std::runtime_error("input_topics and output_topics length mismatch");
    }

    qos_ = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    for (size_t i = 0; i < inputs.size(); ++i) {
      pubs_.push_back(
        create_publisher<sensor_msgs::msg::PointCloud2>(outputs[i], qos_));
      subs_.push_back(
        create_subscription<sensor_msgs::msg::PointCloud2>(
          inputs[i], qos_,
          [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            on_cloud(i, msg);
          }));
    }
    last_pub_.assign(inputs.size(), -1.0);

    const bool sor_on = get_parameter("sor_enable").as_bool();
    const bool rad_on = get_parameter("radius_enable").as_bool();
    RCLCPP_INFO(
      get_logger(),
      "pc_nav_filter ready: %zu streams -> *_points_nav @%.1f Hz (crop+voxel%s%s)",
      inputs.size(), get_parameter("max_rate_hz").as_double(),
      sor_on ? "+SOR" : "", rad_on ? "+radius" : "");
  }

private:
  // Python _stream_param(name, idx, default): indexed list overwrite or default.
  double stream_double(const std::string & name, size_t idx, double def)
  {
    const auto v = get_parameter(name).as_double_array();
    return idx < v.size() ? v[idx] : def;
  }
  int stream_int(const std::string & name, size_t idx, int def)
  {
    const auto v = get_parameter(name).as_integer_array();
    return idx < v.size() ? static_cast<int>(v[idx]) : def;
  }
  bool stream_bool(const std::string & name, size_t idx, bool def)
  {
    const auto v = get_parameter(name).as_bool_array();
    return idx < v.size() ? v[idx] : def;
  }

  void on_cloud(size_t idx, const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const double rate = get_parameter("max_rate_hz").as_double();
    const double now = get_clock()->now().seconds();
    const double min_dt = 1.0 / std::max(rate, 0.5);
    if (last_pub_[idx] >= 0.0 && now - last_pub_[idx] < min_dt) {
      return;
    }

    int x_off = 0, y_off = 0, z_off = 0, step = 0;
    if (!find_xyz_offsets(*msg, x_off, y_off, z_off, step)) {
      return;
    }
    const size_t n = static_cast<size_t>(msg->width) * msg->height;
    if (n == 0 || step <= 0 || msg->data.size() < static_cast<size_t>(step)) {
      return;
    }

    const double z_min = get_parameter("z_min").as_double();
    const double z_max = get_parameter("z_max").as_double();
    const double abs_x_max = get_parameter("abs_x_max").as_double();
    const double y_min = stream_double("stream_y_min", idx, get_parameter("y_min").as_double());
    const double y_max = stream_double("stream_y_max", idx, get_parameter("y_max").as_double());
    const double leaf = std::max(get_parameter("voxel_leaf").as_double(), 0.02);
    const int stride = std::max(stream_int("stream_stride", idx, get_parameter("stride").as_int()), 1);
    const int max_out = std::max(static_cast<int>(get_parameter("max_points_out").as_int()), 100);
    const bool sor_on = stream_bool("stream_sor_enable", idx, get_parameter("sor_enable").as_bool());
    const bool rad_on = stream_bool("stream_radius_enable", idx, get_parameter("radius_enable").as_bool());

    const uint8_t * data = msg->data.data();
    const uint32_t max_off = static_cast<uint32_t>(
      std::max({x_off, y_off, z_off}));

    // Voxel downsample: first point per voxel wins, insertion order kept.
    std::unordered_map<Key3, size_t, Key3Hash> voxel_index;
    std::vector<float> vox;  // 3*M
    vox.reserve(3 * static_cast<size_t>(max_out));
    for (size_t i = 0; i < n; i += static_cast<size_t>(stride)) {
      const size_t base = i * static_cast<size_t>(step);
      if (base + max_off + 4u > msg->data.size()) {
        break;
      }
      const float x = read_f32(data, base + x_off);
      const float y = read_f32(data, base + y_off);
      const float z = read_f32(data, base + z_off);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (z < z_min || z > z_max) {
        continue;
      }
      if (std::abs(x) > abs_x_max) {
        continue;
      }
      if (y < y_min || y > y_max) {
        continue;
      }
      Key3 key{
        static_cast<int64_t>(std::floor(x / leaf)),
        static_cast<int64_t>(std::floor(y / leaf)),
        static_cast<int64_t>(std::floor(z / leaf))};
      auto it = voxel_index.find(key);
      if (it == voxel_index.end()) {
        const size_t m = vox.size() / 3u;
        voxel_index.emplace(key, m);
        vox.push_back(x);
        vox.push_back(y);
        vox.push_back(z);
        if (vox.size() / 3u >= static_cast<size_t>(max_out)) {
          break;
        }
      }
    }

    if (vox.empty()) {
      auto out = pack_xyz_cloud(msg->header, {});
      pubs_[idx]->publish(out);
      last_pub_[idx] = now;
      return;
    }

    if (sor_on) {
      statistical_outlier_removal(vox,
        get_parameter("sor_mean_k").as_int(),
        static_cast<float>(get_parameter("sor_stddev_mul").as_double()));
    }

    if (rad_on && !vox.empty()) {
      radius_outlier_removal(vox,
        static_cast<float>(get_parameter("radius_search").as_double()),
        get_parameter("radius_min_neighbors").as_int());
    }

    // Preserve acquisition stamp (do not stamp with now()).
    auto out = pack_xyz_cloud(msg->header, vox);
    pubs_[idx]->publish(out);
    last_pub_[idx] = now;
  }

  rclcpp::QoS qos_{rclcpp::KeepLast(1)};
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pubs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_;
  std::vector<double> last_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PcNavFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
