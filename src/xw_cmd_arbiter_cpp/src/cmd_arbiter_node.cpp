// C++ port of xw_cmd_arbiter (Python -> C++).
// Logic, functionality, topics and node names kept identical.
// Original Python: src/xw_cmd_arbiter/xw_cmd_arbiter/cmd_arbiter_node.py (kept as backup).
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace {

// Higher number = higher priority
// follow > motion so loc self-heal spin cannot steal body-follow
const std::map<std::string, int> SOURCE_PRIORITY = {
    {"teleop", 50},
    {"follow", 45},
    {"motion", 40},
    {"nav", 30},
    {"recharge", 10},
};

inline bool isActive(const geometry_msgs::msg::Twist & msg, double eps = 1e-3) {
  return std::abs(msg.linear.x) > eps || std::abs(msg.linear.y) > eps ||
         std::abs(msg.angular.z) > eps;
}

}  // namespace

class CmdArbiterNode : public rclcpp::Node {
 public:
  CmdArbiterNode() : Node("xw_cmd_arbiter"), last_source_("") {
    this->declare_parameter<double>("stale_timeout_sec", 0.4);
    this->declare_parameter<double>("publish_rate_hz", 20.0);

    for (const auto & kv : SOURCE_PRIORITY) {
      const std::string name = kv.first;
      auto sub = this->create_subscription<geometry_msgs::msg::Twist>(
          "/xw/cmd/" + name, rclcpp::QoS(10),
          [this, name](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->on_cmd(name, *msg);
          });
      subs_.push_back(sub);
    }

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/xw/cmd/gated", rclcpp::QoS(10));
    src_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/xw/cmd/active_source", rclcpp::QoS(10));

    const double rate =
        std::max(this->get_parameter("publish_rate_hz").as_double(), 1.0);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        [this]() { this->tick(); });
    RCLCPP_INFO(this->get_logger(), "cmd arbiter ready (active_source)");
  }

 private:
  void on_cmd(const std::string & name, const geometry_msgs::msg::Twist & msg) {
    // Near-zero means this source is idle. If we kept zeros as "fresh nav",
    // a stopped controller would starve other sources and force gated=0.
    if (isActive(msg)) {
      sources_[name] = {msg, this->now().seconds()};
    } else {
      sources_.erase(name);
    }
  }

  std::pair<geometry_msgs::msg::Twist, std::string> select() {
    const double timeout = this->get_parameter("stale_timeout_sec").as_double();
    const double now = this->now().seconds();
    int best_pri = -1;
    geometry_msgs::msg::Twist best_twist;
    std::string best_name;

    for (auto it = sources_.begin(); it != sources_.end();) {
      const double t = it->second.second;
      if (now - t > timeout) {
        it = sources_.erase(it);
        continue;
      }
      if (!isActive(it->second.first)) {
        ++it;
        continue;
      }
      const int pri = SOURCE_PRIORITY.count(it->first)
                          ? SOURCE_PRIORITY.at(it->first)
                          : 0;
      if (pri > best_pri) {
        best_pri = pri;
        best_twist = it->second.first;
        best_name = it->first;
      }
      ++it;
    }
    return {best_twist, best_name};
  }

  void tick() {
    auto [twist, name] = select();
    geometry_msgs::msg::Twist out;
    if (!name.empty()) {
      out = twist;
    }
    pub_->publish(out);
    if (name != last_source_) {
      last_source_ = name;
    }
    std_msgs::msg::String src;
    src.data = name;
    src_pub_->publish(src);
  }

  std::map<std::string, std::pair<geometry_msgs::msg::Twist, double>> sources_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> subs_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr src_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string last_source_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdArbiterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
