// UltrasonicProximity BT condition (Gen2): true when the nearest object seen
// by the front ultrasonic sector is closer than a threshold.
//
// Generic (no environment assumptions): reads /ultrasonic_scan, takes the
// minimum range inside +/- sector/2 of the robot forward and compares with a
// distance threshold. Environment-agnostic: works for doors, handles or any
// structure the ultrasound can see.
//
// Ports (set in XML):
//   threshold: double (m)
//   sector_deg: double (deg)
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace xw_bt_plugins
{

class UltrasonicProximity : public BT::ConditionNode
{
public:
  UltrasonicProximity(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/ultrasonic_scan",
      rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        scan_ = msg;
      });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("threshold"),
      BT::InputPort<double>("sector_deg"),
    };
  }

  BT::NodeStatus tick() override
  {
    double threshold = 0.30;
    getInput("threshold", threshold);
    double sector = 60.0;
    getInput("sector_deg", sector);
    const double half = (sector / 2.0) * M_PI / 180.0;

    sensor_msgs::msg::LaserScan::SharedPtr scan;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      scan = scan_;
    }
    if (!scan) {
      return BT::NodeStatus::FAILURE;
    }

    double min_r = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      const double a = std::abs(
        scan->angle_min + scan->angle_increment * static_cast<double>(i));
      if (a > half) {
        continue;
      }
      const double r = scan->ranges[i];
      if (r > 0 && r < min_r) {
        min_r = r;
      }
    }

    const bool blocked = !std::isinf(min_r) && min_r < threshold;
    static rclcpp::Time last_log_{0, 0, RCL_ROS_TIME};
    const rclcpp::Time now = node_->now();
    if ((now - last_log_).seconds() > 5.0) {
      RCLCPP_INFO(
        node_->get_logger(),
        "ultrasonic_proximity min=%.2f threshold=%.2f -> %s",
        min_r, threshold, blocked ? "BLOCKED" : "CLEAR");
      last_log_ = now;
    }
    return blocked ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  std::mutex mtx_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
};

}  // namespace xw_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<xw_bt_plugins::UltrasonicProximity>("UltrasonicProximity");
}
