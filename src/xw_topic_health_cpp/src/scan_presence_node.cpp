#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class ScanPresenceNode : public rclcpp::Node
{
public:
  ScanPresenceNode()
  : Node("xw_scan_presence")
  {
    declare_parameter<std::string>("scan_topic", "scan");
    declare_parameter<std::string>("out_topic", "/xw/health/scan_alive");
    declare_parameter<double>("publish_hz", 2.0);
    declare_parameter<double>("stale_sec", 1.5);

    const auto scan_topic = get_parameter("scan_topic").as_string();
    const auto out_topic = get_parameter("out_topic").as_string();
    const double hz = std::max(0.5, get_parameter("publish_hz").as_double());
    stale_sec_ = get_parameter("stale_sec").as_double();

    // SensorData QoS; C++ deserialize is cheap vs Python for 10Hz scans.
    auto qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, qos,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr) {
        last_scan_ = now();
      });
    pub_ = create_publisher<std_msgs::msg::Bool>(out_topic, rclcpp::QoS(1).transient_local());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / hz),
      [this]() {
        std_msgs::msg::Bool msg;
        msg.data = (last_scan_.nanoseconds() > 0) &&
          ((now() - last_scan_).seconds() <= stale_sec_);
        pub_->publish(msg);
      });
    RCLCPP_INFO(get_logger(), "scan_presence %s -> %s @ %.1f Hz",
      scan_topic.c_str(), out_topic.c_str(), hz);
  }

private:
  double stale_sec_{1.5};
  rclcpp::Time last_scan_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanPresenceNode>());
  rclcpp::shutdown();
  return 0;
}
