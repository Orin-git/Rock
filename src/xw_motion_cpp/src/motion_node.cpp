// C++ port of xw_motion/motion_node.py (angle+distance jog via /xw/cmd/motion).
// Logic, topics, node name and publish cadence identical to the Python version.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "xw_interfaces/msg/motion_status.hpp"
#include "xw_interfaces/msg/task_progress.hpp"
#include "xw_interfaces/msg/task_result.hpp"
#include "xw_interfaces/srv/motion_command.hpp"

namespace {

constexpr int IDLE = 0;
constexpr int TURN = 1;
constexpr int DRIVE = 2;
constexpr int DONE = 3;

double ang_diff(double a, double b) {
  return std::remainder(a - b, 2.0 * M_PI);
}

// Python: f"{v:.1f}".rstrip('0').rstrip('.')
std::string fmt_dist(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.1f", v);
  std::string s = buf;
  while (!s.empty() && s.back() == '0') {
    s.pop_back();
  }
  if (!s.empty() && s.back() == '.') {
    s.pop_back();
  }
  return s;
}

}  // namespace

class MotionNode : public rclcpp::Node {
 public:
  MotionNode() : Node("xw_motion") {
    this->declare_parameter<double>("linear_speed", 0.2);
    this->declare_parameter<double>("angular_speed", 0.5);
    this->declare_parameter<double>("angle_tol_deg", 3.0);
    this->declare_parameter<double>("dist_tol_m", 0.03);
    this->declare_parameter<double>("timeout_margin_sec", 5.0);
    this->declare_parameter<bool>("preempt", true);

    state_ = IDLE;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          this->on_odom(*msg);
        });
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/xw/cmd/motion", 10);
    status_pub_ = this->create_publisher<xw_interfaces::msg::MotionStatus>(
        "/xw/motion/status", 10);
    progress_pub_ = this->create_publisher<xw_interfaces::msg::TaskProgress>(
        "/xw/task/progress", 10);
    result_pub_ = this->create_publisher<xw_interfaces::msg::TaskResult>(
        "/xw/task/result", 10);
    srv_ = this->create_service<xw_interfaces::srv::MotionCommand>(
        "/xw/motion/command",
        [this](const std::shared_ptr<xw_interfaces::srv::MotionCommand::Request> req,
               std::shared_ptr<xw_interfaces::srv::MotionCommand::Response> res) {
          return this->on_command(req, res);
        });
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                     [this]() { this->tick(); });
    RCLCPP_INFO(this->get_logger(), "motion node ready");
  }

 private:
  void on_odom(const nav_msgs::msg::Odometry & msg) {
    odom_x_ = msg.pose.pose.position.x;
    odom_y_ = msg.pose.pose.position.y;
    const auto & q = msg.pose.pose.orientation;
    odom_yaw_ = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    have_odom_ = true;
  }

  void on_command(
      const std::shared_ptr<xw_interfaces::srv::MotionCommand::Request> & req,
      std::shared_ptr<xw_interfaces::srv::MotionCommand::Response> res) {
    if (!have_odom_) {
      res->success = false;
      res->message = "no odom yet";
      return;
    }
    const bool preempt = this->get_parameter("preempt").as_bool();
    if (state_ != IDLE && state_ != DONE) {
      if (!preempt) {
        res->success = false;
        res->message = "busy";
        return;
      }
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      finish(1, "preempted");
    }

    cmd_id_ = req->command_id.empty()
                  ? "motion-" + std::to_string(this->now().nanoseconds())
                  : req->command_id;
    start_x_ = odom_x_;
    start_y_ = odom_y_;
    const double angle = req->angle_deg * M_PI / 180.0;
    target_yaw_ = odom_yaw_ + angle;
    const double dist_signed = req->distance_m;
    drive_sign_ = dist_signed >= 0.0 ? 1.0 : -1.0;
    target_dist_ = std::abs(dist_signed);
    const double lin = std::max(0.05, this->get_parameter("linear_speed").as_double());
    const double ang = std::max(0.05, this->get_parameter("angular_speed").as_double());
    const double margin =
        this->get_parameter("timeout_margin_sec").as_double();
    const double turn_t = std::abs(angle) / ang;
    const double drive_t = target_dist_ / lin;
    deadline_ = steady_now() + turn_t + drive_t + margin;

    if (std::abs(req->angle_deg) <= 0.1 && target_dist_ < 1e-3) {
      finish(0, "noop");
      res->success = true;
      res->message = "noop";
      return;
    }
    state_ = std::abs(req->angle_deg) > 0.1 ? TURN : DRIVE;
    res->success = true;
    const std::string direction = drive_sign_ < 0 ? "往后走" : "往前走";
    res->message = direction + " " + fmt_dist(target_dist_) + "米";
    publish_status(state_ == TURN ? TURN : DRIVE, "started");
  }

  double traveled() const {
    return std::hypot(odom_x_ - start_x_, odom_y_ - start_y_);
  }

  void tick() {
    out_ = geometry_msgs::msg::Twist();  // fresh per tick (python Twist() semantics)
    if (state_ == IDLE || state_ == DONE || !have_odom_) {
      if (state_ == IDLE) {
        cmd_pub_->publish(geometry_msgs::msg::Twist());
      }
      return;
    }

    if (deadline_ > 0.0 && steady_now() > deadline_) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      finish(2, "走动超时了");
      return;
    }

    const double ang_sp = this->get_parameter("angular_speed").as_double();
    const double lin_sp = this->get_parameter("linear_speed").as_double();
    const double ang_tol =
        this->get_parameter("angle_tol_deg").as_double() * M_PI / 180.0;
    const double dist_tol = this->get_parameter("dist_tol_m").as_double();

    if (state_ == TURN) {
      const double err = ang_diff(target_yaw_, odom_yaw_);
      if (std::abs(err) < ang_tol) {
        if (target_dist_ > dist_tol) {
          start_x_ = odom_x_;
          start_y_ = odom_y_;
          state_ = DRIVE;
          publish_status(DRIVE, "driving");
        } else {
          finish(0, "done");
        }
        cmd_pub_->publish(geometry_msgs::msg::Twist());
        return;
      }
      out_.angular.z = err > 0 ? ang_sp : -ang_sp;
      cmd_pub_->publish(out_);
      publish_progress("turn", 0.0);
      return;
    }

    if (state_ == DRIVE) {
      const double t = traveled();
      if (t >= target_dist_ - dist_tol) {
        finish(0, "done");
        cmd_pub_->publish(geometry_msgs::msg::Twist());
        return;
      }
      out_.linear.x = lin_sp * drive_sign_;
      cmd_pub_->publish(out_);
      const double pct =
          target_dist_ > 1e-6 ? 100.0 * std::min(1.0, t / target_dist_) : 100.0;
      publish_progress(drive_sign_ < 0 ? "back" : "fwd", pct);
    }
  }

  void publish_progress(const std::string & phase, double percent) {
    auto p = xw_interfaces::msg::TaskProgress();
    p.stamp = this->now();
    p.command_id = cmd_id_;
    p.capability = "motion";
    p.phase = phase;
    p.percent = static_cast<float>(percent);
    p.detail = "";
    progress_pub_->publish(p);
  }

  void publish_status(int status, const std::string & message) {
    auto m = xw_interfaces::msg::MotionStatus();
    m.stamp = this->now();
    m.command_id = cmd_id_;
    m.status = static_cast<uint8_t>(status);
    m.message = message;
    status_pub_->publish(m);
    publish_progress(message, 0.0);
  }

  void finish(int code, const std::string & message) {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    publish_status(code == 0 ? DONE : 5, message);
    auto r = xw_interfaces::msg::TaskResult();
    r.stamp = this->now();
    r.command_id = cmd_id_;
    r.capability = "motion";
    r.code = code;
    r.message = message;
    r.data_json = "";
    result_pub_->publish(r);
    state_ = IDLE;
    deadline_ = 0.0;
  }

  static double steady_now() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  int state_ = IDLE;
  std::string cmd_id_;
  double target_yaw_ = 0.0;
  double target_dist_ = 0.0;
  double start_x_ = 0.0;
  double start_y_ = 0.0;
  double odom_yaw_ = 0.0;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  bool have_odom_ = false;
  double deadline_ = 0.0;
  double drive_sign_ = 1.0;
  geometry_msgs::msg::Twist out_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<xw_interfaces::msg::MotionStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<xw_interfaces::msg::TaskProgress>::SharedPtr progress_pub_;
  rclcpp::Publisher<xw_interfaces::msg::TaskResult>::SharedPtr result_pub_;
  rclcpp::Service<xw_interfaces::srv::MotionCommand>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
