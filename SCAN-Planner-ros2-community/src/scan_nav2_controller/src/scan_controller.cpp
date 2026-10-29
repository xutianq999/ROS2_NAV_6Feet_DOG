#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
#include <memory>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_core/controller.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace scan_nav2_controller
{
class ScanController : public nav2_core::Controller
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
    node_ = parent.lock();
    if (!node_) throw std::runtime_error("SCAN controller parent expired");
    name_ = name;
    tf_ = tf;
    node_->declare_parameter(name_ + ".max_linear", 0.10);
    node_->declare_parameter(name_ + ".max_angular", 0.25);
    node_->declare_parameter(name_ + ".command_timeout", 0.3);
    max_linear_ = node_->get_parameter(name_ + ".max_linear").as_double();
    max_angular_ = node_->get_parameter(name_ + ".max_angular").as_double();
    timeout_ = node_->get_parameter(name_ + ".command_timeout").as_double();
    if (max_linear_ <= 0 || max_angular_ <= 0 || timeout_ <= 0) {
      throw std::invalid_argument("SCAN controller limits and timeout must be positive");
    }
    speed_limit_ = max_linear_;
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      "/scan_control/plan", rclcpp::QoS(1).transient_local());
    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/scan_compare/cmd_vel_stamped", 10,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (msg->header.stamp.sec != path_stamp_.sec ||
            msg->header.stamp.nanosec != path_stamp_.nanosec || path_stamp_.sec == 0) return;
        if (msg->header.frame_id != "base_footprint") {
          have_command_ = false;
          return;
        }
        command_ = msg->twist;
        received_ = std::chrono::steady_clock::now();
        have_command_ = true;
      });
  }
  void cleanup() override { cmd_sub_.reset(); path_pub_.reset(); node_.reset(); }
  void activate() override { path_pub_->on_activate(); }
  void deactivate() override {
    std::lock_guard<std::mutex> lock(mutex_);
    have_command_ = false;
    path_stamp_ = builtin_interfaces::msg::Time();
    path_pub_->on_deactivate();
  }
  void setPlan(const nav_msgs::msg::Path & path) override {
    auto output = path;
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = node_->now().nanoseconds();
    if (now <= last_stamp_) now = last_stamp_ + 1;
    last_stamp_ = now;
    output.header.stamp = rclcpp::Time(now, node_->get_clock()->get_clock_type());
    if (!path.poses.empty()) {
      goal_ = path.poses.back();
      if (goal_.header.frame_id.empty()) goal_.header.frame_id = path.header.frame_id;
      has_goal_ = true;
    } else {
      has_goal_ = false;
    }
    path_stamp_ = output.header.stamp;
    have_command_ = false;
    path_pub_->publish(output);
    RCLCPP_INFO(node_->get_logger(), "SCAN path sent: stamp=%d.%09u points=%zu",
      path_stamp_.sec, path_stamp_.nanosec, output.poses.size());
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override {
    geometry_msgs::msg::TwistStamped result;
    result.header = pose.header;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_command_ || std::chrono::duration<double>(
        std::chrono::steady_clock::now() - received_).count() > timeout_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
        "SCAN command unavailable or stale for current path; stopping");
      return result;
    }
    if (!std::isfinite(command_.linear.x) || !std::isfinite(command_.angular.z)) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
        "SCAN command contains non-finite values; stopping");
      return result;
    }
    if (has_goal_) {
      auto goal = goal_;
      try {
        if (goal.header.frame_id != pose.header.frame_id) {
          if (!tf_ || goal.header.frame_id.empty() || pose.header.frame_id.empty()) return result;
          goal.header.stamp = rclcpp::Time(0);
          goal = tf_->transform(goal, pose.header.frame_id, tf2::durationFromSec(0.05));
        }
        const auto & q = goal.pose.orientation;
        const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (std::isfinite(norm) && norm > 0.5 && norm < 1.5 &&
            std::hypot(goal.pose.position.x - pose.pose.position.x,
                       goal.pose.position.y - pose.pose.position.y) <= 0.18) {
          const double error = std::atan2(
            std::sin(tf2::getYaw(q) - tf2::getYaw(pose.pose.orientation)),
            std::cos(tf2::getYaw(q) - tf2::getYaw(pose.pose.orientation)));
          if (std::isfinite(error)) result.twist.angular.z = std::clamp(error, -max_angular_, max_angular_);
          return result;
        }
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
          "Cannot transform SCAN goal: %s", ex.what());
        return result;
      }
    }
    result.twist.linear.x = std::clamp(command_.linear.x, -speed_limit_, speed_limit_);
    result.twist.angular.z = std::clamp(command_.angular.z, -max_angular_, max_angular_);
    return result;
  }
  void setSpeedLimit(const double & speed, const bool & percentage) override {
    std::lock_guard<std::mutex> lock(mutex_);
    speed_limit_ = speed <= 0 ? max_linear_ :
      std::clamp(percentage ? max_linear_ * speed / 100.0 : speed, 0.0, max_linear_);
  }
private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  std::mutex mutex_;
  builtin_interfaces::msg::Time path_stamp_;
  int64_t last_stamp_{0};
  bool have_command_{false};
  bool has_goal_{false};
  geometry_msgs::msg::PoseStamped goal_;
  geometry_msgs::msg::Twist command_;
  std::chrono::steady_clock::time_point received_{};
  double max_linear_{0.1}, max_angular_{0.25}, timeout_{0.3}, speed_limit_{0.1};
};
}
PLUGINLIB_EXPORT_CLASS(scan_nav2_controller::ScanController, nav2_core::Controller)
