#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);
    trajectory_timeout_ = declare_parameter<double>("trajectory_timeout", 2.0);
    cloud_timeout_ = declare_parameter<double>("cloud_timeout", 1.0);
    require_cloud_ = declare_parameter<bool>("require_cloud", false);
    if (odom_timeout_ <= 0.0 || trajectory_timeout_ <= 0.0 || cloud_timeout_ <= 0.0) {
      throw std::invalid_argument("timeouts must be positive");
    }

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    if (require_cloud_) {
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          "cloud", rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            if (msg->width != 0 && msg->height != 0 && !msg->data.empty()) {
              last_cloud_received_ = std::chrono::steady_clock::now();
              have_cloud_ = true;
            }
          });
    }
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    cmd_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0, bool valid = false)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    publishCommand(cmd, valid);
  }

  void publishCommand(const geometry_msgs::msg::Twist & cmd, bool valid = true)
  {
    cmd_vel_pub_->publish(cmd);
    geometry_msgs::msg::TwistStamped tagged;
    tagged.header.stamp = reference_path_stamp_;
    tagged.header.frame_id = valid ? "base_footprint" : "";
    tagged.twist = cmd;
    cmd_stamped_pub_->publish(tagged);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (!have_odom_ ||
        std::chrono::duration<double>(steady_now - last_odom_received_).count() > odom_timeout_ ||
        (require_cloud_ && (!have_cloud_ ||
          std::chrono::duration<double>(steady_now - last_cloud_received_).count() > cloud_timeout_)))
    {
      return;
    }
    if (msg->order <= 0 || msg->pos_pts.size() <= static_cast<size_t>(msg->order) ||
        msg->knots.size() != msg->pos_pts.size() + static_cast<size_t>(msg->order) + 1 ||
        !std::all_of(msg->pos_pts.begin(), msg->pos_pts.end(), [](const auto & p) {
          return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
        }) || !std::all_of(msg->knots.begin(), msg->knots.end(), [](double v) { return std::isfinite(v); }) ||
        std::adjacent_find(msg->knots.begin(), msg->knots.end(), std::greater_equal<double>()) != msg->knots.end())
    {
      receive_traj_ = false;
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    reference_path_stamp_ = msg->reference_path_stamp;
    // Planning and transport already consumed part of this trajectory. Starting
    // it again at zero causes repeated braking whenever a replan arrives.
    exec_time_ = 0.0;
    if (msg->start_time.sec != 0) {
      const double age = (now() - rclcpp::Time(msg->start_time, get_clock()->get_clock_type())).seconds();
      if (!std::isfinite(age) || age > trajectory_timeout_ || age < -0.2) {
        receive_traj_ = false;
        return;
      }
      exec_time_ = std::clamp(age, 0.0, traj_duration_);
    }
    last_update_time_ = now();
    receive_traj_ = true;
    last_trajectory_received_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
    last_odom_received_ = std::chrono::steady_clock::now();
  }

  void cmdCallback()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const bool odom_stale = !have_odom_ ||
      std::chrono::duration<double>(steady_now - last_odom_received_).count() > odom_timeout_;
    const bool trajectory_stale = !receive_traj_ ||
      std::chrono::duration<double>(steady_now - last_trajectory_received_).count() > trajectory_timeout_;
    const bool cloud_stale = require_cloud_ && (!have_cloud_ ||
      std::chrono::duration<double>(steady_now - last_cloud_received_).count() > cloud_timeout_);
    if (odom_stale || trajectory_stale || cloud_stale)
    {
      // A recovered odometry stream must not restart an old trajectory.
      receive_traj_ = false;
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command, true);
      last_update_time_ = current_time;
      return;
    }

    publishExecutionFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();
    publishCommand(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  builtin_interfaces::msg::Time reference_path_stamp_;
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  double odom_timeout_, trajectory_timeout_;
  double cloud_timeout_;
  bool require_cloud_{false}, have_cloud_{false};
  std::chrono::steady_clock::time_point last_odom_received_{};
  std::chrono::steady_clock::time_point last_trajectory_received_{};
  std::chrono::steady_clock::time_point last_cloud_received_{};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
