#include <chrono>
#include <thread>
#include <gtest/gtest.h>
#include "../src/scan_controller.cpp"

using namespace std::chrono_literals;

TEST(ScanController, AcceptsOnlyCurrentFreshPathAndHonorsSpeedLimit)
{
  rclcpp::init(0, nullptr);
  auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("scan_test_controller");
  auto client = std::make_shared<rclcpp::Node>("scan_test_client");
  scan_nav2_controller::ScanController plugin;
  plugin.configure(parent, "FollowPath", nullptr, nullptr);
  plugin.activate();
  nav_msgs::msg::Path received;
  auto sub = client->create_subscription<nav_msgs::msg::Path>(
    "/scan_control/plan", 10, [&](nav_msgs::msg::Path::SharedPtr msg) {received = *msg;});
  auto pub = client->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/scan_compare/cmd_vel_stamped", 10);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(parent->get_node_base_interface());
  exec.add_node(client);
  auto spin = [&]() {for (int i = 0; i < 30; ++i) {exec.spin_some(); std::this_thread::sleep_for(10ms);}};
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist speed;
  pose.header.frame_id = "odom"; pose.pose.orientation.w = 1.0;
  nav_msgs::msg::Path plan; plan.header.frame_id = "odom";
  plan.poses.emplace_back();
  plan.poses.back().pose.position.x = 2.0;
  plan.poses.back().pose.orientation.w = 1.0;
  spin(); plugin.setPlan(plan); spin();
  ASSERT_NE(received.header.stamp.sec, 0);
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = "base_footprint";
  cmd.header.stamp = received.header.stamp;
  cmd.header.stamp.nanosec += 1;
  cmd.twist.linear.x = 0.2;
  pub->publish(cmd); spin();
  EXPECT_EQ(plugin.computeVelocityCommands(pose, speed, nullptr).twist.linear.x, 0.0);
  cmd.header.stamp = received.header.stamp;
  pub->publish(cmd);
  for (int i = 0; i < 10; ++i) {exec.spin_some(); std::this_thread::sleep_for(5ms);}
  EXPECT_NEAR(plugin.computeVelocityCommands(pose, speed, nullptr).twist.linear.x, 0.1, 1e-6);
  plugin.setSpeedLimit(50.0, true);
  EXPECT_NEAR(plugin.computeVelocityCommands(pose, speed, nullptr).twist.linear.x, 0.05, 1e-6);
  std::this_thread::sleep_for(400ms);
  EXPECT_EQ(plugin.computeVelocityCommands(pose, speed, nullptr).twist.linear.x, 0.0);
  plugin.setPlan(plan); spin();
  pub->publish(cmd);
  for (int i = 0; i < 10; ++i) {exec.spin_some(); std::this_thread::sleep_for(5ms);}
  EXPECT_EQ(plugin.computeVelocityCommands(pose, speed, nullptr).twist.linear.x, 0.0);
  plan.poses.back().pose.orientation.z = std::sin(0.5);
  plan.poses.back().pose.orientation.w = std::cos(0.5);
  plugin.setPlan(plan); spin();
  cmd.header.stamp = received.header.stamp;
  pub->publish(cmd);
  for (int i = 0; i < 10; ++i) {exec.spin_some(); std::this_thread::sleep_for(5ms);}
  pose.pose.position.x = 1.85;
  auto aligned = plugin.computeVelocityCommands(pose, speed, nullptr);
  EXPECT_EQ(aligned.twist.linear.x, 0.0);
  EXPECT_NEAR(aligned.twist.angular.z, 0.25, 1e-6);
  pose.pose.orientation = plan.poses.back().pose.orientation;
  EXPECT_NEAR(plugin.computeVelocityCommands(pose, speed, nullptr).twist.angular.z, 0.0, 1e-6);
  cmd.header.frame_id = "";
  pub->publish(cmd);
  for (int i = 0; i < 10; ++i) {exec.spin_some(); std::this_thread::sleep_for(5ms);}
  pose.pose.orientation.z = 0.0; pose.pose.orientation.w = 1.0;
  EXPECT_EQ(plugin.computeVelocityCommands(pose, speed, nullptr).twist.angular.z, 0.0);
  plugin.deactivate(); plugin.cleanup();
  exec.remove_node(client); exec.remove_node(parent->get_node_base_interface());
  rclcpp::shutdown();
}
