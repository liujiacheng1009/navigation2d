#include "navigation2d/ros/exploration/explorer_subscriptions.h"

#include <chrono>
#include <stdexcept>
#include <string>
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("explorer subscription contract failed");
  }
}
template <class Predicate>
void SpinUntil(rclcpp::Executor& executor, Predicate ready) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!ready() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds(10));
  }
  Require(ready());
}
void CheckSource(const std::string& source) {
  using namespace navigation2d::exploration_ros;
  auto node = std::make_shared<rclcpp::Node>("explorer_subscriptions_test");
  ExplorerObservations observations;
  int maps = 0, scans = 0;
  auto subscriptions = MakeExplorerSubscriptions(
      *node, source, observations,
      [&](const nav_msgs::msg::OccupancyGrid& value) {
        Require(observations.map() && observations.map()->data == value.data);
        ++maps;
      },
      [&] {
        Require(observations.scan() && observations.scan()->ranges[0] == 2.f);
        ++scans;
      });
  auto map = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/localization2d/map", rclcpp::QoS(1).transient_local().reliable());
  auto scan = node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());
  auto odom = node->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS());
  auto truth =
      node->create_publisher<nav_msgs::msg::Odometry>("/ground_truth", rclcpp::SensorDataQoS());
  auto pose = node->create_publisher<geometry_msgs::msg::PoseStamped>("/localization2d/pose",
                                                                      rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const bool ground_truth = source == "ground_truth";
  SpinUntil(executor, [&] {
    return map->get_subscription_count() == 1 && scan->get_subscription_count() == 1 &&
           odom->get_subscription_count() == 1 &&
           (ground_truth ? truth->get_subscription_count() : pose->get_subscription_count()) == 1;
  });
  Require((ground_truth ? pose->get_subscription_count() : truth->get_subscription_count()) == 0);
  nav_msgs::msg::OccupancyGrid grid;
  grid.data = {-1, 49, 100};
  map->publish(grid);
  sensor_msgs::msg::LaserScan laser;
  laser.ranges = {2.f};
  scan->publish(laser);
  nav_msgs::msg::Odometry measured;
  measured.header.stamp.sec = 10;
  measured.twist.twist.linear.x = .3;
  odom->publish(measured);
  if (ground_truth) {
    measured.pose.pose.position.x = 12.;
    truth->publish(measured);
  } else {
    geometry_msgs::msg::PoseStamped localized;
    localized.pose.position.x = 3.;
    pose->publish(localized);
  }
  SpinUntil(executor, [&] {
    return maps == 1 && scans == 1 && observations.pose() &&
           observations.FreshVelocity(rclcpp::Time(10, 0, RCL_ROS_TIME)).linear == .3;
  });
  Require(observations.pose()->x == (ground_truth ? 0. : 3.));
  subscriptions.reset();
  SpinUntil(executor, [&] {
    return map->get_subscription_count() == 0 && scan->get_subscription_count() == 0 &&
           odom->get_subscription_count() == 0 && pose->get_subscription_count() == 0 &&
           truth->get_subscription_count() == 0;
  });
}
}  // namespace
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  CheckSource("localization");
  CheckSource("ground_truth");
  rclcpp::shutdown();
}
