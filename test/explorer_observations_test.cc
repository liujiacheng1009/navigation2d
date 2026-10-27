#include "navigation2d/ros/exploration/explorer_observations.h"

#include <cmath>
#include <limits>
#include <stdexcept>
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("observation cache contract failed");
  }
}
}  // namespace
int main() {
  using navigation2d::exploration_ros::ExplorerObservations;
  ExplorerObservations cache;
  const rclcpp::Time reference(1000000000LL, RCL_ROS_TIME);
  Require(!cache.map() && !cache.scan() && !cache.pose() && cache.trail().empty());
  Require(cache.FreshVelocity(reference).linear == 0.);
  nav_msgs::msg::Odometry velocity;
  velocity.header.stamp.sec = 1;
  velocity.twist.twist.linear.x = .3;
  velocity.twist.twist.angular.z = -.4;
  cache.ObserveVelocity(velocity);
  for (auto offset : {-200000000LL, 0LL, 200000000LL}) {
    const auto fresh = cache.FreshVelocity(rclcpp::Time(1000000000LL + offset, RCL_ROS_TIME));
    Require(fresh.linear == .3 && fresh.angular == -.4);
  }
  for (auto offset : {-200000001LL, 200000001LL}) {
    const auto stale = cache.FreshVelocity(rclcpp::Time(1000000000LL + offset, RCL_ROS_TIME));
    Require(stale.linear == 0. && stale.angular == 0.);
  }
  bool incompatible_clock = false;
  try {
    static_cast<void>(cache.FreshVelocity(rclcpp::Time(1000000000LL, RCL_SYSTEM_TIME)));
  } catch (const std::runtime_error&) {
    incompatible_clock = true;
  }
  Require(incompatible_clock);
  velocity.header.stamp.sec = -1;
  velocity.twist.twist.linear.x = .7;
  bool invalid_stamp = false;
  try {
    cache.ObserveVelocity(velocity);
  } catch (const std::runtime_error&) {
    invalid_stamp = true;
  }
  // Preserve the old callback's assignment order when timestamp construction fails.
  Require(invalid_stamp && cache.FreshVelocity(reference).linear == .7);
  cache.ObserveVelocity({std::numeric_limits<double>::quiet_NaN(), .1}, reference);
  Require(std::isnan(cache.FreshVelocity(reference).linear));
  cache.ObserveVelocity({.4, .2}, rclcpp::Time(0, 0, RCL_ROS_TIME));
  Require(cache.FreshVelocity(reference).linear == 0.);
  nav_msgs::msg::OccupancyGrid map;
  map.data = {-1, 49, 50, 100};
  cache.ObserveMap(map);
  map.data[0] = 100;
  Require(cache.map()->data[0] == -1);
  sensor_msgs::msg::LaserScan scan;
  scan.ranges = {1.f, 2.f};
  cache.ObserveScan(scan);
  scan.ranges[0] = 8.f;
  Require(cache.scan()->ranges[0] == 1.f);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp.sec = 3;
  cache.ObserveLocalization(pose);
  auto independent = cache;
  pose.pose.position.x = 1.;
  cache.ObserveLocalization(pose);
  Require(cache.pose()->x == 1. && cache.distance_m() == 1. && cache.trail().size() == 2);
  Require(independent.pose()->x == 0. && independent.distance_m() == 0.);
  Require(independent.map()->data[0] == -1 && independent.scan()->ranges[0] == 1.f);
}
