#pragma once
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navigation2d/geometry/twist_2d.h"
#include "navigation2d/ros/exploration/pose_feedback.h"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
namespace navigation2d::exploration_ros {
// Single-executor latest observations. Views expire at the next observation;
// no subscriber, navigation, map integration or publisher is owned here.
class ExplorerObservations {
 public:
  void ObserveMap(const nav_msgs::msg::OccupancyGrid& value) { map_ = value; }
  void ObserveScan(const sensor_msgs::msg::LaserScan& value) { scan_ = value; }
  void ObserveLocalization(const geometry_msgs::msg::PoseStamped& value) {
    pose_.ObserveLocalization(value);
  }
  void ObserveGroundTruth(const nav_msgs::msg::Odometry& value) { pose_.ObserveGroundTruth(value); }
  void ObserveVelocity(const nav_msgs::msg::Odometry& value) {
    velocity_ = {value.twist.twist.linear.x, value.twist.twist.angular.z};
    velocity_stamp_ = rclcpp::Time(value.header.stamp);
  }
  // Explicit measured input in one ROS clock domain; also used by replay.
  void ObserveVelocity(Twist2d value, const rclcpp::Time& stamp) {
    velocity_ = value;
    velocity_stamp_ = stamp;
  }
  // Preserve the encoder freshness window, including both 200 ms endpoints.
  // Pose feedback remains authoritative for accumulated motion during delays.
  Twist2d FreshVelocity(const rclcpp::Time& reference) const {
    if (velocity_stamp_.nanoseconds() == 0) {
      return {};
    }
    const double age = (reference - velocity_stamp_).seconds();
    if (!std::isfinite(age) || age < -.20 || age > .20) {
      return {};
    }
    return velocity_;
  }
  const std::optional<nav_msgs::msg::OccupancyGrid>& map() const { return map_; }
  const std::optional<sensor_msgs::msg::LaserScan>& scan() const { return scan_; }
  const std::optional<exploration::PoseObservation>& pose() const { return pose_.observation(); }
  const std::vector<std::pair<double, double>>& trail() const { return pose_.trail(); }
  double distance_m() const { return pose_.distance_m(); }

 private:
  rclcpp::Time velocity_stamp_{0, 0, RCL_ROS_TIME};
  std::optional<nav_msgs::msg::OccupancyGrid> map_;
  std::optional<sensor_msgs::msg::LaserScan> scan_;
  PoseFeedback pose_;
  Twist2d velocity_;
};
}  // namespace navigation2d::exploration_ros
