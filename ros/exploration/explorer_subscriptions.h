#pragma once
#include <functional>
#include <memory>
#include <string>

#include "navigation2d/ros/exploration/explorer_observations.h"
#include "rclcpp/rclcpp.hpp"
namespace navigation2d::exploration_ros {
// ROS wiring only. Observations/node and callback captures outlive this owner.
// A single executor serializes cache updates before their application callbacks.
class ExplorerSubscriptions {
 public:
  using MapApplied = std::function<void(const nav_msgs::msg::OccupancyGrid&)>;
  using ScanApplied = std::function<void()>;
  ExplorerSubscriptions(rclcpp::Node&, const std::string& pose_source, ExplorerObservations&,
                        MapApplied, ScanApplied);
  ExplorerSubscriptions(const ExplorerSubscriptions&) = delete;
  ExplorerSubscriptions& operator=(const ExplorerSubscriptions&) = delete;

 private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr localization_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
};
std::unique_ptr<ExplorerSubscriptions> MakeExplorerSubscriptions(
    rclcpp::Node&, const std::string& pose_source, ExplorerObservations&,
    ExplorerSubscriptions::MapApplied, ExplorerSubscriptions::ScanApplied);
}  // namespace navigation2d::exploration_ros
