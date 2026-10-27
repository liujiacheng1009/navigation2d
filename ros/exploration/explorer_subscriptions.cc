#include "navigation2d/ros/exploration/explorer_subscriptions.h"

#include <utility>
namespace navigation2d::exploration_ros {
ExplorerSubscriptions::ExplorerSubscriptions(rclcpp::Node& node, const std::string& pose_source,
                                             ExplorerObservations& observations, MapApplied on_map,
                                             ScanApplied on_scan) {
  map_ = node.create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/localization2d/map", rclcpp::QoS(1).transient_local().reliable(),
      [&observations,
       on_map = std::move(on_map)](nav_msgs::msg::OccupancyGrid::ConstSharedPtr value) {
        observations.ObserveMap(*value);
        on_map(*value);
      });
  if (pose_source == "ground_truth") {
    truth_ = node.create_subscription<nav_msgs::msg::Odometry>(
        "/ground_truth", rclcpp::SensorDataQoS(),
        [&observations](nav_msgs::msg::Odometry::ConstSharedPtr value) {
          observations.ObserveGroundTruth(*value);
        });
  } else {
    localization_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
        "/localization2d/pose", rclcpp::SensorDataQoS(),
        [&observations](geometry_msgs::msg::PoseStamped::ConstSharedPtr value) {
          observations.ObserveLocalization(*value);
        });
  }
  odometry_ = node.create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      [&observations](nav_msgs::msg::Odometry::ConstSharedPtr value) {
        observations.ObserveVelocity(*value);
      });
  scan_ = node.create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [&observations,
       on_scan = std::move(on_scan)](sensor_msgs::msg::LaserScan::ConstSharedPtr value) {
        observations.ObserveScan(*value);
        on_scan();
      });
}
std::unique_ptr<ExplorerSubscriptions> MakeExplorerSubscriptions(
    rclcpp::Node& node, const std::string& pose_source, ExplorerObservations& observations,
    ExplorerSubscriptions::MapApplied on_map, ExplorerSubscriptions::ScanApplied on_scan) {
  return std::make_unique<ExplorerSubscriptions>(node, pose_source, observations, std::move(on_map),
                                                 std::move(on_scan));
}
}  // namespace navigation2d::exploration_ros
