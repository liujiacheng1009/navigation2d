#include "navigation2d/ros/exploration/pose_feedback.h"

#include <cmath>
namespace navigation2d::exploration_ros {
namespace {
double Yaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y * q.y + q.z * q.z));
}
}  // namespace
void PoseFeedback::ObserveLocalization(const geometry_msgs::msg::PoseStamped& pose) {
  pose_ = pose;
  const auto point = std::pair<double, double>{pose.pose.position.x, pose.pose.position.y};
  if (trail_.empty() ||
      std::hypot(point.first - trail_.back().first, point.second - trail_.back().second) > .05) {
    if (!trail_.empty()) {
      distance_m_ +=
          std::hypot(point.first - trail_.back().first, point.second - trail_.back().second);
    }
    trail_.push_back(point);
  }
}
void PoseFeedback::ObserveGroundTruth(const nav_msgs::msg::Odometry& odometry) {
  const auto& source = odometry.pose.pose;
  const double source_yaw = Yaw(source.orientation);
  if (!truth_origin_) {
    truth_x_ = source.position.x;
    truth_y_ = source.position.y;
    truth_yaw_ = source_yaw;
    truth_origin_ = true;
  }
  const double dx = source.position.x - truth_x_;
  const double dy = source.position.y - truth_y_;
  geometry_msgs::msg::PoseStamped value_in_map;
  value_in_map.header = odometry.header;
  value_in_map.header.frame_id = "map";
  value_in_map.pose.position.x = std::cos(truth_yaw_) * dx + std::sin(truth_yaw_) * dy;
  value_in_map.pose.position.y = -std::sin(truth_yaw_) * dx + std::cos(truth_yaw_) * dy;
  const double yaw = source_yaw - truth_yaw_;
  value_in_map.pose.orientation.z = std::sin(yaw / 2.);
  value_in_map.pose.orientation.w = std::cos(yaw / 2.);
  ObserveLocalization(value_in_map);
}
}  // namespace navigation2d::exploration_ros
