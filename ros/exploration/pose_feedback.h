#pragma once
#include <optional>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navigation2d/application/exploration/pose_observation.h"

namespace navigation2d::exploration_ros {
// Owns the latest map-frame pose and accumulated driven trail. Inputs are
// consumed synchronously by the single ROS executor. Views are read-only and
// remain valid until the next observation; no caller can append to the trail.
class PoseFeedback {
 public:
  void ObserveLocalization(const geometry_msgs::msg::PoseStamped& pose);
  void ObserveGroundTruth(const nav_msgs::msg::Odometry& odometry);
  const std::optional<geometry_msgs::msg::PoseStamped>& pose() const { return pose_; }
  const std::vector<std::pair<double, double>>& trail() const { return trail_; }
  double distance_m() const { return distance_m_; }
  const std::optional<exploration::PoseObservation>& observation() const { return observation_; }

 private:
  std::optional<geometry_msgs::msg::PoseStamped> pose_;
  std::vector<std::pair<double, double>> trail_;
  double distance_m_ = 0.;
  bool truth_origin_ = false;
  double truth_x_ = 0., truth_y_ = 0., truth_yaw_ = 0.;
  std::optional<exploration::PoseObservation> observation_;
};
}  // namespace navigation2d::exploration_ros
