#pragma once
#include <vector>

#include "navigation2d/configuration/navigation_config.h"
#include "navigation2d/geometry/pose_2d.h"

namespace navigation2d::collision_internal {
// Owns depth tracks and briefly held returns in the world frame. Single writer.
// Update uses base-frame metres; timestamp is the caller's sensor time domain.
// Config is borrowed for each operation and is never retained.
class DepthObservation {
 public:
  void Update(const Pose2d& pose, const std::vector<Eigen::Vector2d>& base_points, double timestamp,
              const NavigationConfig& config);
  void Refresh(const Pose2d& pose, double timestamp, const NavigationConfig& config);
  const std::vector<Eigen::Vector2d>& points() const { return depth_points_; }
  const std::vector<Eigen::Vector2d>& held() const { return held_depth_points_; }

 private:
  struct Track {
    Eigen::Vector2d point = Eigen::Vector2d::Zero();
    int hits = 0;
  };
  void CullPassed(const Pose2d& pose, const NavigationConfig& config);
  std::vector<Eigen::Vector2d> depth_points_, held_depth_points_;
  std::vector<Track> depth_tracks_;
  bool has_depth_stamp_ = false;
  double depth_stamp_ = 0.;
};
}  // namespace navigation2d::collision_internal
