#pragma once
#include <vector>

#include "navigation2d/geometry/pose_2d.h"
#include "navigation2d/sensor/observations.h"

namespace navigation2d::collision_internal {
// Owns the latest selected laser/point-cloud source in world-frame metres.
// Freshness is acknowledged lazily by an enabled filter, not by sensor stamps.
class RangeObservation {
 public:
  void UpdateLaserScan(const Pose2d&, const LaserScan&);
  void UpdatePointCloud(const Pose2d&, const PointCloud2d&);
  bool FreshAt(double timestamp, double timeout_s);
  const std::vector<Eigen::Vector2d>& points() const& { return points_; }
  const std::vector<Eigen::Vector2d>& blind_zone_points() const& { return blind_zone_points_; }
  const std::vector<Eigen::Vector2d>& points() const&& = delete;
  const std::vector<Eigen::Vector2d>& blind_zone_points() const&& = delete;

 private:
  void UpdatePoints(const Pose2d&, std::vector<Eigen::Vector2d> points);
  std::vector<Eigen::Vector2d> points_, blind_zone_points_;
  bool observation_pending_ = false, has_observation_ = false;
  double last_observation_time_ = 0.;
};
}  // namespace navigation2d::collision_internal
