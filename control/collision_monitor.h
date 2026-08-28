#pragma once

#include <vector>

#include <Eigen/Core>

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/control/local_controller.h"
#include "navigation2d/sensor/observations.h"

namespace navigation2d {

enum class CollisionMonitorAction { kNone, kSlowdown, kStop, kSourceTimeout };

struct CollisionMonitorResult {
  Twist2d command;
  CollisionMonitorAction action = CollisionMonitorAction::kNone;
  double min_distance_m = 0.;
  double time_to_collision_s = 0.;
};

// Raw-sensor safety layer adapted from Nav2 Collision Monitor's independent
// stop/slowdown/approach architecture. It intentionally never reads a costmap.
class CollisionMonitor {
 public:
  explicit CollisionMonitor(NavigationConfig config) : config_(std::move(config)) {}
  void UpdateLaserScan(const Pose2d& sensor_pose, const LaserScan& scan);
  void UpdatePointCloud(const Pose2d& sensor_pose, const PointCloud2d& cloud);
  CollisionMonitorResult Filter(const Pose2d& robot_pose, Twist2d command,
                                double timestamp);

 private:
  void UpdatePoints(const Pose2d& sensor_pose, std::vector<Eigen::Vector2d> points);
  NavigationConfig config_;
  std::vector<Eigen::Vector2d> points_;
  bool observation_pending_ = false;
  bool has_observation_ = false;
  double last_observation_time_ = 0.;
  int trigger_count_ = 0;
  int release_count_ = 0;
  CollisionMonitorAction latched_action_ = CollisionMonitorAction::kNone;
};

}  // namespace navigation2d
