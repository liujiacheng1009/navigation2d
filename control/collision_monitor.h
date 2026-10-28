#pragma once

#include <Eigen/Core>
#include <utility>
#include <vector>

#include "navigation2d/configuration/navigation_config.h"
#include "navigation2d/control/collision_latch.h"
#include "navigation2d/control/collision_monitor_types.h"
#include "navigation2d/control/depth_observation.h"
#include "navigation2d/control/range_observation.h"
#include "navigation2d/sensor/observations.h"

namespace navigation2d {

// Raw-sensor safety layer adapted from Nav2 Collision Monitor's independent
// stop/slowdown/approach architecture. It intentionally never reads a costmap.
class CollisionMonitor {
 public:
  explicit CollisionMonitor(NavigationConfig config) : config_(std::move(config)) {}
  void UpdateLaserScan(const Pose2d& sensor_pose, const LaserScan& scan);
  void UpdatePointCloud(const Pose2d& sensor_pose, const PointCloud2d& cloud);
  // Depth returns are already in the robot frame. They do not replace the
  // lidar scan and they do not use the lidar slowdown zone. A return that
  // stays inside 8 cm for several frames, then disappears from the forward
  // corridor, is held until the bumper passes it. The hold uses the same
  // stop-corridor test as a live return. Silence for 0.4 s drops the depth
  // points and the hold; the lidar source timeout is unchanged.
  void UpdateDepthPoints(const Pose2d& robot_pose, const std::vector<Eigen::Vector2d>& base_points,
                         double timestamp);
  CollisionMonitorResult Filter(const Pose2d& robot_pose, Twist2d command, double timestamp);

 private:
  NavigationConfig config_;
  collision_internal::RangeObservation range_;
  collision_internal::DepthObservation depth_;
  collision_internal::CollisionLatch latch_;
};

}  // namespace navigation2d
