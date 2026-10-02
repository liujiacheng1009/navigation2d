#pragma once

#include <vector>

#include <Eigen/Core>

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/control/local_controller.h"
#include "navigation2d/sensor/observations.h"

namespace navigation2d {

enum class CollisionMonitorAction {
  kNone,
  kSlowdown,
  kStop,
  // The lidar reported a return below its declared minimum range.  This is
  // not an ordinary clear ray: it means an obstacle is in the sensor blind
  // zone, so a command moving toward that return must be stopped.
  kBlindZoneStop,
  kSourceTimeout,
};

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
  // Depth returns are already in the robot frame. They do not replace the
  // lidar scan and they do not use the lidar slowdown zone. A return that
  // stays inside 8 cm for several frames, then disappears from the forward
  // corridor, is held until the bumper passes it. The hold uses the same
  // stop-corridor test as a live return. Silence for 0.4 s drops the depth
  // points and the hold; the lidar source timeout is unchanged.
  void UpdateDepthPoints(const Pose2d& robot_pose,
                         const std::vector<Eigen::Vector2d>& base_points,
                         double timestamp);
  CollisionMonitorResult Filter(const Pose2d& robot_pose, Twist2d command,
                                double timestamp);

 private:
  struct DepthTrack {
    Eigen::Vector2d point = Eigen::Vector2d::Zero();
    int hits = 0;
  };
  void UpdatePoints(const Pose2d& sensor_pose, std::vector<Eigen::Vector2d> points);
  void CullPassedDepthHolds(const Pose2d& robot_pose);
  NavigationConfig config_;
  std::vector<Eigen::Vector2d> points_;
  std::vector<Eigen::Vector2d> blind_zone_points_;
  std::vector<Eigen::Vector2d> depth_points_;
  std::vector<Eigen::Vector2d> held_depth_points_;
  std::vector<DepthTrack> depth_tracks_;
  bool has_depth_stamp_ = false;
  double depth_stamp_ = 0.;
  bool observation_pending_ = false;
  bool has_observation_ = false;
  double last_observation_time_ = 0.;
  int trigger_count_ = 0;
  int release_count_ = 0;
  CollisionMonitorAction latched_action_ = CollisionMonitorAction::kNone;
};

}  // namespace navigation2d
