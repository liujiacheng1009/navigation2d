#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "navigation2d/application/navigation_execution_options.h"
#include "navigation2d/application/navigation_state.h"
#include "navigation2d/configuration/navigation_config.h"
#include "navigation2d/control/collision_monitor.h"
#include "navigation2d/control/local_controller.h"
#include "navigation2d/geometry/pose_2d.h"
#include "navigation2d/planning/global_planner.h"
#include "navigation2d/sensor/observations.h"

namespace navigation2d {

class AlgorithmRegistry;

// Product-facing navigation core. Pose/velocity come from localization and the
// base; range observations come from the real lidar adapter. No simulation or
// wall-clock ownership is allowed behind this interface.
class NavigationSystem {
 public:
  NavigationSystem(NavigationConfig config, const std::string& map_path);
  NavigationSystem(NavigationConfig config, const std::string& map_path,
                   std::shared_ptr<const AlgorithmRegistry> algorithms);
  NavigationSystem(NavigationConfig config, const std::string& map_path,
                   std::shared_ptr<const AlgorithmRegistry> algorithms,
                   NavigationExecutionOptions options);
  ~NavigationSystem();
  NavigationSystem(NavigationSystem&&) noexcept;
  NavigationSystem& operator=(NavigationSystem&&) noexcept;
  NavigationSystem(const NavigationSystem&) = delete;
  NavigationSystem& operator=(const NavigationSystem&) = delete;

  void SetGoal(Pose2d goal);
  void ClearGoal();
  void BanPathSignatures(std::vector<std::uint64_t> signatures);
  void UpdateLaserScan(const Pose2d& sensor_pose, const LaserScan& scan);
  // Keeps live range data out of the global static-map graph when an online
  // mapper already owns static occupancy integration.  CollisionMonitor
  // still guards every commanded motion.
  void UpdateCollisionMonitorLaserScan(const Pose2d& sensor_pose, const LaserScan& scan);
  void UpdateCollisionMonitorDepthPoints(const Pose2d& robot_pose,
                                         const std::vector<Eigen::Vector2d>& base_points,
                                         double timestamp);
  void UpdatePointCloud(const Pose2d& sensor_pose, const PointCloud2d& cloud);
  // Supplies tracker output in the map frame. Predictions are consumed by the
  // constrained MPC controller; legacy controllers safely ignore them.
  void UpdateDynamicObstacles(std::vector<PredictedObstacle> obstacles);
  // Ordered topology candidates produced by the pinned TUD Guidance Planner
  // adapter. The first safe feasible candidate has priority.
  void UpdateGuidanceCandidates(std::vector<GuidanceCandidate> candidates);
  NavigationState ComputeCommand(const Pose2d& pose, Twist2d measured_velocity,
                                 double timestamp);
  NavigationState ComputeCommand(const Pose2d& pose, Twist2d measured_velocity, double timestamp,
                                 const AlgorithmBudget& budget);
  const PlannedPathMetadata& path_metadata() const;
  // Active global route in the navigation/map frame (empty when idle).
  const Path& path() const;
  const std::string& planner_id() const;
  const std::string& controller_id() const;

 private:
  NavigationState ComputeCommandCycle(const Pose2d& pose, Twist2d measured_velocity,
                                      double timestamp, const AlgorithmBudget& budget);
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace navigation2d
