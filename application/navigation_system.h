#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include "navigation2d/application/navigation_config.h"
#include "navigation2d/types.h"

namespace navigation2d {

enum class NavigationStatus { kIdle, kNavigating, kSucceeded, kBlocked };

struct NavigationState {
  NavigationStatus status = NavigationStatus::kIdle;
  Velocity command;
  int replans = 0;
  int emergency_stops = 0;
  int recoveries = 0;
  double global_path_length_m = 0.;
  std::uint64_t costmap_digest = 0;
};

// Product-facing navigation core. Pose/velocity come from localization and the
// base; range observations come from the real lidar adapter. No simulation or
// wall-clock ownership is allowed behind this interface.
class NavigationSystem {
 public:
  NavigationSystem(NavigationConfig config, const std::string& map_path);
  ~NavigationSystem();
  NavigationSystem(NavigationSystem&&) noexcept;
  NavigationSystem& operator=(NavigationSystem&&) noexcept;
  NavigationSystem(const NavigationSystem&) = delete;
  NavigationSystem& operator=(const NavigationSystem&) = delete;

  void SetGoal(Pose2d goal);
  void ClearGoal();
  void UpdateLaserScan(const Pose2d& sensor_pose, const LaserScan& scan);
  void UpdatePointCloud(const Pose2d& sensor_pose, const PointCloud2d& cloud);
  NavigationState ComputeCommand(const Pose2d& pose, Velocity measured_velocity,
                                 double timestamp);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace navigation2d
