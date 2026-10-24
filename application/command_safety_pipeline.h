#pragma once
#include <vector>

#include "navigation2d/control/collision_monitor.h"
namespace navigation2d {
// Owns sensor safety state and dynamic predictions. Configuration is borrowed
// read-only from NavigationSystem's earlier-declared, stable-lifetime config;
// callers must retain it for the whole pipeline lifetime. No ROS or costmap.
class CommandSafetyPipeline {
 public:
  explicit CommandSafetyPipeline(const NavigationConfig& config)
      : config_(config), monitor_(config) {}
  CommandSafetyPipeline(NavigationConfig&&) = delete;
  CommandSafetyPipeline(const NavigationConfig&&) = delete;
  void UpdateLaserScan(const Pose2d& sensor, const LaserScan& scan);
  void UpdatePointCloud(const Pose2d& sensor, const PointCloud2d& cloud);
  void UpdateDepthPoints(const Pose2d& robot, const std::vector<Eigen::Vector2d>& points,
                         double stamp_s);
  void SetPredictions(std::vector<PredictedObstacle> predictions);
  const std::vector<PredictedObstacle>& predictions() const { return predictions_; }
  // Dynamic veto precedes raw monitoring even when it produces zero; that
  // monitoring call still consumes fresh observations and advances latches.
  CollisionMonitorResult Filter(const Pose2d& robot, Twist2d command, double stamp_s);
  // Deliberately separate: tracking computes TTC only after its legacy stall
  // exits, whereas docking/alignment expose only the raw monitor's TTC.
  double DynamicTtc(const Pose2d& robot, Twist2d published) const;

 private:
  const NavigationConfig& config_;
  CollisionMonitor monitor_;
  std::vector<PredictedObstacle> predictions_;
};
}  // namespace navigation2d
