#include "navigation2d/application/command_safety_pipeline.h"

#include <utility>

#include "navigation2d/control/dynamic_safety.h"
namespace navigation2d {
void CommandSafetyPipeline::UpdateLaserScan(const Pose2d& sensor, const LaserScan& scan) {
  monitor_.UpdateLaserScan(sensor, scan);
}
void CommandSafetyPipeline::UpdatePointCloud(const Pose2d& sensor, const PointCloud2d& cloud) {
  monitor_.UpdatePointCloud(sensor, cloud);
}
void CommandSafetyPipeline::UpdateDepthPoints(const Pose2d& robot,
                                              const std::vector<Eigen::Vector2d>& points,
                                              double stamp_s) {
  monitor_.UpdateDepthPoints(robot, points, stamp_s);
}
void CommandSafetyPipeline::SetPredictions(std::vector<PredictedObstacle> predictions) {
  predictions_ = std::move(predictions);
}
CollisionMonitorResult CommandSafetyPipeline::Filter(const Pose2d& robot, Twist2d command,
                                                     double stamp_s) {
  if (DynamicCollisionImminent(robot, command, predictions_, config_)) {
    command = {};
  }
  return monitor_.Filter(robot, command, stamp_s);
}
double CommandSafetyPipeline::DynamicTtc(const Pose2d& robot, Twist2d published) const {
  return MinimumDynamicTtc(robot, published, predictions_, config_);
}
}  // namespace navigation2d
