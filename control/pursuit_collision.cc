#include "navigation2d/control/pursuit_collision.h"

#include <cmath>
namespace navigation2d::pursuit_internal {
namespace {
double Angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
}  // namespace
bool CollisionImminent(const CollisionParameters& config, const Pose2d& pose, Twist2d command,
                       const LayeredCostmap& costmap) {
  // The controller uses the same circular footprint as the global planner.
  // Pure rotation is not exempt: if the current footprint is already in
  // contact (for example after a discretized corner cut), publishing another
  // rotation would only spin the wheels against the obstacle.
  const double collision_radius = config.robot_radius;
  Pose2d projected = pose;
  for (double t = 0.; t <= config.collision_horizon; t += config.control_period) {
    const double yaw = Angle(Yaw(projected) + command.angular * config.control_period);
    projected =
        MakePose2d(X(projected) + command.linear * std::cos(yaw) * config.control_period,
                   Y(projected) + command.linear * std::sin(yaw) * config.control_period, yaw);
    if (costmap.lethal(X(projected), Y(projected), collision_radius)) {
      return true;
    }
  }
  return false;
}
}  // namespace navigation2d::pursuit_internal
