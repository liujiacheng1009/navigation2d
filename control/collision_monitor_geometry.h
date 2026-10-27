#pragma once
#include "navigation2d/configuration/navigation_config.h"
#include "navigation2d/geometry/pose_2d.h"
#include "navigation2d/geometry/twist_2d.h"

namespace navigation2d::collision_internal {
// Stateless geometry used by sensor retention and swept-motion filtering.
double TravelCorridorHalfWidth(const NavigationConfig& config);
double StopCorridorHalfWidth(const NavigationConfig& config);
double CommandedExtent(const NavigationConfig& config, double linear);
double SlowdownScale(double clearance, double time_to_collision, const NavigationConfig& config);
bool PointInsideOrNearFootprint(const Eigen::Vector2d& point, const Pose2d& pose,
                                const NavigationConfig& config, double margin);
bool ShellReturn(const Eigen::Vector2d& point, const Pose2d& pose, const NavigationConfig& config);
Pose2d Integrate(const Pose2d& pose, const Twist2d& command, double dt);
}  // namespace navigation2d::collision_internal
