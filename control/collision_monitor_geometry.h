#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "navigation2d/control/collision_monitor_parameters.h"
#include "navigation2d/geometry/pose_2d.h"
#include "navigation2d/geometry/twist_2d.h"

namespace navigation2d::collision_internal {
// Stateless geometry used by sensor retention and swept-motion filtering.
double TravelCorridorHalfWidth(const CollisionMonitorParameters& config);
double StopCorridorHalfWidth(const CollisionMonitorParameters& config);
double CommandedExtent(const CollisionMonitorParameters& config, double linear);
double SlowdownScale(double clearance, double time_to_collision,
                     const CollisionMonitorParameters& config);
// Per-return hot path: keeping the small admission checks visible here lets
// callers hoist frame/configuration invariants out of their sensor loops.
inline bool PointInsideOrNearFootprint(const Eigen::Vector2d& world_point, const Pose2d& pose,
                                       const CollisionMonitorParameters& config, double margin) {
  if (config.footprint.size() < 3) {
    return false;
  }
  const double yaw = Yaw(pose);
  const double c = std::cos(yaw), s = std::sin(yaw);
  const Eigen::Vector2d delta = world_point - pose.translation();
  const Eigen::Vector2d point(c * delta.x() + s * delta.y(), -s * delta.x() + c * delta.y());
  bool inside = false;
  double min_edge_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < config.footprint.size(); ++i) {
    const Eigen::Vector2d& a = config.footprint[i];
    const Eigen::Vector2d& b = config.footprint[(i + 1) % config.footprint.size()];
    const Eigen::Vector2d edge = b - a;
    const double edge_norm2 = edge.squaredNorm();
    const double t =
        edge_norm2 > 1e-12 ? std::clamp((point - a).dot(edge) / edge_norm2, 0., 1.) : 0.;
    min_edge_distance = std::min(min_edge_distance, (point - (a + t * edge)).norm());
    if ((a.y() > point.y()) != (b.y() > point.y()) &&
        point.x() < (b.x() - a.x()) * (point.y() - a.y()) / (b.y() - a.y()) + a.x()) {
      inside = !inside;
    }
  }
  return inside || min_edge_distance <= margin;
}

// A return on the chassis is the shell or a one-shot reflection. The measured
// footprint plus 2 cm covers the flats, about 0.16 m from the centre. Without
// a footprint the circular body is the shell. A real return farther out still
// stops.
// A supplied distance must belong to this exact point/robot-pose pair. The
// default computes it lazily, only for the circular-footprint branch.
inline bool ShellReturn(const Eigen::Vector2d& point, const Pose2d& pose,
                        const CollisionMonitorParameters& config,
                        std::optional<double> distance_to_robot = std::nullopt) {
  if (config.footprint.size() >= 3) {
    return PointInsideOrNearFootprint(point, pose, config, .02);
  }
  const double distance =
      distance_to_robot ? *distance_to_robot : (point - pose.translation()).norm();
  return distance <= std::max(.16, config.robot_radius);
}

Pose2d Integrate(const Pose2d& pose, const Twist2d& command, double dt);
}  // namespace navigation2d::collision_internal
