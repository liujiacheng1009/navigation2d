#include "navigation2d/control/collision_monitor_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation2d::collision_internal {

double BodyHalfWidth(const NavigationConfig& config) {
  double half_width = 0.;
  for (const auto& corner : config.footprint) {
    half_width = std::max(half_width, std::abs(corner.y()));
  }
  if (half_width < 1e-6) {
    half_width = config.robot_radius;
  }
  return half_width;
}

// Lateral half-width of the commanded travel corridor. collision_radius adds
// another 0.10 m that is an along-track stop margin, not extra vehicle width.
// Using it as the corridor width treats a parallel wall as a forward obstacle:
// the slowdown latch then never releases, and every command is multiplied by
// slowdown_ratio before being floored at a crawl. The extra 0.05 m is only a
// slowdown margin so a corner beside the bumper eases the speed; it must not
// widen the stop zone.
double TravelCorridorHalfWidth(const NavigationConfig& config) {
  const double body = BodyHalfWidth(config);
  const double raster = std::sqrt(.5) * config.map_resolution;
  // Passage mode: body + one raster cell only. The extra 0.05 m open-space
  // margin turns door jambs into travel-corridor hits and latches STOP.
  if (config.collision_monitor_passage_mode) {
    return body + raster;
  }
  return body + raster + .05;
}

// Stop only for returns that overlap the body, plus one raster cell. A jamb
// a few centimetres outside the chassis is still inside the slowdown corridor
// and used to satisfy the radial stop test, which then latched a zero command
// for the rest of the traverse.
double StopCorridorHalfWidth(const NavigationConfig& config) {
  const double body = BodyHalfWidth(config);
  if (config.collision_monitor_passage_mode) {
    return body;
  }
  return body + std::sqrt(.5) * config.map_resolution;
}

// Distance from the robot origin to the bumper in the commanded direction.
// Without a footprint the circular planning radius is that bumper.
double CommandedExtent(const NavigationConfig& config, double linear) {
  if (config.footprint.size() >= 3) {
    double extent = 0.;
    for (const auto& corner : config.footprint) {
      const double axial = linear >= 0. ? corner.x() : -corner.x();
      extent = std::max(extent, axial);
    }
    if (extent > 1e-6) {
      return extent;
    }
  }
  return config.robot_radius;
}

// 1 at the outer edge of the slowdown zone, slowdown_ratio at the stop
// distance or at a one-cycle time-to-collision. A flat ratio makes a return
// 0.40 m away as slow as one 0.10 m away.
double SlowdownScale(double clearance, double time_to_collision, const NavigationConfig& config) {
  const double minimum = std::clamp(config.collision_monitor_slowdown_ratio, 0., 1.);
  double scale = 1.;
  bool applied = false;
  if (std::isfinite(clearance)) {
    const double span = std::max(
        1e-3, config.collision_monitor_slowdown_distance - config.collision_monitor_stop_distance);
    const double closeness =
        std::clamp((config.collision_monitor_slowdown_distance - clearance) / span, 0., 1.);
    scale = std::min(scale, 1. - closeness * (1. - minimum));
    applied = true;
  }
  if (std::isfinite(time_to_collision) && config.collision_monitor_approach_horizon > 1e-3) {
    const double closeness =
        std::clamp((config.collision_monitor_approach_horizon - time_to_collision) /
                       config.collision_monitor_approach_horizon,
                   0., 1.);
    scale = std::min(scale, 1. - closeness * (1. - minimum));
    applied = true;
  }
  return applied ? scale : minimum;
}

bool PointInsideOrNearFootprint(const Eigen::Vector2d& world_point, const Pose2d& pose,
                                const NavigationConfig& config, double margin) {
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
bool ShellReturn(const Eigen::Vector2d& point, const Pose2d& pose, const NavigationConfig& config) {
  if (config.footprint.size() >= 3) {
    return PointInsideOrNearFootprint(point, pose, config, .02);
  }
  return (point - pose.translation()).norm() <= std::max(.16, config.robot_radius);
}

Pose2d Integrate(const Pose2d& pose, const Twist2d& command, double dt) {
  const double yaw = Yaw(pose);
  const double next_yaw = yaw + command.angular * dt;
  if (std::abs(command.angular) < 1e-9) {
    return MakePose2d(X(pose) + command.linear * std::cos(yaw) * dt,
                      Y(pose) + command.linear * std::sin(yaw) * dt, next_yaw);
  }
  const double radius = command.linear / command.angular;
  return MakePose2d(X(pose) + radius * (std::sin(next_yaw) - std::sin(yaw)),
                    Y(pose) - radius * (std::cos(next_yaw) - std::cos(yaw)), next_yaw);
}

}  // namespace navigation2d::collision_internal
