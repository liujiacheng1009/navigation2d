#include "navigation2d/control/regulated_pure_pursuit.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/configuration/navigation_config.h"
#include "navigation2d/control/pursuit_collision.h"
#include "navigation2d/control/pursuit_regulation.h"
namespace navigation2d {
RegulatedPurePursuit::RegulatedPurePursuit(NavigationConfig config)
    : RegulatedPurePursuit(pursuit_internal::ParametersFrom(config), ParameterTag{}) {}
RegulatedPurePursuit::RegulatedPurePursuit(pursuit_internal::Parameters parameters, ParameterTag)
    : config_(parameters.tracking), alignment_(parameters.alignment) {}
Twist2d RegulatedPurePursuit::Compute(
    const Path& path, const Pose2d& pose, Twist2d current, const LayeredCostmap& costmap,
    const std::vector<PredictedObstacle>& dynamic_obstacles) const {
  (void)dynamic_obstacles;
  alignment_.BeginCycle();
  if (path.empty()) {
    return {};
  }
  if (progress_.Observe(path)) {
    alignment_.NewPath();
  }
  if (path.size() == 1) {
    return {};
  }
  const auto projection = progress_.Project(path, pose);
  const double nominal_lookahead =
      std::clamp(std::abs(current.linear) * config_.lookahead_time, config_.min_lookahead_distance,
                 config_.max_lookahead_distance);
  const auto target_for = [&](const Eigen::Vector2d& carrot) {
    return pursuit_internal::TrackingTarget(path, pose, carrot, costmap, config_.regulation);
  };
  const double dv = config_.max_linear_acceleration * config_.collision.control_period;
  const double dw = config_.max_angular_acceleration * config_.collision.control_period;
  if (const auto command = alignment_.Continue(path, projection.segment, pose, current, costmap)) {
    return *command;
  }
  // A globally valid polyline may require a tighter turn than its nominal
  // lookahead circle.  Search progressively shorter *path-relative* carrots
  // before declaring a stop.  This is a feasibility projection, not a speed
  // parameter: every candidate remains on the same footprint-checked route.
  double lookahead = nominal_lookahead;
  Twist2d target = target_for(pursuit_internal::CarrotAt(path, projection, lookahead));
  for (int attempt = 0; attempt < 4 && CollisionImminent(pose, target, costmap); ++attempt) {
    lookahead *= .5;
    target = target_for(pursuit_internal::CarrotAt(path, projection, lookahead));
  }
  if (CollisionImminent(pose, target, costmap)) {
    return alignment_.StartAndBrake(path, projection.segment, pose, current, costmap);
  }
  // Remember whether the geometric target was rotate-only before the accel
  // clamp reintroduces residual translation from the previous command.
  const bool rotate_only_target = std::abs(target.linear) < 1e-9;
  target.linear = std::clamp(target.linear, current.linear - dv, current.linear + dv);
  target.angular = std::clamp(target.angular, current.angular - dw, current.angular + dw);
  if (CollisionImminent(pose, target, costmap)) {
    // A pure-rotation request that only collides after the linear accel clamp
    // must stop and align — not binary-search a residual forward crawl into
    // the obstacle (shelf-corner / doorway yaw mismatch).
    if (rotate_only_target) {
      return alignment_.StartAndBrake(path, projection.segment, pose, current, costmap);
    }
    // Preserve the executable prefix: shorten the current control arc before
    // abandoning the route.  A binary search gives the largest safe command
    // without rejecting a path merely because its nominal horizon reaches a
    // corner.  This is local (eight collision checks), not a full-route scan.
    Twist2d safe{};
    double low = 0., high = 1.;
    for (int attempt = 0; attempt < 8; ++attempt) {
      const double scale = .5 * (low + high);
      const Twist2d candidate{target.linear * scale, target.angular * scale};
      if (CollisionImminent(pose, candidate, costmap)) {
        high = scale;
      } else {
        low = scale;
        safe = candidate;
      }
    }
    if (low > 1e-3) {
      return safe;
    }
    return alignment_.StartAndBrake(path, projection.segment, pose, current, costmap);
  }
  return target;
}
bool RegulatedPurePursuit::CollisionImminent(const Pose2d& pose, Twist2d command,
                                             const LayeredCostmap& costmap) const {
  return pursuit_internal::CollisionImminent(config_.collision, pose, command, costmap);
}
}  // namespace navigation2d
