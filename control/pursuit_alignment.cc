#include "navigation2d/control/pursuit_alignment.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "navigation2d/control/pursuit_collision.h"
namespace navigation2d::pursuit_internal {
namespace {
double Angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double ApproachZero(double value, double delta) {
  return std::clamp(0., value - delta, value + delta);
}
}  // namespace
bool Alignment::ForwardProbeSafe(double heading, const Pose2d& pose,
                                 const LayeredCostmap& costmap) const {
  const Pose2d aligned = MakePose2d(X(pose), Y(pose), heading);
  return !pursuit_internal::CollisionImminent(config_.collision, aligned,
                                              {config_.regulated_min_speed, 0.}, costmap);
}
std::optional<double> Alignment::SelectHeading(const Path& path, std::size_t nearest,
                                               const Pose2d& pose,
                                               const LayeredCostmap& costmap) const {
  const auto incoming = path[nearest + 1].translation() - path[nearest].translation();
  double preferred = std::atan2(incoming.y(), incoming.x());
  double forward_arc = 0.;
  for (std::size_t index = nearest + 1; index + 1 < path.size(); ++index) {
    const auto segment = path[index + 1].translation() - path[index].translation();
    const double length = segment.norm();
    if (length <= 1e-9) {
      continue;
    }
    forward_arc += length;
    const double candidate = std::atan2(segment.y(), segment.x());
    if (std::abs(Angle(candidate - preferred)) > .10) {
      preferred = candidate;
      break;
    }
    if (forward_arc > config_.max_lookahead_distance) {
      break;
    }
  }
  constexpr std::array<double, 13> kHeadingOffsets{0.,  .10,  -.10, .20,  -.20, .35, -.35,
                                                   .50, -.50, .70,  -.70, .95,  -.95};
  for (const double offset : kHeadingOffsets) {
    const double candidate = Angle(preferred + offset);
    if (ForwardProbeSafe(candidate, pose, costmap)) {
      return candidate;
    }
  }
  return std::nullopt;
}
bool Alignment::Begin(const Path& path, std::size_t nearest, const Pose2d& pose,
                      const LayeredCostmap& costmap) {
  const auto heading = SelectHeading(path, nearest, pose, costmap);
  if (!heading) {
    mode_ = Mode::kRouteInfeasible;
    maneuver_ = ControllerManeuver::kRouteInfeasible;
    fallback_level_ = 7;
    return false;
  }
  recovery_heading_ = *heading;
  mode_ = Mode::kStopping;
  stopped_cycles_ = 0;
  aligned_cycles_ = 0;
  return true;
}
Twist2d Alignment::Brake(const Pose2d& pose, Twist2d current, const LayeredCostmap& costmap) {
  const double dv = config_.max_linear_acceleration * config_.collision.control_period;
  const double dw = config_.max_angular_acceleration * config_.collision.control_period;
  maneuver_ = ControllerManeuver::kStopping;
  intentional_stop_ = true;
  fallback_level_ = 5;
  Twist2d braking{ApproachZero(current.linear, dv), ApproachZero(current.angular, dw)};
  // Prefer physical deceleration. If even that short rollout enters an
  // obstacle, safety takes precedence and the velocity command is zeroed
  // immediately; the simulator/base driver then performs the hard stop.
  if (pursuit_internal::CollisionImminent(config_.collision, pose, braking, costmap)) {
    braking = {};
  }
  return braking;
}
Twist2d Alignment::StartAndBrake(const Path& path, std::size_t nearest, const Pose2d& pose,
                                 Twist2d current, const LayeredCostmap& costmap) {
  if (!Begin(path, nearest, pose, costmap)) {
    return {};
  }
  return Brake(pose, current, costmap);
}
std::optional<Twist2d> Alignment::Continue(const Path& path, std::size_t nearest,
                                           const Pose2d& pose, Twist2d current,
                                           const LayeredCostmap& costmap) {
  const double dw = config_.max_angular_acceleration * config_.collision.control_period;
  if (mode_ == Mode::kRouteInfeasible) {
    maneuver_ = ControllerManeuver::kRouteInfeasible;
    fallback_level_ = 7;
    return Twist2d{};
  }
  if (mode_ == Mode::kStopping) {
    const bool stopped = std::abs(current.linear) < .015 && std::abs(current.angular) < .05;
    stopped_cycles_ = stopped ? stopped_cycles_ + 1 : 0;
    if (stopped_cycles_ < 2) {
      return Brake(pose, current, costmap);
    }
    mode_ = Mode::kRotateToPath;
    aligned_cycles_ = 0;
  }
  if (mode_ == Mode::kRotateToPath) {
    maneuver_ = ControllerManeuver::kRotateToPath;
    fallback_level_ = 6;
    double heading_error = Angle(recovery_heading_ - Yaw(pose));
    const bool aligned = std::abs(heading_error) < .08;
    aligned_cycles_ = aligned ? aligned_cycles_ + 1 : 0;
    if (aligned_cycles_ >= 2) {
      // Revalidate the constructive forward probe because the local costmap
      // may have changed while braking and rotating.
      if (!ForwardProbeSafe(recovery_heading_, pose, costmap)) {
        return StartAndBrake(path, nearest, pose, current, costmap);
      }
      mode_ = Mode::kTracking;
      maneuver_ = ControllerManeuver::kTracking;
      fallback_level_ = 0;
      aligned_cycles_ = 0;
    } else {
      const double desired = std::clamp(2.2 * heading_error, -config_.max_angular_velocity,
                                        config_.max_angular_velocity);
      // Rotation shim semantics: translation is exactly zero. Applying the
      // linear acceleration clamp here would preserve residual forward speed
      // and reproduce the stage-4 corner collision this state resolves.
      return Twist2d{0., std::clamp(desired, current.angular - dw, current.angular + dw)};
    }
  }
  return std::nullopt;
}
}  // namespace navigation2d::pursuit_internal
