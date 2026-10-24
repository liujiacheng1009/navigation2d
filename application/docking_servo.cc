#include "navigation2d/application/docking_servo.h"

#include <algorithm>
#include <cmath>
namespace navigation2d {
namespace {
double NormalizeAngle(double value) { return std::atan2(std::sin(value), std::cos(value)); }
}  // namespace
void DockingServo::Reset() {
  best_distance_m_ = std::numeric_limits<double>::infinity();
  best_yaw_error_rad_ = std::numeric_limits<double>::infinity();
  position_reached_ = false;
}
void DockingServo::NoteProgress(double distance_m, DockingOutcome& outcome) {
  if (distance_m + .001 < best_distance_m_) {
    best_distance_m_ = distance_m;
    outcome.distance_progress = true;
  }
  if (distance_m <= settings_.xy_tolerance_m) {
    if (!position_reached_) {
      position_reached_ = true;
      best_yaw_error_rad_ = std::abs(outcome.yaw_error_rad);
      outcome.yaw_progress = true;
    } else if (std::abs(outcome.yaw_error_rad) + .01 < best_yaw_error_rad_) {
      best_yaw_error_rad_ = std::abs(outcome.yaw_error_rad);
      outcome.yaw_progress = true;
    }
  } else if (distance_m > settings_.xy_tolerance_m + .01) {
    position_reached_ = false;
    best_yaw_error_rad_ = std::numeric_limits<double>::infinity();
  }
}
Twist2d DockingServo::TargetCommand(const Pose2d& pose, const Pose2d& goal, double distance_m,
                                    double yaw_error_rad) const {
  Twist2d command;
  if (distance_m <= settings_.xy_tolerance_m) {
    command.angular = std::clamp(yaw_error_rad, -.35, .35);
    return command;
  }
  const auto delta = goal.translation() - pose.translation();
  const double forward_error = NormalizeAngle(std::atan2(delta.y(), delta.x()) - Yaw(pose));
  // Only the final docking servo chooses reverse, when the target is behind.
  const bool reverse = std::abs(forward_error) > .5 * std::acos(-1.);
  const double bearing_error =
      reverse ? NormalizeAngle(forward_error + std::acos(-1.)) : forward_error;
  if (std::abs(bearing_error) > .35) {
    command.angular = std::clamp(1.2 * bearing_error, -.35, .35);
  } else {
    const double speed = std::min(.08, std::max(.003, .4 * distance_m));
    command.linear = reverse ? -speed : speed;
    command.angular = std::clamp(1.2 * bearing_error, -.35, .35);
  }
  return command;
}
Twist2d DockingServo::Slew(Twist2d target, Twist2d published) const {
  const double dv = settings_.linear_acceleration_mps2 * settings_.period_s;
  const double dw = settings_.angular_acceleration_rps2 * settings_.period_s;
  // Slew from the command actually published, retaining the odom-wrap defense.
  target.linear = std::clamp(target.linear, published.linear - dv, published.linear + dv);
  target.angular = std::clamp(target.angular, published.angular - dw, published.angular + dw);
  return target;
}
DockingOutcome DockingServo::Advance(const Pose2d& pose, const Pose2d& goal, double distance_m,
                                     Twist2d published) {
  DockingOutcome outcome;
  outcome.yaw_error_rad = NormalizeAngle(Yaw(goal) - Yaw(pose));
  NoteProgress(distance_m, outcome);
  outcome.succeeded = distance_m <= settings_.xy_tolerance_m &&
                      std::abs(outcome.yaw_error_rad) <= settings_.yaw_tolerance_rad;
  if (!outcome.succeeded) {
    outcome.command = Slew(TargetCommand(pose, goal, distance_m, outcome.yaw_error_rad), published);
  }
  return outcome;
}
}  // namespace navigation2d
