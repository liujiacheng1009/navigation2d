#pragma once
#include <limits>

#include "navigation2d/geometry/pose_2d.h"
#include "navigation2d/geometry/twist_2d.h"
namespace navigation2d {
struct DockingSettings {
  double xy_tolerance_m, yaw_tolerance_rad;
  double linear_acceleration_mps2, angular_acceleration_rps2, period_s;
};
struct DockingOutcome {
  Twist2d command{};
  double yaw_error_rad = 0.;
  bool distance_progress = false, yaw_progress = false, succeeded = false;
};
// Centimetre-scale pose servo only. Owns docking progress/hysteresis, never
// applies safety filters or changes route/recovery state. Reset per new goal,
// not per route replan. Poses share a frame; distance is that goal's XY norm.
class DockingServo {
 public:
  explicit DockingServo(DockingSettings settings) : settings_(settings) {}
  void Reset();
  DockingOutcome Advance(const Pose2d& pose, const Pose2d& goal, double distance_m,
                         Twist2d previous_published);

 private:
  void NoteProgress(double distance_m, DockingOutcome& outcome);
  Twist2d TargetCommand(const Pose2d& pose, const Pose2d& goal, double distance_m,
                        double yaw_error_rad) const;
  Twist2d Slew(Twist2d target, Twist2d previous_published) const;
  DockingSettings settings_;
  double best_distance_m_ = std::numeric_limits<double>::infinity();
  double best_yaw_error_rad_ = std::numeric_limits<double>::infinity();
  bool position_reached_ = false;
};
}  // namespace navigation2d
