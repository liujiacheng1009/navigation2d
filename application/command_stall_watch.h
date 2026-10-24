#pragma once

#include "navigation2d/geometry/pose_2d.h"
#include "navigation2d/geometry/twist_2d.h"

namespace navigation2d {
struct StallObservation {
  const Pose2d& pose;
  double route_progress_m;
  Twist2d measured_velocity;
  double heading_error_rad;
  double timestamp_s;
  bool commanded;
  bool translation_expected;
};

// Single-writer, allocation-free accumulated-motion watchdog. Inputs share
// one map frame/time domain; measured velocity freshness belongs to the caller.
// Translation commands cannot use rotation alone as proof of progress.
class CommandStallWatch {
 public:
  explicit CommandStallWatch(double control_period_s) : control_period_s_(control_period_s) {}
  [[nodiscard]] bool Observe(const StallObservation& observation);
  void Reset();
  // Legacy recovery/goal handoff resets the cycle budget without moving the
  // time/pose anchor. Keep that distinct from a complete evidence reset.
  void RestartCycleBudget() { cycles_ = 0; }

 private:
  void Begin(const StallObservation& observation);
  bool MotionObserved(const StallObservation& observation);
  double control_period_s_;
  bool initialized_ = false;
  int cycles_ = 0;
  double started_s_ = 0., previous_s_ = 0.;
  Pose2d anchor_;
  double progress_m_ = 0., heading_error_rad_ = 0.;
  double measured_distance_m_ = 0., measured_rotation_rad_ = 0.;
};
}  // namespace navigation2d
