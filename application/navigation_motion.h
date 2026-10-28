#pragma once

#include "navigation2d/control/collision_monitor.h"
#include "navigation2d/geometry/pose_2d.h"
#include "navigation2d/geometry/twist_2d.h"
namespace navigation2d {
class CommandSafetyPipeline;
struct NavigationState;
// Borrowed pose and measured velocity in one navigation frame; seconds in the
// caller's simulation/observation clock domain. Does not own a sensor cache.
struct NavigationCycleObservation {
  const Pose2d& pose;
  Twist2d measured_velocity;
  double timestamp_s;
};
struct EvaluatedMotion {
  Twist2d requested;
  CollisionMonitorResult monitored;
  bool requested_motion = false;
  bool safety_stopped = false;
  bool reached_base = false;
};
EvaluatedMotion EvaluateMotion(CommandSafetyPipeline& safety, const Pose2d& pose, Twist2d requested,
                               double timestamp_s);
// Snapshot projection only: no goal, watchdog, command output or TTC updates.
void CaptureMotion(NavigationState& snapshot, const EvaluatedMotion& motion);
}  // namespace navigation2d
