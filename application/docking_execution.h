#pragma once
#include <optional>

#include "navigation2d/application/docking_servo.h"
#include "navigation2d/application/execution_watchdog.h"
#include "navigation2d/application/navigation_motion.h"
namespace navigation2d {
struct DockingInput {
  NavigationCycleObservation robot;
  const Pose2d& goal;
  double goal_distance_m;
  Twist2d previous_published;
};
struct DockingCycle {
  bool succeeded = false;
  std::optional<EvaluatedMotion> motion;
  NavigationFailure failure = NavigationFailure::kNone;
};
// Owns the precision servo's hysteresis. Reset only for a new goal; safety
// filtering and watchdog consumption retain their original causal order.
class DockingExecution {
 public:
  explicit DockingExecution(DockingSettings settings) : servo_(settings) {}
  void Reset() { servo_.Reset(); }
  DockingCycle Advance(ExecutionWatchdog& watchdog, CommandSafetyPipeline& safety,
                       const DockingInput& input);

 private:
  DockingServo servo_;
};
}  // namespace navigation2d
