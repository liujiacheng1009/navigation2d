#pragma once
#include <optional>

#include "navigation2d/application/execution_watchdog.h"
#include "navigation2d/application/navigation_motion.h"
#include "navigation2d/geometry/path_2d.h"
#include "navigation2d/planning/path_metadata.h"
namespace navigation2d {
struct AlignmentSettings {
  double linear_acceleration_mps2, angular_acceleration_rps2, maximum_angular_rps, period_s;
};
struct AlignmentInput {
  NavigationCycleObservation robot;
  const Path& path;
  const PlannedPathMetadata& metadata;
  double path_progress_m;
  Twist2d previous_published;
};
struct AlignmentCycle {
  // Empty motion means alignment is complete; tracking may run in this tick.
  std::optional<EvaluatedMotion> motion;
  NavigationFailure failure = NavigationFailure::kNone;
};
// Rotation-only alignment, except braking residual translation. Budget and
// progress remain owned by the shared single-writer watchdog component.
// A tracker sample from rest can slew in translation and look like a drive
// stall; alignment instead credits the initial rotation. Path has >= 2 poses.
class AlignmentExecution {
 public:
  explicit AlignmentExecution(AlignmentSettings settings) : settings_(settings) {}
  AlignmentCycle Advance(ExecutionWatchdog& watchdog, CommandSafetyPipeline& safety,
                         const AlignmentInput& input) const;

 private:
  AlignmentSettings settings_;
};
}  // namespace navigation2d
