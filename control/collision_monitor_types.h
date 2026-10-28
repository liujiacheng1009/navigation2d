#pragma once
#include "navigation2d/geometry/twist_2d.h"

namespace navigation2d {
enum class CollisionMonitorAction {
  kNone,
  kSlowdown,
  kStop,
  // The lidar reported a return below its declared minimum range.  This is
  // not an ordinary clear ray: it means an obstacle is in the sensor blind
  // zone, so a command moving toward that return must be stopped.
  kBlindZoneStop,
  kSourceTimeout,
};

struct CollisionMonitorResult {
  Twist2d command;
  CollisionMonitorAction action = CollisionMonitorAction::kNone;
  double min_distance_m = 0.;
  double time_to_collision_s = 0.;
};

}  // namespace navigation2d
