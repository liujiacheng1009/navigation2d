#pragma once
#include "navigation2d/control/pursuit_parameters.h"
#include "navigation2d/costmap/layered_costmap.h"
#include "navigation2d/geometry/pose_2d.h"
#include "navigation2d/geometry/twist_2d.h"
namespace navigation2d::pursuit_internal {
// Validated positive control period; horizon seconds, radius metres. Rotation
// receives the same footprint check as translation. Sampling preserves the
// legacy update-then-test order.
bool CollisionImminent(const CollisionParameters&, const Pose2d&, Twist2d, const LayeredCostmap&);
}  // namespace navigation2d::pursuit_internal
