#include <cassert>

#include "navigation2d/control/dynamic_safety.h"

int main() {
  navigation2d::NavigationConfig config;
  config.control_period = .1;
  config.collision_horizon = 1.;
  const auto pose = navigation2d::MakePose2d(1., 1., 0.);
  const std::vector<navigation2d::PredictedObstacle> crossing{{1.65, .55, 0., .5, .16, .02, .02}};
  // The obstacle is initially off the path, but intersects it in the command
  // horizon. A snapshot-only collision check would incorrectly accept this.
  assert(navigation2d::DynamicCollisionImminent(pose, {.3, 0.}, crossing, config));
  assert(!navigation2d::DynamicCollisionImminent(pose, {.0, 0.}, crossing, config));
}
