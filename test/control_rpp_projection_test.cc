#include <cassert>

#include "navigation2d/control/regulated_pure_pursuit.h"
#include "navigation2d/test/control_safety_scenes.h"

int main() {
  auto config = navigation2d::test::MonitorTestConfig();
  auto costmap = navigation2d::test::ControlCorridorMap(config, "projection");
  // A future branch passes closer to the slightly off-route robot than the
  // active branch. RPP must retain the local arc coordinate instead of
  // teleporting its projection to that future, westbound segment.
  navigation2d::Path self_near;
  for (int index = 0; index <= 20; ++index) {
    self_near.push_back(navigation2d::MakePose2d(1. + .1 * index, 2., 0.));
  }
  for (int index = 1; index <= 10; ++index) {
    self_near.push_back(navigation2d::MakePose2d(3., 2. + .1 * index, 1.57));
  }
  self_near.push_back(navigation2d::MakePose2d(3., 2.05, -1.57));
  for (int index = 1; index <= 20; ++index) {
    self_near.push_back(navigation2d::MakePose2d(3. - .1 * index, 2.05, 3.14));
  }
  navigation2d::RegulatedPurePursuit rpp(config);
  const auto command = rpp.Compute(self_near, navigation2d::MakePose2d(1.2, 2.06, 0.), {}, costmap);
  assert(command.linear > 0.);
  assert(command.angular < 0.);
}
