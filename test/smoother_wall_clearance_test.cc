#include <cassert>

#include "navigation2d/planning/collision_checker.h"
#include "navigation2d/planning/path_smoother.h"
#include "navigation2d/test/planner_test_maps.h"

int main() {
  auto [corridor_config, corridor_map] = navigation2d::test::CorridorPlannerMap("smoother_wall");
  navigation2d::Path hugging{
      navigation2d::MakePose2d(.50, .26, 0.), navigation2d::MakePose2d(1.00, .26, 0.),
      navigation2d::MakePose2d(1.50, .26, 0.), navigation2d::MakePose2d(2.00, .26, 0.),
      navigation2d::MakePose2d(2.50, .26, 0.)};
  const auto lifted = navigation2d::planning_internal::ConstrainedPathSmoother(corridor_config)
                          .Smooth(hugging, corridor_map, .22);
  assert(navigation2d::Y(lifted[2]) > navigation2d::Y(hugging[2]) + .02);
  const navigation2d::planning_internal::DistanceField corridor_field(corridor_map);
  assert(corridor_field.distance(navigation2d::X(lifted[2]), navigation2d::Y(lifted[2])) >= .275);
}
