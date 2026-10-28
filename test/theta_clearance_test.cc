#include <cassert>
#include <cmath>

#include "navigation2d/planning/theta_star_planner.h"
#include "navigation2d/test/planner_test_maps.h"

int main() {
  auto [corridor_config, corridor_map] = navigation2d::test::CorridorPlannerMap();
  // Clearance-aware traversal: equal-length segment farther from the wall is
  // cheaper, so Theta* prefers the open lane over the inflated edge.
  navigation2d::ThetaStarPlanner theta(.22);
  const auto open_lane = theta.Plan(corridor_map, navigation2d::MakePose2d(.40, 1.00, 0.),
                                    navigation2d::MakePose2d(2.60, 1.00, 0.));
  double mid_y_sum = 0.;
  int mid_count = 0;
  for (const auto& sample : open_lane) {
    if (navigation2d::X(sample) < .80 || navigation2d::X(sample) > 2.20) {
      continue;
    }
    mid_y_sum += navigation2d::Y(sample);
    ++mid_count;
  }
  assert(mid_count > 0);
  assert(std::abs(mid_y_sum / mid_count - 1.00) < .25);
}
