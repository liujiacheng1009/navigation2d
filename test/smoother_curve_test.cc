#include <cassert>

#include "navigation2d/planning/path_smoother.h"
#include "navigation2d/test/planner_test_maps.h"

int main() {
  auto map = navigation2d::test::CommonPlannerMap("smoother_curve", true);
  navigation2d::NavigationConfig smoother_config;
  smoother_config.smoother_max_curvature = 100.;
  smoother_config.smoother_min_clearance = .2;
  const navigation2d::Path zigzag{navigation2d::MakePose2d(3., 1., 0.),
                                  navigation2d::MakePose2d(3.2, 1.2, 0.),
                                  navigation2d::MakePose2d(3.4, 1., 0.)};
  const auto smooth = navigation2d::planning_internal::ConstrainedPathSmoother(smoother_config)
                          .Smooth(zigzag, map, .18);
  assert(smooth.size() == zigzag.size());
  assert(navigation2d::Y(smooth[1]) < navigation2d::Y(zigzag[1]));
  assert((smooth[1].translation() - zigzag[1].translation()).norm() <=
         smoother_config.smoother_max_deviation + 1e-12);
  assert(navigation2d::X(smooth.front()) == navigation2d::X(zigzag.front()));
  assert(navigation2d::X(smooth.back()) == navigation2d::X(zigzag.back()));
}
