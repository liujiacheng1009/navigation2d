#include "navigation2d/costmap/costmap_parameters.h"

#include <cassert>

#include "navigation2d/test/costmap_parameter_workload.h"

int main() {
  for (int variant = 0; variant < 4; ++variant) {
    auto config = navigation2d::test::CostmapWorkloadConfig(variant);
    navigation2d::LayeredCostmap legacy(navigation2d::test::CostmapWorkloadGrid(), config);
    auto scoped = navigation2d::LayeredCostmap::WithParameters(
        navigation2d::test::CostmapWorkloadGrid(), navigation2d::CostmapParametersFrom(config));
    // Both maps are construction-time snapshots independent of later caller edits.
    config.robot_radius = 2.;
    config.inflation_radius = 3.;
    config.local_window_width = .1;
    assert(navigation2d::test::CostmapParameterWorkload(legacy) ==
           navigation2d::test::CostmapParameterWorkload(scoped));
  }
}
