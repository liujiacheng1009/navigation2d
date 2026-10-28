#include <iostream>

#include "navigation2d/test/costmap_parameter_workload.h"

int main() {
  for (int variant = 0; variant < 4; ++variant) {
    navigation2d::LayeredCostmap map(navigation2d::test::CostmapWorkloadGrid(),
                                     navigation2d::test::CostmapWorkloadConfig(variant));
    std::cout << variant;
    for (auto value : navigation2d::test::CostmapParameterWorkload(map)) {
      std::cout << ' ' << value;
    }
    std::cout << '\n';
  }
}
