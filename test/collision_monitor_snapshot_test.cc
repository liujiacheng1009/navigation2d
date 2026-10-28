#include <cassert>
#include <cmath>
#include <iostream>

#include "navigation2d/test/collision_monitor_workload.h"

int main() {
  std::cout << std::hexfloat;
  for (int variant = 0; variant < 16; ++variant) {
    navigation2d::CollisionMonitor monitor(navigation2d::test::CollisionWorkloadConfig(variant));
    for (const auto& result : navigation2d::test::CollisionWorkload(monitor)) {
      assert(std::isfinite(result.command.linear));
      assert(std::isfinite(result.command.angular));
      std::cout << variant << ' ' << static_cast<int>(result.action) << ' ' << result.command.linear
                << ' ' << result.command.angular << ' ' << result.min_distance_m << ' '
                << result.time_to_collision_s << '\n';
    }
  }
}
