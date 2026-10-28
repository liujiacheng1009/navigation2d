#include <cassert>
#include <fstream>

#include "navigation2d/control/safe_corridor.h"
#include "navigation2d/costmap/grid_2d.h"
#include "navigation2d/test/control_safety_scenes.h"

int main() {
  auto config = navigation2d::test::MonitorTestConfig();
  const char* map_path = "/tmp/navigation2d_safe_corridor_test.json";
  std::ofstream output(map_path);
  output << R"({"width":60,"height":40,"resolution":0.1,"cells":[)";
  for (int y = 0; y < 40; ++y) {
    for (int x = 0; x < 60; ++x) {
      const bool wall = x == 0 || y == 0 || x == 59 || y == 39;
      output << (x || y ? "," : "") << (wall ? 100 : 0);
    }
  }
  output << "]}";
  output.close();
  navigation2d::LayeredCostmap costmap(navigation2d::Grid2d::Load(map_path), config);
  navigation2d::Path path;
  for (int index = 0; index < 20; ++index) {
    path.push_back(navigation2d::MakePose2d(1. + .1 * index, 2., 0.));
  }
  const auto corridor = navigation2d::control_internal::BuildSafeCorridor(path, costmap, config);
  assert(!corridor.empty());
  for (const auto& halfspace : corridor.front()) {
    assert(halfspace.normal.dot(path.front().translation()) <= halfspace.bound + 1e-6);
  }
}
