#include <cassert>
#include <cmath>
#include <fstream>
#include <limits>

#include "navigation2d/control/collision_monitor.h"
#include "navigation2d/control/safe_corridor.h"
#include "navigation2d/costmap/grid_2d.h"

int main() {
  navigation2d::NavigationConfig config;
  config.collision_monitor_trigger_cycles = 2;
  config.collision_monitor_release_cycles = 2;
  navigation2d::CollisionMonitor monitor(config);
  const auto before_observation = monitor.Filter(
      navigation2d::MakePose2d(0., 0., 0.), {.2, 0.}, 0.);
  assert(before_observation.action == navigation2d::CollisionMonitorAction::kSourceTimeout);
  navigation2d::LaserScan scan;
  scan.angle_min = 0.; scan.angle_increment = .1;
  scan.range_min = .02; scan.range_max = 5.; scan.ranges = {.20};
  const auto pose = navigation2d::MakePose2d(1., 1., 0.);
  monitor.UpdateLaserScan(pose, scan);
  auto result = monitor.Filter(pose, {.2, 0.}, 1.);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  monitor.UpdateLaserScan(pose, scan);
  result = monitor.Filter(pose, {.2, 0.}, 1.05);
  assert(result.action == navigation2d::CollisionMonitorAction::kStop);
  assert(result.command.linear == 0.);

  scan.ranges = {std::numeric_limits<double>::infinity()};
  monitor.UpdateLaserScan(pose, scan); monitor.Filter(pose, {.2, 0.}, 1.10);
  monitor.UpdateLaserScan(pose, scan); result = monitor.Filter(pose, {.2, 0.}, 1.15);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  result = monitor.Filter(pose, {.2, 0.}, 2.);
  assert(result.action == navigation2d::CollisionMonitorAction::kSourceTimeout);

  const char* map_path = "/tmp/navigation2d_safe_corridor_test.json";
  std::ofstream output(map_path);
  output << R"({"width":60,"height":40,"resolution":0.1,"cells":[)";
  for (int y = 0; y < 40; ++y) for (int x = 0; x < 60; ++x) {
    const bool wall = x == 0 || y == 0 || x == 59 || y == 39;
    output << (x || y ? "," : "") << (wall ? 100 : 0);
  }
  output << "]}"; output.close();
  navigation2d::LayeredCostmap costmap(navigation2d::Grid2d::Load(map_path), config);
  navigation2d::Path path;
  for (int index = 0; index < 20; ++index)
    path.push_back(navigation2d::MakePose2d(1. + .1 * index, 2., 0.));
  const auto corridor = navigation2d::control_internal::BuildSafeCorridor(path, costmap, config);
  assert(!corridor.empty());
  for (const auto& halfspace : corridor.front())
    assert(halfspace.normal.dot(path.front().translation()) <= halfspace.bound + 1e-6);
}
