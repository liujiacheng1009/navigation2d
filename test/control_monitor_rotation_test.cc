#include <cassert>

#include "navigation2d/control/collision_monitor.h"
#include "navigation2d/test/control_safety_scenes.h"

int main() {
  auto config = navigation2d::test::MonitorTestConfig();
  auto scan = navigation2d::test::MonitorTestScan(.29);
  const auto pose = navigation2d::MakePose2d(1., 1., 0.);
  navigation2d::CollisionMonitorResult result;
  // A noisy return already inside the circular body (radius 0.18 m) must not
  // make motion impossible: it is removed by footprint clearing.
  navigation2d::NavigationConfig rotation_config = config;
  rotation_config.collision_monitor_trigger_cycles = 1;
  navigation2d::CollisionMonitor rotation_monitor(rotation_config);
  scan.ranges = {.12};
  rotation_monitor.UpdateLaserScan(pose, scan);
  result = rotation_monitor.Filter(pose, {0., .4}, 3.);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  assert(result.command.angular == .4);
  rotation_monitor.UpdateLaserScan(pose, scan);
  result = rotation_monitor.Filter(pose, {.2, 0.}, 3.05);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
}
