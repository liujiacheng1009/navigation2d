#include <cassert>
#include <limits>

#include "navigation2d/control/collision_monitor.h"

int main() {
  navigation2d::NavigationConfig config;
  config.collision_monitor_trigger_cycles = 2;
  config.collision_monitor_release_cycles = 2;
  navigation2d::CollisionMonitor monitor(config);
  const auto before_observation =
      monitor.Filter(navigation2d::MakePose2d(0., 0., 0.), {.2, 0.}, 0.);
  assert(before_observation.action == navigation2d::CollisionMonitorAction::kSourceTimeout);
  navigation2d::LaserScan scan;
  scan.angle_min = 0.;
  scan.angle_increment = .1;
  scan.range_min = .02;
  scan.range_max = 5.;
  scan.ranges = {.29};
  const auto pose = navigation2d::MakePose2d(1., 1., 0.);
  monitor.UpdateLaserScan(pose, scan);
  auto result = monitor.Filter(pose, {.2, 0.}, 1.);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  monitor.UpdateLaserScan(pose, scan);
  result = monitor.Filter(pose, {.2, 0.}, 1.05);
  // Clearance is 0.29 - 0.18 = 0.11 m, just outside the 0.10 m stop distance.
  // The monitor must slow this command, not zero it.
  assert(result.action == navigation2d::CollisionMonitorAction::kSlowdown);
  assert(result.command.linear >= .05);

  scan.ranges = {std::numeric_limits<double>::infinity()};
  monitor.UpdateLaserScan(pose, scan);
  monitor.Filter(pose, {.2, 0.}, 1.10);
  monitor.UpdateLaserScan(pose, scan);
  result = monitor.Filter(pose, {.2, 0.}, 1.15);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  result = monitor.Filter(pose, {.2, 0.}, 2.);
  assert(result.action == navigation2d::CollisionMonitorAction::kSourceTimeout);
}
