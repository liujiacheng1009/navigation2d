#include <cassert>
#include <cmath>

#include "navigation2d/control/collision_monitor.h"
#include "navigation2d/test/control_safety_scenes.h"

int main() {
  auto config = navigation2d::test::MonitorTestConfig();
  navigation2d::CollisionMonitorResult result;
  // A parallel wall beside the chassis used to sit inside collision_radius
  // (~0.30 m) and latch the flat 0.35 slowdown for the whole traverse.
  navigation2d::NavigationConfig side_config = config;
  side_config.collision_monitor_trigger_cycles = 1;
  side_config.collision_monitor_release_cycles = 1;
  navigation2d::CollisionMonitor side_monitor(side_config);
  navigation2d::LaserScan side_scan;
  side_scan.angle_min = std::atan2(.30, .40);
  side_scan.angle_increment = .1;
  side_scan.range_min = .02;
  side_scan.range_max = 5.;
  side_scan.ranges = {std::hypot(.40, .30)};
  const auto origin = navigation2d::MakePose2d(0., 0., 0.);
  side_monitor.UpdateLaserScan(origin, side_scan);
  result = side_monitor.Filter(origin, {.24, 0.}, 5.);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  assert(std::abs(result.command.linear - .24) < 1e-9);

  // A return directly ahead scales with clearance and stays far above the
  // old 0.035 m/s crawl. Clearance here is 0.40 - 0.18 = 0.22 m (inside the
  // 0.28 m slowdown band; farther returns must not throttle open travel).
  navigation2d::CollisionMonitor ahead_monitor(side_config);
  navigation2d::LaserScan ahead_scan = side_scan;
  ahead_scan.angle_min = 0.;
  ahead_scan.ranges = {.40};
  ahead_monitor.UpdateLaserScan(origin, ahead_scan);
  result = ahead_monitor.Filter(origin, {.24, 0.}, 5.);
  assert(result.action == navigation2d::CollisionMonitorAction::kSlowdown);
  assert(result.command.linear > .12);
  assert(result.command.linear < .24);

  // Clearance 0.36 - 0.18 = 0.18 m on an already regulated 0.08 m/s command.
  // The old 0.35 scale plus 0.035 m/s floor turned this into the recorded crawl.
  navigation2d::CollisionMonitor floor_monitor(side_config);
  ahead_scan.ranges = {.36};
  floor_monitor.UpdateLaserScan(origin, ahead_scan);
  result = floor_monitor.Filter(origin, {.08, 0.}, 5.2);
  assert(result.action == navigation2d::CollisionMonitorAction::kSlowdown);
  assert(result.command.linear >= .08 - 1e-9);

  // A return inside the stop distance still zeroes the command.
  navigation2d::CollisionMonitor near_monitor(side_config);
  ahead_scan.ranges = {.27};
  near_monitor.UpdateLaserScan(origin, ahead_scan);
  result = near_monitor.Filter(origin, {.24, 0.}, 5.3);
  assert(result.action == navigation2d::CollisionMonitorAction::kStop);
  assert(result.command.linear == 0.);
}
