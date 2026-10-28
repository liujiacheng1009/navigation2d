#include <cassert>
#include <cmath>
#include <limits>

#include "navigation2d/control/collision_monitor.h"
#include "navigation2d/test/control_safety_scenes.h"

int main() {
  auto corner_stop_config = navigation2d::test::PolygonMonitorTestConfig();
  navigation2d::CollisionMonitorResult result;
  auto bumper_scan = navigation2d::test::MonitorTestScan(.145 + .09, 8.);
  // Near-field depth stops inside the bumper gap and does not use the
  // lidar slowdown zone. A one-frame return is not held. A return seen for
  // three frames is held, and that hold stops only a closing forward command.
  // 0.4 s of silence clears it without a lidar source timeout.
  navigation2d::NavigationConfig depth_config = corner_stop_config;
  depth_config.collision_monitor_trigger_cycles = 1;
  depth_config.collision_monitor_release_cycles = 1;
  navigation2d::CollisionMonitor depth_monitor(depth_config);
  navigation2d::LaserScan clear_scan = bumper_scan;
  clear_scan.ranges = {std::numeric_limits<double>::infinity()};
  const auto depth_origin = navigation2d::MakePose2d(0., 0., 0.);
  depth_monitor.UpdateLaserScan(depth_origin, clear_scan);
  depth_monitor.UpdateDepthPoints(depth_origin, {{.90, 0.}}, 7.);
  result = depth_monitor.Filter(depth_origin, {.24, 0.}, 7.);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  assert(std::abs(result.command.linear - .24) < 1e-9);

  for (int sample = 0; sample < 3; ++sample) {
    depth_monitor.UpdateDepthPoints(depth_origin, {{.65, 0.}}, 7.05 + .01 * sample);
  }
  result = depth_monitor.Filter(depth_origin, {.24, 0.}, 7.08);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  assert(std::abs(result.command.linear - .24) < 1e-9);
  depth_monitor.UpdateDepthPoints(depth_origin, {}, 7.10);
  result = depth_monitor.Filter(depth_origin, {.24, 0.}, 7.10);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  assert(std::abs(result.command.linear - .24) < 1e-9);

  navigation2d::CollisionMonitor depth_stop(depth_config);
  depth_stop.UpdateLaserScan(depth_origin, clear_scan);
  const double stop_point = .145 + .09;
  depth_stop.UpdateDepthPoints(depth_origin, {{stop_point, 0.}}, 7.20);
  depth_stop.UpdateDepthPoints(depth_origin, {}, 7.21);
  result = depth_stop.Filter(depth_origin, {.24, 0.}, 7.21);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);

  for (int sample = 0; sample < 3; ++sample) {
    depth_stop.UpdateDepthPoints(depth_origin, {{stop_point, 0.}}, 7.30 + .01 * sample);
  }
  result = depth_stop.Filter(depth_origin, {.24, 0.}, 7.33);
  assert(result.action == navigation2d::CollisionMonitorAction::kStop);
  assert(result.command.linear == 0.);
  depth_stop.UpdateDepthPoints(depth_origin, {}, 7.34);
  result = depth_stop.Filter(depth_origin, {.24, 0.}, 7.34);
  assert(result.action == navigation2d::CollisionMonitorAction::kStop);
  assert(result.command.linear == 0.);
  result = depth_stop.Filter(depth_origin, {0., .4}, 7.35);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  assert(std::abs(result.command.angular - .4) < 1e-9);

  const auto passed = navigation2d::MakePose2d(.60, 0., 0.);
  depth_stop.UpdateLaserScan(passed, clear_scan);
  result = depth_stop.Filter(passed, {.24, 0.}, 7.36);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  assert(std::abs(result.command.linear - .24) < 1e-9);

  navigation2d::CollisionMonitor depth_timeout(depth_config);
  depth_timeout.UpdateLaserScan(depth_origin, clear_scan);
  for (int sample = 0; sample < 3; ++sample) {
    depth_timeout.UpdateDepthPoints(depth_origin, {{stop_point, 0.}}, 8. + .01 * sample);
  }
  depth_timeout.UpdateDepthPoints(depth_origin, {}, 8.04);
  result = depth_timeout.Filter(depth_origin, {.24, 0.}, 8.04);
  assert(result.action == navigation2d::CollisionMonitorAction::kStop);
  depth_timeout.UpdateLaserScan(depth_origin, clear_scan);
  result = depth_timeout.Filter(depth_origin, {.24, 0.}, 8.50);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  assert(result.action != navigation2d::CollisionMonitorAction::kSourceTimeout);
  assert(std::abs(result.command.linear - .24) < 1e-9);

  navigation2d::CollisionMonitor depth_side(depth_config);
  depth_side.UpdateLaserScan(depth_origin, clear_scan);
  depth_side.UpdateDepthPoints(depth_origin, {{.40, .40}}, 9.);
  result = depth_side.Filter(depth_origin, {.24, 0.}, 9.);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  assert(std::abs(result.command.linear - .24) < 1e-9);
}
