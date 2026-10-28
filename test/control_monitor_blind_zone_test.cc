#include <cassert>
#include <limits>

#include "navigation2d/control/collision_monitor.h"
#include "navigation2d/test/control_safety_scenes.h"

int main() {
  auto config = navigation2d::test::MonitorTestConfig();
  auto scan = navigation2d::test::MonitorTestScan(.29);
  navigation2d::CollisionMonitorResult result;
  // A below-minimum lidar return is a near-field obstacle, not a free ray.
  // With the sensor 0.18 m ahead of the base, range_min=0.15 places the
  // synthetic return 0.33 m from the robot centre: a forward command must
  // stop, while a command moving away may release the latch.
  navigation2d::NavigationConfig blind_config = config;
  blind_config.collision_monitor_trigger_cycles = 1;
  blind_config.collision_monitor_release_cycles = 1;
  navigation2d::CollisionMonitor blind_monitor(blind_config);
  scan.angle_min = 0.;
  scan.angle_increment = .1;
  scan.range_min = .15;
  scan.ranges = {-std::numeric_limits<double>::infinity()};
  const auto base_pose = navigation2d::MakePose2d(0., 0., 0.);
  const auto sensor_pose = navigation2d::MakePose2d(.18, 0., 0.);
  blind_monitor.UpdateLaserScan(sensor_pose, scan);
  result = blind_monitor.Filter(base_pose, {.05, 0.}, 4.);
  assert(result.action == navigation2d::CollisionMonitorAction::kBlindZoneStop);
  assert(result.command.linear == 0.);
  result = blind_monitor.Filter(base_pose, {-.05, 0.}, 4.05);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  assert(result.command.linear < 0.);
}
