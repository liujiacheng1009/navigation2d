#include <cassert>
#include <cmath>

#include "navigation2d/control/collision_monitor.h"
#include "navigation2d/test/control_safety_scenes.h"

int main() {
  auto side_config = navigation2d::test::MonitorTestConfig(1, 1);
  const auto origin = navigation2d::MakePose2d(0., 0., 0.);
  navigation2d::CollisionMonitorResult result;
  // Recorded exploration stop, bag 045634-49f99c62 at t=80.6 s. The closest
  // return sits beside the front corner (body half-width 0.147 m, point at
  // y=0.194 m) while the lane ahead of the bumper is clear. Radial clearance
  // against the 0.22 m planning circle was 0.08 m, inside the 0.10 m stop
  // distance, so the old monitor latched a zero command for the rest of the
  // goal. This return may slow the robot; it must not stop it.
  navigation2d::NavigationConfig corner_stop_config = side_config;
  corner_stop_config.robot_radius = .22;
  corner_stop_config.map_resolution = .05;
  corner_stop_config.footprint = {{-.145, -.147}, {.115, -.147}, {.145, -.132},
                                  {.145, .132},   {.115, .147},  {-.145, .147}};
  corner_stop_config.regulated_min_speed = .08;
  navigation2d::CollisionMonitor corner_monitor(corner_stop_config);
  navigation2d::LaserScan corner_scan;
  corner_scan.angle_min = std::atan2(.194, .224);
  corner_scan.angle_increment = .1;
  corner_scan.range_min = .02;
  corner_scan.range_max = 8.;
  corner_scan.ranges = {std::hypot(.224, .194)};
  corner_monitor.UpdateLaserScan(origin, corner_scan);
  result = corner_monitor.Filter(origin, {.24, 0.}, 6.);
  assert(result.action != navigation2d::CollisionMonitorAction::kStop);
  assert(result.action != navigation2d::CollisionMonitorAction::kBlindZoneStop);
  assert(result.command.linear >= .08 - 1e-9);

  // Kitchen doorway: jambs sit in the travel band (~0.20 m lateral, 0.30 m
  // ahead) while the centerline is free. Predicted footprint contact used to
  // latch STOP on those side returns and park the robot on the threshold.
  navigation2d::CollisionMonitor door_monitor(corner_stop_config);
  navigation2d::LaserScan door_scan;
  door_scan.angle_min = -std::atan2(.20, .30);
  door_scan.angle_increment = 2. * std::atan2(.20, .30);
  door_scan.range_min = .02;
  door_scan.range_max = 8.;
  door_scan.ranges = {std::hypot(.30, .20), std::hypot(.30, .20)};
  door_monitor.UpdateLaserScan(origin, door_scan);
  result = door_monitor.Filter(origin, {.20, 0.}, 6.05);
  assert(result.action != navigation2d::CollisionMonitorAction::kStop);
  assert(result.action != navigation2d::CollisionMonitorAction::kBlindZoneStop);
  assert(result.command.linear > 0.);

  // The same chassis still stops for a return 0.09 m in front of the bumper.
  navigation2d::CollisionMonitor bumper_monitor(corner_stop_config);
  navigation2d::LaserScan bumper_scan = corner_scan;
  bumper_scan.angle_min = 0.;
  bumper_scan.ranges = {.145 + .09};
  bumper_monitor.UpdateLaserScan(origin, bumper_scan);
  result = bumper_monitor.Filter(origin, {.24, 0.}, 6.1);
  assert(result.action == navigation2d::CollisionMonitorAction::kStop);
  assert(result.command.linear == 0.);

  // A return on the shell (0.15 m, bumper at 0.145 m) is the chassis, not a
  // wall. It must not stop a drive across an open floor.
  navigation2d::CollisionMonitor shell_monitor(corner_stop_config);
  navigation2d::LaserScan shell_scan = corner_scan;
  shell_scan.angle_min = 0.;
  shell_scan.ranges = {.15};
  shell_monitor.UpdateLaserScan(origin, shell_scan);
  result = shell_monitor.Filter(origin, {.24, 0.}, 6.2);
  assert(result.action != navigation2d::CollisionMonitorAction::kStop);
  assert(result.action != navigation2d::CollisionMonitorAction::kBlindZoneStop);
  assert(std::abs(result.command.linear - .24) < 1e-9);
}
