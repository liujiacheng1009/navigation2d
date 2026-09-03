#include <cassert>
#include <cmath>
#include <fstream>
#include <limits>

#include "navigation2d/control/collision_monitor.h"
#include "navigation2d/control/regulated_pure_pursuit.h"
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
  scan.range_min = .02; scan.range_max = 5.; scan.ranges = {.29};
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

  // A noisy return already inside a circular footprint must not make motion
  // impossible: it is removed by footprint clearing. The test above still
  // proves that a point initially outside and newly swept by motion stops.
  navigation2d::NavigationConfig rotation_config = config;
  rotation_config.collision_monitor_trigger_cycles = 1;
  navigation2d::CollisionMonitor rotation_monitor(rotation_config);
  scan.ranges = {.20};
  rotation_monitor.UpdateLaserScan(pose, scan);
  result = rotation_monitor.Filter(pose, {0., .4}, 3.);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  assert(result.command.angular == .4);
  rotation_monitor.UpdateLaserScan(pose, scan);
  result = rotation_monitor.Filter(pose, {.2, 0.}, 3.05);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);

  // A below-minimum lidar return is a near-field obstacle, not a free ray.
  // With the sensor 0.18 m ahead of the base, range_min=0.15 places the
  // synthetic return 0.33 m from the robot centre: a forward command must
  // stop, while a command moving away may release the latch.
  navigation2d::NavigationConfig blind_config = config;
  blind_config.collision_monitor_trigger_cycles = 1;
  blind_config.collision_monitor_release_cycles = 1;
  navigation2d::CollisionMonitor blind_monitor(blind_config);
  scan.angle_min = 0.; scan.angle_increment = .1; scan.range_min = .15;
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

  // A future branch passes closer to the slightly off-route robot than the
  // active branch. RPP must retain the local arc coordinate instead of
  // teleporting its projection to that future, westbound segment.
  navigation2d::Path self_near;
  for (int index = 0; index <= 20; ++index)
    self_near.push_back(navigation2d::MakePose2d(1. + .1 * index, 2., 0.));
  for (int index = 1; index <= 10; ++index)
    self_near.push_back(navigation2d::MakePose2d(3., 2. + .1 * index, 1.57));
  self_near.push_back(navigation2d::MakePose2d(3., 2.05, -1.57));
  for (int index = 1; index <= 20; ++index)
    self_near.push_back(navigation2d::MakePose2d(3. - .1 * index, 2.05, 3.14));
  navigation2d::RegulatedPurePursuit rpp(config);
  const auto command = rpp.Compute(
      self_near, navigation2d::MakePose2d(1.2, 2.06, 0.), {}, costmap);
  assert(command.linear > 0.);
  assert(command.angular < 0.);

  // Regression for the warehouse return stall. The base reaches a shelf
  // corner with forward velocity while the validated route turns north. The
  // desired pure rotation is safe, but applying a one-cycle acceleration
  // limit retains enough eastbound velocity to hit the shelf. This must be a
  // finite stopping -> rotate-to-path manoeuvre, not an unlabelled stage-4
  // zero command which the navigation watchdog replans forever.
  const char* corner_map_path = "/tmp/navigation2d_rpp_corner_test.json";
  std::ofstream corner_output(corner_map_path);
  corner_output << R"({"width":80,"height":80,"resolution":0.05,"cells":[)";
  for (int index = 0; index < 6400; ++index)
    corner_output << (index ? ",0" : "0");
  corner_output << "]}";
  corner_output.close();
  navigation2d::NavigationConfig corner_config;
  corner_config.map_resolution = .05;
  corner_config.robot_radius = .28;
  corner_config.inflation_radius = .32;
  corner_config.collision_horizon = .35;
  navigation2d::LayeredCostmap corner_costmap(
      navigation2d::Grid2d::Load(corner_map_path), corner_config);
  corner_costmap.MarkObstacle(1.325, 1.0);
  navigation2d::Path corner_path;
  for (int index = 0; index <= 20; ++index)
    corner_path.push_back(navigation2d::MakePose2d(1., 1. + .05 * index, 1.57));
  navigation2d::RegulatedPurePursuit corner_rpp(corner_config);
  const auto corner_pose = navigation2d::MakePose2d(1., 1., 0.);
  auto corner_command = corner_rpp.Compute(
      corner_path, corner_pose, {.24, 0.}, corner_costmap);
  auto corner_diagnostics = corner_rpp.Diagnostics();
  assert(corner_command.linear == 0.);
  assert(corner_command.angular == 0.);
  assert(corner_diagnostics.maneuver == navigation2d::ControllerManeuver::kStopping);
  assert(corner_diagnostics.intentional_stop);
  corner_command = corner_rpp.Compute(corner_path, corner_pose, {}, corner_costmap);
  assert(corner_rpp.Diagnostics().maneuver == navigation2d::ControllerManeuver::kStopping);
  corner_command = corner_rpp.Compute(corner_path, corner_pose, {}, corner_costmap);
  corner_diagnostics = corner_rpp.Diagnostics();
  assert(corner_command.linear == 0.);
  assert(corner_command.angular > 0.);
  assert(corner_diagnostics.maneuver == navigation2d::ControllerManeuver::kRotateToPath);
  const auto aligned_corner_pose = navigation2d::MakePose2d(1., 1., .5 * std::acos(-1.));
  corner_rpp.Compute(corner_path, aligned_corner_pose, {}, corner_costmap);
  corner_command = corner_rpp.Compute(corner_path, aligned_corner_pose, {}, corner_costmap);
  assert(corner_command.linear > 0.);
  assert(corner_rpp.Diagnostics().maneuver == navigation2d::ControllerManeuver::kTracking);

  // The local controller checks the physical footprint continuously.  A pose
  // whose centre is 0.275 m from the occupied cell must be rejected before
  // the base reaches contact; the raster half-diagonal is applied by the
  // global planner/path validator, not twice in RPP.
  corner_costmap.ClearObstacle(1.325, 1.0);
  corner_costmap.MarkObstacle(1.025, 1.0);
  navigation2d::RegulatedPurePursuit margin_rpp(corner_config);
  assert(margin_rpp.CollisionImminent(
      navigation2d::MakePose2d(1.350, 1.0, M_PI), {.048, 0.}, corner_costmap));
}
