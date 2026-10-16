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
  // Clearance is 0.29 - 0.18 = 0.11 m, just outside the 0.10 m stop distance.
  // The monitor must slow this command, not zero it.
  assert(result.action == navigation2d::CollisionMonitorAction::kSlowdown);
  assert(result.command.linear >= .05);

  scan.ranges = {std::numeric_limits<double>::infinity()};
  monitor.UpdateLaserScan(pose, scan); monitor.Filter(pose, {.2, 0.}, 1.10);
  monitor.UpdateLaserScan(pose, scan); result = monitor.Filter(pose, {.2, 0.}, 1.15);
  assert(result.action == navigation2d::CollisionMonitorAction::kNone);
  result = monitor.Filter(pose, {.2, 0.}, 2.);
  assert(result.action == navigation2d::CollisionMonitorAction::kSourceTimeout);

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
                                  {.145, .132}, {.115, .147}, {-.145, .147}};
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
  // Shelf-corner stop→rotate regression uses the historical radii that made
  // the eastbound accel clamp collide; CAD doorway cases below use 0.22 m.
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
  // Centre 0.275 m from the marked cell (comment contract); with r=0.28 the
  // projected footprint is already in contact.
  assert(margin_rpp.CollisionImminent(
      navigation2d::MakePose2d(1.300, 1.0, M_PI), {.048, 0.}, corner_costmap));

  // Doorway regression: parallel walls leave ~0.12 m margin outside the
  // 0.22 m footprint while the forward corridor is free. Old inflation-cost
  // regulation crawled to ≤0.25×; clearance+trust must keep a healthy speed.
  const char* door_map_path = "/tmp/navigation2d_rpp_doorway_test.json";
  {
    std::ofstream door_output(door_map_path);
    door_output << R"({"width":80,"height":40,"resolution":0.05,"cells":[)";
    for (int y = 0; y < 40; ++y) {
      for (int x = 0; x < 80; ++x) {
        // Free corridor |y| < 0.34 m around y=1.0 → walls at ±0.34 m from
        // centreline. Robot at (1,1) has clearance 0.34 m, margin 0.12 m.
        const double wy = (y + .5) * .05;
        const bool wall = wy < .66 || wy > 1.34;
        door_output << ((x || y) ? "," : "") << (wall ? 254 : 0);
      }
    }
    door_output << "]}";
  }
  navigation2d::NavigationConfig door_config;
  door_config.map_resolution = .05;
  door_config.robot_radius = .22;
  door_config.inflation_radius = .30;
  door_config.desired_linear_velocity = .24;
  door_config.collision_horizon = .35;
  door_config.regulated_min_speed = .08;
  navigation2d::LayeredCostmap door_costmap(
      navigation2d::Grid2d::Load(door_map_path), door_config);
  navigation2d::Path door_path;
  for (int index = 0; index <= 30; ++index)
    door_path.push_back(navigation2d::MakePose2d(1. + .05 * index, 1., 0.));
  navigation2d::RegulatedPurePursuit door_rpp(door_config);
  // Warm up path projection with zero speed, then command from rest so the
  // acceleration clamp does not hide the clearance scale result.
  door_rpp.Compute(door_path, navigation2d::MakePose2d(1., 1., 0.), {}, door_costmap);
  auto door_cmd = door_rpp.Compute(
      door_path, navigation2d::MakePose2d(1., 1., 0.), {}, door_costmap);
  // Several cycles to finish accel ramp toward desired speed.
  for (int i = 0; i < 20; ++i) {
    door_cmd = door_rpp.Compute(
        door_path, navigation2d::MakePose2d(1., 1., 0.), door_cmd, door_costmap);
  }
  assert(door_cmd.linear > .15);
  assert(door_rpp.Diagnostics().maneuver == navigation2d::ControllerManeuver::kTracking);

  // Tight sides + blocked forward → trust off, soft band applies; still above
  // crawl floor, but clearly slower than the clear doorway case.
  const char* pinch_map_path = "/tmp/navigation2d_rpp_pinch_test.json";
  {
    std::ofstream pinch_output(pinch_map_path);
    pinch_output << R"({"width":80,"height":40,"resolution":0.05,"cells":[)";
    for (int y = 0; y < 40; ++y) {
      for (int x = 0; x < 80; ++x) {
        const double wx = (x + .5) * .05;
        const double wy = (y + .5) * .05;
        const bool wall = wy < .75 || wy > 1.25 || wx > 1.45;
        pinch_output << ((x || y) ? "," : "") << (wall ? 254 : 0);
      }
    }
    pinch_output << "]}";
  }
  navigation2d::LayeredCostmap pinch_costmap(
      navigation2d::Grid2d::Load(pinch_map_path), door_config);
  navigation2d::Path pinch_path;
  for (int index = 0; index <= 8; ++index)
    pinch_path.push_back(navigation2d::MakePose2d(1. + .05 * index, 1., 0.));
  navigation2d::RegulatedPurePursuit pinch_rpp(door_config);
  navigation2d::Twist2d pinch_cmd{};
  for (int i = 0; i < 25; ++i) {
    pinch_cmd = pinch_rpp.Compute(
        pinch_path, navigation2d::MakePose2d(1., 1., 0.), pinch_cmd, pinch_costmap);
  }
  assert(pinch_cmd.linear >= door_config.regulated_min_speed - 1e-9);
  assert(pinch_cmd.linear + 1e-9 < door_cmd.linear);

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

  for (int sample = 0; sample < 3; ++sample)
    depth_monitor.UpdateDepthPoints(depth_origin, {{.65, 0.}}, 7.05 + .01 * sample);
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

  for (int sample = 0; sample < 3; ++sample)
    depth_stop.UpdateDepthPoints(depth_origin, {{stop_point, 0.}}, 7.30 + .01 * sample);
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
  for (int sample = 0; sample < 3; ++sample)
    depth_timeout.UpdateDepthPoints(depth_origin, {{stop_point, 0.}}, 8. + .01 * sample);
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
