#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include "navigation2d/control/collision_monitor.h"

namespace navigation2d::test {
// Uses only the original public API so the same workload can be linked to the
// frozen core, with its own original production headers.
inline NavigationConfig CollisionWorkloadConfig(int variant) {
  NavigationConfig c;
  if (variant & 1) {
    c.footprint = {{-.21, -.13}, {.17, -.13}, {.20, -.09}, {.20, .09}, {.17, .13}, {-.21, .13}};
  }
  if (variant & 2) {
    c.map_resolution = .07;
    c.robot_radius = .23;
    c.regulated_min_speed = .12;
    c.collision_monitor_stop_distance = .14;
    c.collision_monitor_slowdown_distance = .42;
    c.collision_monitor_slowdown_ratio = .31;
    c.collision_monitor_approach_horizon = 1.2;
    c.collision_monitor_source_timeout = .6;
    c.collision_monitor_trigger_cycles = 3;
    c.collision_monitor_release_cycles = 4;
    c.control_period = .04;
  }
  c.collision_monitor_passage_mode = variant & 4;
  c.collision_monitor_enabled = !(variant & 8);
  return c;
}

inline std::vector<CollisionMonitorResult> CollisionWorkload(CollisionMonitor& monitor) {
  std::vector<CollisionMonitorResult> results;
  const auto origin = MakePose2d(1.1, -.7, .6);
  const auto sensor =
      MakePose2d(X(origin) + .30 * std::cos(.6), Y(origin) + .30 * std::sin(.6), .6);
  LaserScan scan;
  scan.angle_min = -.3;
  scan.angle_increment = .15;
  scan.range_min = .02;
  scan.range_max = 5.;
  auto sample = [&](Twist2d command, double time) {
    results.push_back(monitor.Filter(origin, command, time));
  };
  sample({.24, .1}, 0.);  // Missing source.
  for (double range : {.7, .31, .08, .01, std::numeric_limits<double>::infinity()}) {
    scan.ranges = {range, range, range, range, range};
    monitor.UpdateLaserScan(sensor, scan);
    for (int i = 0; i < 5; ++i) {
      sample({.24, .1}, 1. + .02 * i);
    }
    sample({-.24, -.1}, 1.12);
    sample({0., .4}, 1.14);
    sample({.02, .4}, 1.16);
  }
  PointCloud2d cloud;
  cloud.range_max = 5.;
  cloud.points = {{-.35, 0.}, {0., .25}, {.4, 0.}};
  monitor.UpdatePointCloud(origin, cloud);
  for (int i = 0; i < 5; ++i) {
    sample({-.24, .1}, 2. + .02 * i);
  }
  scan.ranges = {std::numeric_limits<double>::infinity()};
  monitor.UpdateLaserScan(origin, scan);
  sample({.24, 0.}, 3.);
  for (int i = 0; i < 6; ++i) {
    monitor.UpdateDepthPoints(origin, {{.25, 0.}}, 3.02 + .02 * i);
    sample({.24, 0.}, 3.02 + .02 * i);
  }
  monitor.UpdateDepthPoints(origin, {}, 3.15);
  for (int i = 0; i < 5; ++i) {
    sample({.24, 0.}, 3.15 + .02 * i);
  }
  monitor.UpdateLaserScan(origin, scan);
  sample({.24, 0.}, 3.7);  // Depth silence clears retained points.
  sample({.24, 0.}, 4.4);  // Lidar source timeout.
  sample({.24, 0.}, 3.);   // Backwards sensor time.
  return results;
}
}  // namespace navigation2d::test
