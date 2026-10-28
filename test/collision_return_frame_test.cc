#include <bit>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "navigation2d/control/collision_monitor.h"
#include "navigation2d/test/control_safety_scenes.h"

int main() {
  using namespace navigation2d;
  auto config = test::PolygonMonitorTestConfig();
  config.collision_monitor_trigger_cycles = 1;
  config.collision_monitor_release_cycles = 1;
  std::uint64_t fingerprint = 1469598103934665603ULL;
  for (const auto count : {0u, 1u, 7u, 64u}) {
    for (const bool near : {false, true}) {
      CollisionMonitor monitor(config);
      LaserScan scan = test::MonitorTestScan(8., 8.);
      scan.ranges.assign(count, 8.);
      const auto origin = MakePose2d(1., 2., .3);
      monitor.UpdateLaserScan(origin, scan);
      for (int sample = 0; sample < 3; ++sample) {
        monitor.UpdateDepthPoints(origin, {{near ? .235 : .65, 0.}}, 1. + sample * .01);
      }
      for (int step = 0; step < 3; ++step) {
        // Change both the borrowed robot pose and the number of live lidar
        // returns. Depth keeps its world-frame observation and hold history.
        if (step == 1) {
          monitor.UpdateDepthPoints(origin, {}, 1.04);
        }
        const auto robot = MakePose2d(1. + step * .005, 2., .3 + step * .01);
        scan.ranges.assign(step == 1 ? count + 5 : count, 8.);
        monitor.UpdateLaserScan(robot, scan);
        const auto result = monitor.Filter(robot, {.24, 0.}, 1.03 + step * .02);
        assert(result.action ==
               (near ? CollisionMonitorAction::kStop : CollisionMonitorAction::kNone));
        assert(std::abs(result.command.linear - (near ? 0. : .24)) < 1e-9);
        for (const auto value : {result.command.linear, result.command.angular,
                                 result.min_distance_m, result.time_to_collision_s}) {
          fingerprint = (fingerprint ^ std::bit_cast<std::uint64_t>(value)) * 1099511628211ULL;
        }
        fingerprint = (fingerprint ^ static_cast<std::uint64_t>(result.action)) * 1099511628211ULL;
      }
    }
  }
  std::cout << fingerprint << '\n';
}
