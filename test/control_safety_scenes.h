#pragma once

#include <fstream>
#include <string>

#include "navigation2d/control/collision_monitor.h"
#include "navigation2d/costmap/layered_costmap.h"

namespace navigation2d::test {
inline NavigationConfig MonitorTestConfig(int trigger_cycles = 2, int release_cycles = 2) {
  NavigationConfig config;
  config.collision_monitor_trigger_cycles = trigger_cycles;
  config.collision_monitor_release_cycles = release_cycles;
  return config;
}
inline NavigationConfig PolygonMonitorTestConfig() {
  auto config = MonitorTestConfig(1, 1);
  config.robot_radius = .22;
  config.map_resolution = .05;
  config.footprint = {{-.145, -.147}, {.115, -.147}, {.145, -.132},
                      {.145, .132},   {.115, .147},  {-.145, .147}};
  config.regulated_min_speed = .08;
  return config;
}
inline LaserScan MonitorTestScan(double range, double maximum_range = 5.) {
  LaserScan scan;
  scan.angle_min = 0.;
  scan.angle_increment = .1;
  scan.range_min = .02;
  scan.range_max = maximum_range;
  scan.ranges = {range};
  return scan;
}
// The projection case reuses the original corridor geometry/configuration,
// without depending on another test's mutable controller or output.
inline LayeredCostmap ControlCorridorMap(const NavigationConfig& config, const std::string& name) {
  const std::string map_path = "/tmp/navigation2d_" + name + "_corridor.json";
  std::ofstream output(map_path);
  output << R"({"width":60,"height":40,"resolution":0.1,"cells":[)";
  for (int y = 0; y < 40; ++y) {
    for (int x = 0; x < 60; ++x) {
      const bool wall = x == 0 || y == 0 || x == 59 || y == 39;
      output << (x || y ? "," : "") << (wall ? 100 : 0);
    }
  }
  output << "]}";
  output.close();
  return LayeredCostmap(Grid2d::Load(map_path), config);
}
}  // namespace navigation2d::test
