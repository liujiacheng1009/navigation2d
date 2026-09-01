#include "navigation2d/application/navigation_system.h"

#include <cassert>
#include <fstream>
#include <limits>

int main() {
  const char* path = "/tmp/navigation2d_recovery_test.json";
  std::ofstream map(path);
  map << R"({"width":50,"height":30,"resolution":0.1,"cells":[)";
  for (int index = 0; index < 1500; ++index) map << (index ? ",0" : "0");
  map << "]}";
  map.close();

  navigation2d::NavigationConfig config;
  config.map_resolution = .1;
  config.planner = "astar";
  config.controller = "rpp";
  config.robot_radius = .15;
  config.progress_timeout = .12;
  config.recovery_duration = .06;
  config.control_period = .06;
  config.max_recovery_attempts = 2;
  navigation2d::NavigationSystem navigation(config, path);
  navigation.SetGoal(navigation2d::MakePose2d(4., 1.5, 0.));
  navigation2d::LaserScan scan;
  scan.angle_min = 0.; scan.angle_increment = .1;
  scan.range_min = .02; scan.range_max = 5.;
  scan.ranges.assign(60, std::numeric_limits<double>::infinity());
  const auto stuck_pose = navigation2d::MakePose2d(1., 1.5, 0.);
  navigation2d::NavigationState state;
  for (int step = 0; step < 30; ++step) {
    const double now = step * config.control_period;
    navigation.UpdateLaserScan(stuck_pose, scan);
    state = navigation.ComputeCommand(stuck_pose, {}, now);
    if (state.status == navigation2d::NavigationStatus::kBlocked) break;
  }
  assert(state.status == navigation2d::NavigationStatus::kBlocked);
  assert(state.recoveries == 3);
}
