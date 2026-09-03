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

  // A precise SE(2) goal uses the docking servo rather than turning a few
  // centimetres of residual route into another RPP/recovery cycle.
  navigation2d::NavigationConfig docking_config = config;
  docking_config.goal_xy_tolerance = .03;
  docking_config.goal_yaw_tolerance = .08;
  docking_config.progress_timeout = 1.;
  navigation2d::NavigationSystem docking(docking_config, path);
  docking.SetGoal(navigation2d::MakePose2d(1., 1., 0.));
  docking.UpdateLaserScan(navigation2d::MakePose2d(.75, 1., 0.), scan);
  state = docking.ComputeCommand(navigation2d::MakePose2d(.75, 1., 0.), {}, 0.);
  assert(state.phase == navigation2d::NavigationPhase::kDockToGoal);
  assert(state.command.linear > 0.);
  docking.UpdateLaserScan(navigation2d::MakePose2d(.99, 1., .25), scan);
  state = docking.ComputeCommand(navigation2d::MakePose2d(.99, 1., .25), {}, .06);
  assert(state.phase == navigation2d::NavigationPhase::kDockToGoal);
  assert(state.command.linear == 0.);
  assert(state.command.angular < 0.);
  state = docking.ComputeCommand(navigation2d::MakePose2d(.99, 1., .02), {}, .12);
  assert(state.status == navigation2d::NavigationStatus::kSucceeded);

  // A command that remains non-zero while the measured base stays still is
  // an execution stall (for example, a wheel/body contact), not a valid
  // controller state.  Debounce it for a short handoff window, then
  // invalidate the route so the caller can replan from the measured pose.
  navigation2d::NavigationConfig stall_config = config;
  stall_config.robot_radius = .15;
  stall_config.progress_timeout = 10.;
  stall_config.max_recovery_attempts = 3;
  navigation2d::NavigationSystem stalled(stall_config, path);
  stalled.SetGoal(navigation2d::MakePose2d(4., 1.5, 0.));
  navigation2d::NavigationState stalled_state;
  for (int step = 0; step < 20; ++step) {
    stalled.UpdateLaserScan(navigation2d::MakePose2d(1., 1.5, 0.), scan);
    stalled_state = stalled.ComputeCommand(
        navigation2d::MakePose2d(1., 1.5, 0.), {}, step * .06);
    if (stalled_state.status == navigation2d::NavigationStatus::kBlocked) break;
  }
  assert(stalled_state.status == navigation2d::NavigationStatus::kBlocked);
  assert(stalled_state.recoveries >= 1);
  assert(stalled_state.planning_failure_reason ==
         "controller command produced no measured motion");
}
