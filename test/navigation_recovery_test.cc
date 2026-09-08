#include "navigation2d/application/navigation_system.h"

#include <cassert>
#include <cmath>
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

  // A long first ComputeCommand is planning, not execution.  Two samples
  // whose timestamps are 1 s apart must not count as a 0.9 s stall.
  navigation2d::NavigationSystem probe_stall(stall_config, path);
  probe_stall.SetGoal(navigation2d::MakePose2d(4., 1.5, 0.));
  probe_stall.UpdateLaserScan(navigation2d::MakePose2d(1., 1.5, 0.), scan);
  auto probe_state = probe_stall.ComputeCommand(
      navigation2d::MakePose2d(1., 1.5, 0.), {}, 0.);
  assert(probe_state.status != navigation2d::NavigationStatus::kBlocked);
  probe_state = probe_stall.ComputeCommand(
      navigation2d::MakePose2d(1., 1.5, 0.), {}, 1.);
  assert(probe_state.status != navigation2d::NavigationStatus::kBlocked);

  // A path that starts behind the robot must rotate in place.  A coupled
  // crawl+turn command is a failed translation even when yaw is changing.
  navigation2d::NavigationConfig align_config = config;
  align_config.progress_timeout = 10.;
  align_config.max_recovery_attempts = 3;
  navigation2d::NavigationSystem aligning(align_config, path);
  aligning.SetGoal(navigation2d::MakePose2d(4., 1.5, 0.));
  navigation2d::Pose2d align_pose = navigation2d::MakePose2d(1., 1.5, std::acos(-1.));
  navigation2d::NavigationState align_state;
  for (int step = 0; step < 40; ++step) {
    aligning.UpdateLaserScan(align_pose, scan);
    align_state = aligning.ComputeCommand(align_pose, {0., align_state.command.angular},
                                          step * .06);
    if (align_state.phase == navigation2d::NavigationPhase::kTrackPath) break;
    assert(align_state.status == navigation2d::NavigationStatus::kNavigating);
    assert(std::abs(align_state.command.linear) < .02);
    assert(std::abs(align_state.command.angular) > 1e-3);
    const double yaw = navigation2d::Yaw(align_pose) + align_state.command.angular * .06;
    align_pose = navigation2d::MakePose2d(1., 1.5, yaw);
  }
  assert(align_state.phase == navigation2d::NavigationPhase::kTrackPath);
  assert(align_state.status != navigation2d::NavigationStatus::kBlocked);

  // A cooled first-segment signature must not be accepted again from the
  // same start pose.  A* is deterministic here, so the only legal outcome
  // is an explicit replay rejection rather than another identical prefix.
  navigation2d::NavigationSystem signed_route(config, path);
  signed_route.SetGoal(navigation2d::MakePose2d(4., 1.5, 0.));
  signed_route.UpdateLaserScan(navigation2d::MakePose2d(1., 1.5, 0.), scan);
  const auto signed_state = signed_route.ComputeCommand(
      navigation2d::MakePose2d(1., 1.5, 0.), {}, 0.);
  assert(signed_state.path_signature != 0);
  assert(signed_state.global_path_length_m > 0.);
  signed_route.BanPathSignatures({signed_state.path_signature});
  signed_route.SetGoal(navigation2d::MakePose2d(4., 1.5, 0.));
  signed_route.UpdateLaserScan(navigation2d::MakePose2d(1., 1.5, 0.), scan);
  const auto replay = signed_route.ComputeCommand(
      navigation2d::MakePose2d(1., 1.5, 0.), {}, .06);
  assert(replay.global_path_length_m <= 0.);
  assert(replay.planning_failure_reason == "failed first-segment replay");
}
