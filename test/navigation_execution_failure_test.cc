#include <cassert>
#include <limits>
#include <stop_token>
#include <string>

#include "navigation2d/application/navigation_system.h"
#include "navigation2d/test/navigation_plugin_fixture.h"
#include "navigation_recovery_fixture.h"
namespace {
void CheckNoMotion(bool docking) {
  using namespace navigation2d;
  test::RecoveryWorld world;
  auto config = world.Config();
  config.progress_timeout = 10.;
  config.max_recovery_attempts = 3;
  config.goal_xy_tolerance = .03;
  config.goal_yaw_tolerance = .08;
  const auto pose = docking ? MakePose2d(.75, 1., 0.) : MakePose2d(1., 1.5, 0.);
  const auto goal = docking ? MakePose2d(1., 1., 0.) : MakePose2d(4., 1.5, 0.);
  NavigationSystem nav(config, world.path.string());
  nav.SetGoal(goal);
  NavigationState state;
  for (int step = 0; step < 60; ++step) {
    nav.UpdateLaserScan(pose, world.Scan());
    state = nav.ComputeCommand(pose, {}, step * .06);
    if (state.status == NavigationStatus::kBlocked) {
      break;
    }
  }
  assert(state.status == NavigationStatus::kBlocked);
  assert(state.execution_failure ==
         (docking ? NavigationFailure::kDockingNoMotion : NavigationFailure::kControllerNoMotion));
  assert(IsNoMeasuredMotion(state.execution_failure));
  assert(state.planning_failure_reason == (docking
                                               ? "docking command produced no measured motion"
                                               : "controller command produced no measured motion"));
  std::stop_source stop;
  stop.request_stop();
  state = nav.ComputeCommand(pose, {}, 4., {stop.get_token()});
  assert(state.algorithm_stop == AlgorithmStop::kCancelled);
  assert(state.execution_failure == NavigationFailure::kNone);
  nav.SetGoal(goal);
  nav.UpdateLaserScan(pose, world.Scan());
  state = nav.ComputeCommand(pose, {}, 4.1);
  assert(state.execution_failure == NavigationFailure::kNone);
  auto invalid = pose;
  invalid.translation().x() = std::numeric_limits<double>::quiet_NaN();
  state = nav.ComputeCommand(invalid, {}, 4.2);
  assert(state.control_status == LocalControlStatus::kInvalidInput);
  assert(!IsNoMeasuredMotion(state.execution_failure));
}
}  // namespace
int main() {
  using namespace navigation2d;
  CheckNoMotion(false);
  CheckNoMotion(true);
  const auto start = MakePose2d(1., 1., 0.), goal = MakePose2d(2., 1., 0.);
  const std::string text = "controller command produced no measured motion";
  test::NavigationPluginFixture planner;
  planner.calls.plan = [&](const auto&) {
    return GlobalPlanResult{GlobalPlanStatus::kNoPath, {}, {}, text};
  };
  NavigationSystem no_path(planner.config, planner.map_path, planner.Registry());
  no_path.SetGoal(goal);
  const auto rejected = no_path.ComputeCommand(start, {}, 0.);
  assert(rejected.planning_status == GlobalPlanStatus::kNoPath);
  assert(rejected.planning_failure_reason == text &&
         !IsNoMeasuredMotion(rejected.execution_failure));
  test::NavigationPluginFixture controller;
  controller.calls.control = [&](const auto&) {
    return LocalControlResult{LocalControlStatus::kInfeasible, {}, text};
  };
  NavigationSystem failed(controller.config, controller.map_path, controller.Registry());
  failed.SetGoal(goal);
  const auto infeasible = failed.ComputeCommand(start, {}, 0.);
  assert(infeasible.control_status == LocalControlStatus::kInfeasible);
  assert(infeasible.planning_failure_reason == text &&
         !IsNoMeasuredMotion(infeasible.execution_failure));
}
