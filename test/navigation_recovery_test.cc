#include <array>
#include <cassert>
#include <cmath>
#include <string_view>

#include "navigation2d/application/navigation_system.h"
#include "navigation_recovery_fixture.h"
namespace {
using namespace navigation2d;
using navigation2d::test::RecoveryWorld;
void RecoveryBudget() {
  RecoveryWorld world;
  const auto config = world.Config();
  NavigationSystem navigation(config, world.path.string());
  navigation.SetGoal(MakePose2d(4., 1.5, 0.));
  // A cell-centre start has a straight first segment. A corner start adds an
  // initial turn and tests the no-motion alignment guard before this budget.
  const auto stuck_pose = MakePose2d(1.05, 1.55, 0.);
  NavigationState state;
  for (int step = 0; step < 30; ++step) {
    navigation.UpdateLaserScan(stuck_pose, world.Scan());
    state = navigation.ComputeCommand(stuck_pose, {}, step * config.control_period);
    if (state.status == NavigationStatus::kBlocked) {
      break;
    }
  }
  assert(state.status == NavigationStatus::kBlocked);
  assert(state.recoveries == 3);
}
void Docking() {
  RecoveryWorld world;
  auto config = world.Config();
  config.goal_xy_tolerance = .03;
  config.goal_yaw_tolerance = .08;
  config.progress_timeout = 1.;
  NavigationSystem docking(config, world.path.string());
  docking.SetGoal(MakePose2d(1., 1., 0.));
  docking.UpdateLaserScan(MakePose2d(.75, 1., 0.), world.Scan());
  auto state = docking.ComputeCommand(MakePose2d(.75, 1., 0.), {}, 0.);
  assert(state.phase == NavigationPhase::kDockToGoal);
  assert(state.command.linear > 0.);
  docking.UpdateLaserScan(MakePose2d(.99, 1., .25), world.Scan());
  state = docking.ComputeCommand(MakePose2d(.99, 1., .25), {}, .06);
  assert(state.phase == NavigationPhase::kDockToGoal);
  assert(state.command.linear == 0.);
  assert(state.command.angular < 0.);
  state = docking.ComputeCommand(MakePose2d(.99, 1., .02), {}, .12);
  assert(state.status == NavigationStatus::kSucceeded);
}
void NoMeasuredMotion() {
  RecoveryWorld world;
  auto config = world.Config();
  config.progress_timeout = 10.;
  config.max_recovery_attempts = 3;
  NavigationSystem navigation(config, world.path.string());
  navigation.SetGoal(MakePose2d(4., 1.5, 0.));
  NavigationState state;
  for (int step = 0; step < 20; ++step) {
    navigation.UpdateLaserScan(MakePose2d(1., 1.5, 0.), world.Scan());
    state = navigation.ComputeCommand(MakePose2d(1., 1.5, 0.), {}, step * .06);
    if (state.status == NavigationStatus::kBlocked) {
      break;
    }
  }
  assert(state.status == NavigationStatus::kBlocked);
  assert(state.recoveries >= 1);
  assert(state.planning_failure_reason == "controller command produced no measured motion");
}
void ProbeWindow() {
  RecoveryWorld world;
  auto config = world.Config();
  config.progress_timeout = 10.;
  config.max_recovery_attempts = 3;
  NavigationSystem navigation(config, world.path.string());
  navigation.SetGoal(MakePose2d(4., 1.5, 0.));
  navigation.UpdateLaserScan(MakePose2d(1., 1.5, 0.), world.Scan());
  auto state = navigation.ComputeCommand(MakePose2d(1., 1.5, 0.), {}, 0.);
  assert(state.status != NavigationStatus::kBlocked);
  // The planning interval is not a full second of physical execution.
  state = navigation.ComputeCommand(MakePose2d(1., 1.5, 0.), {}, 1.);
  assert(state.status != NavigationStatus::kBlocked);
}
void Alignment() {
  RecoveryWorld world;
  auto config = world.Config();
  config.progress_timeout = 10.;
  config.max_recovery_attempts = 3;
  NavigationSystem navigation(config, world.path.string());
  navigation.SetGoal(MakePose2d(4., 1.5, 0.));
  Pose2d pose = MakePose2d(1., 1.5, std::acos(-1.));
  NavigationState state;
  // A half turn needs pi/max_w plus acceleration and one second of settling.
  // The old 40 cycles (2.4 s) cannot finish that turn at 0.9 rad/s.
  const int cycles = static_cast<int>(
      std::ceil((std::acos(-1.) / config.max_angular_velocity +
                 config.max_angular_velocity / config.max_angular_acceleration + 1.) /
                config.control_period));
  for (int step = 0; step < cycles; ++step) {
    navigation.UpdateLaserScan(pose, world.Scan());
    state =
        navigation.ComputeCommand(pose, {0., state.command.angular}, step * config.control_period);
    if (state.phase == NavigationPhase::kTrackPath) {
      break;
    }
    assert(state.status == NavigationStatus::kNavigating);
    assert(std::abs(state.command.linear) < .02);
    assert(std::abs(state.command.angular) > 1e-3);
    pose = MakePose2d(1., 1.5, Yaw(pose) + state.command.angular * config.control_period);
  }
  assert(state.phase == NavigationPhase::kTrackPath);
  assert(state.status != NavigationStatus::kBlocked);
}
void SignatureReplay() {
  RecoveryWorld world;
  const auto config = world.Config();
  NavigationSystem navigation(config, world.path.string());
  navigation.SetGoal(MakePose2d(4., 1.5, 0.));
  navigation.UpdateLaserScan(MakePose2d(1., 1.5, 0.), world.Scan());
  const auto state = navigation.ComputeCommand(MakePose2d(1., 1.5, 0.), {}, 0.);
  assert(state.path_signature != 0);
  assert(state.global_path_length_m > 0.);
  navigation.BanPathSignatures({state.path_signature});
  navigation.SetGoal(MakePose2d(4., 1.5, 0.));
  navigation.UpdateLaserScan(MakePose2d(1., 1.5, 0.), world.Scan());
  const auto replay = navigation.ComputeCommand(MakePose2d(1., 1.5, 0.), {}, .06);
  assert(replay.global_path_length_m <= 0.);
  assert(replay.planning_failure_reason == "failed first-segment replay");
}
}  // namespace
int main(int argc, char** argv) {
  struct Case {
    std::string_view name;
    void (*run)();
  };
  const std::array cases{Case{"budget", RecoveryBudget},  Case{"docking", Docking},
                         Case{"stall", NoMeasuredMotion}, Case{"probe", ProbeWindow},
                         Case{"alignment", Alignment},    Case{"signature", SignatureReplay}};
  if (argc > 2) {
    return 2;
  }
  bool matched = false;
  for (const auto& scenario : cases) {
    if (argc == 1 || scenario.name == argv[1]) {
      matched = true;
      scenario.run();
    }
  }
  return matched ? 0 : 2;
}
