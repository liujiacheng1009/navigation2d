#include <bit>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

#include "navigation2d/application/navigation_system.h"
#include "navigation2d/test/algorithm_contract_fixture.h"

int main(int argc, char** argv) {
  using namespace navigation2d;
  assert(argc == 3);
  test::AlgorithmContractFixture fixture;
  static_cast<void>(fixture.Map());
  NavigationConfig config;
  config.planner = argv[1];
  config.controller = argv[2];
  config.map_resolution = .1;
  config.collision_monitor_enabled = false;
  config.smoother_enabled = false;
  NavigationSystem navigation(config, (fixture.directory.path() / "map.json").string());
  const auto goal = MakePose2d(1.85, 1.05, 0.);
  auto pose = MakePose2d(1.05, 1.05, 0.);
  navigation.SetGoal(goal);
  Twist2d measured;
  NavigationState state;
  bool moved = false;
  std::uint64_t trace = 1469598103934665603ULL;
  const auto hash = [&](std::uint64_t word) {
    for (int byte = 0; byte < 8; ++byte) {
      trace ^= (word >> (byte * 8)) & 255u;
      trace *= 1099511628211ULL;
    }
  };
  int tick = 0;
  for (; tick < 500; ++tick) {
    state = navigation.ComputeCommand(pose, measured, tick * config.control_period);
    for (double value :
         {state.command.linear, state.command.angular, X(pose), Y(pose), Yaw(pose)}) {
      hash(std::bit_cast<std::uint64_t>(value));
    }
    hash(state.path_signature);
    hash(static_cast<std::uint64_t>(state.status));
    hash(static_cast<std::uint64_t>(state.phase));
    hash(static_cast<std::uint64_t>(state.replans));
    hash(static_cast<std::uint64_t>(state.recoveries));
    assert(std::isfinite(state.command.linear) && std::isfinite(state.command.angular));
    assert(state.command.linear <= config.desired_linear_velocity + 1e-9);
    assert(state.command.linear >= -config.max_reverse_velocity - 1e-9);
    assert(std::abs(state.command.angular) <= config.max_angular_velocity + 1e-9);
    if (state.status == NavigationStatus::kSucceeded) {
      break;
    }
    if (state.status != NavigationStatus::kNavigating) {
      std::cerr << config.planner << '/' << config.controller << " blocked at " << tick << ": "
                << state.planning_failure_reason << '\n';
    }
    assert(state.status == NavigationStatus::kNavigating);
    moved = moved || std::abs(state.command.linear) > 1e-4;
    const double yaw = Yaw(pose);
    const double next_yaw = yaw + state.command.angular * config.control_period;
    if (std::abs(state.command.angular) < 1e-9) {
      pose = MakePose2d(X(pose) + state.command.linear * std::cos(yaw) * config.control_period,
                        Y(pose) + state.command.linear * std::sin(yaw) * config.control_period,
                        next_yaw);
    } else {
      const double radius = state.command.linear / state.command.angular;
      pose = MakePose2d(X(pose) + radius * (std::sin(next_yaw) - std::sin(yaw)),
                        Y(pose) - radius * (std::cos(next_yaw) - std::cos(yaw)), next_yaw);
    }
    measured = state.command;
  }
  const double error = (goal.translation() - pose.translation()).norm();
  std::cout << config.planner << '/' << config.controller << " ticks=" << tick << " error=" << error
            << " replans=" << state.replans << '\n';
  assert(moved && state.status == NavigationStatus::kSucceeded);
  assert(error <= config.goal_xy_tolerance);
  assert(state.command.linear == 0. && state.command.angular == 0.);
}
