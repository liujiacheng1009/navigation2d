#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <new>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/application/navigation_system.h"

namespace {
// Single-threaded controlled probe. Counts scalar C++ new calls only: not
// malloc, aligned allocation, SDK-internal memory, or process peak RSS.
bool tracking = false;
std::uint64_t calls = 0, bytes = 0;
struct AllocationScope {
  AllocationScope() { tracking = true; }
  ~AllocationScope() { tracking = false; }
};
}  // namespace

void* operator new(std::size_t size) {
  if (void* storage = std::malloc(size ? size : 1)) {
    if (tracking) {
      ++calls;
      bytes += size;
    }
    return storage;
  }
  throw std::bad_alloc();
}
void operator delete(void* storage) noexcept { std::free(storage); }
void operator delete(void* storage, std::size_t) noexcept { std::free(storage); }

int main(int argc, char** argv) {
  using namespace navigation2d;
  if (argc != 3 || (std::string(argv[2]) != "normal" && std::string(argv[2]) != "stuck")) {
    throw std::invalid_argument("usage: navigation_allocation_probe MAP normal|stuck");
  }
  NavigationConfig config;
  config.map_resolution = .05;
  config.control_period = .06;
  NavigationSystem navigation(config, argv[1]);
  auto pose = MakePose2d(1., 1.5, 0.);
  navigation.SetGoal(MakePose2d(4., 1.5, 0.));
  const LaserScan scan{-3.141592653589793, 2. * 3.141592653589793 / 360., .12, 8.,
                       std::vector<double>(360, 2.)};
  Twist2d measured;
  std::uint64_t fingerprint = 1469598103934665603ULL;
  const auto hash = [&](std::uint64_t value) {
    fingerprint = (fingerprint ^ value) * 1099511628211ULL;
  };
  const bool moving = std::string(argv[2]) == "normal";
  for (int tick = 0; tick < 400; ++tick) {
    // Input allocation and fixture integration are outside the measured call.
    navigation.UpdateCollisionMonitorLaserScan(
        MakePose2d(X(pose) + .18 * std::cos(Yaw(pose)),
                   Y(pose) + .18 * std::sin(Yaw(pose)), Yaw(pose)), scan);
    const auto before_calls = calls, before_bytes = bytes;
    const auto state = [&] {
      AllocationScope scope;
      return navigation.ComputeCommand(pose, measured, tick * config.control_period);
    }();
    if (!std::isfinite(state.command.linear) || !std::isfinite(state.command.angular)) {
      throw std::runtime_error("nonfinite navigation command");
    }
    hash(std::bit_cast<std::uint64_t>(state.command.linear));
    hash(std::bit_cast<std::uint64_t>(state.command.angular));
    hash(state.path_signature);
    hash(static_cast<std::uint64_t>(state.status));
    hash(static_cast<std::uint64_t>(state.phase));
    // Keep every cycle, including first planning, replan, blocked and idle.
    std::cout << tick << ',' << calls - before_calls << ',' << bytes - before_bytes << ','
              << static_cast<int>(state.status) << ',' << static_cast<int>(state.phase) << ','
              << fingerprint << '\n';
    if (moving) {
      const double yaw = Yaw(pose), next = yaw + state.command.angular * config.control_period;
      if (std::abs(state.command.angular) < 1e-9) {
        pose = MakePose2d(X(pose) + state.command.linear * std::cos(yaw) * config.control_period,
                         Y(pose) + state.command.linear * std::sin(yaw) * config.control_period, next);
      } else {
        const double radius = state.command.linear / state.command.angular;
        pose = MakePose2d(X(pose) + radius * (std::sin(next) - std::sin(yaw)),
                         Y(pose) - radius * (std::cos(next) - std::cos(yaw)), next);
      }
      measured = state.command;
    }
  }
  std::cerr << "scalar_new_calls=" << calls << " scalar_new_bytes=" << bytes << '\n';
}
