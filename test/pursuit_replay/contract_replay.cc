#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/control/regulated_pure_pursuit.h"
#include "navigation2d/test/pursuit_replay/worlds.h"
namespace {
double Percentile(std::vector<double> samples, double q) {
  std::sort(samples.begin(), samples.end());
  const auto low = static_cast<std::size_t>(q * (samples.size() - 1));
  const auto high = std::min(low + 1, samples.size() - 1);
  const double fraction = q * (samples.size() - 1) - low;
  return samples[low] + fraction * (samples[high] - samples[low]);
}
}  // namespace
int main(int argc, char** argv) {
  using namespace navigation2d;
  if (argc < 2 || argc > 3) {
    throw std::runtime_error("usage: pursuit_replay CONFIG [--benchmark]");
  }
  const bool benchmark = argc == 3 && std::string(argv[2]) == "--benchmark";
  if (argc == 3 && !benchmark) {
    throw std::runtime_error("unsupported pursuit replay mode");
  }
  const auto begin = std::chrono::steady_clock::now();
  const auto loaded = NavigationConfig::Load(argv[1]);
  test::PursuitWorlds worlds;
  const auto paths = test::PursuitPaths();
  std::vector<double> elapsed;
  std::ostringstream trace;
  trace << std::setprecision(17) << '[';
  bool first = true;
  for (int profile = 0; profile < 2; ++profile) {
    auto config = loaded;
    config.map_resolution = .05;
    config.robot_radius = profile ? .255 : loaded.robot_radius;
    for (int map = 0; map < 4; ++map) {
      const LayeredCostmap costmap(worlds.Load(map), config);
      for (std::size_t task = 0; task < paths.size(); ++task) {
        RegulatedPurePursuit controller(config);
        for (int step = 0; step < 20; ++step) {
          auto route = paths[task];
          if (step == 7) {
            route.clear();
          } else if (step >= 14 && !route.empty()) {
            const auto goal = route.back();
            route.back() = MakePose2d(X(goal) + (step < 17 ? .049 : .051), Y(goal), Yaw(goal));
          }
          const auto pose = MakePose2d(1.1 + .02 * (step % 10), 1.3 + .02 * (step % 3),
                                       step < 5    ? .6
                                       : step < 10 ? .3
                                                   : 0.);
          const Twist2d measured = step < 6 ? Twist2d{.2, .1} : Twist2d{};
          const auto tick = std::chrono::steady_clock::now();
          const auto command = controller.Compute(route, pose, measured, costmap);
          elapsed.push_back(
              std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - tick)
                  .count());
          const auto diagnostics = controller.Diagnostics();
          if (!first) {
            trace << ',';
          }
          first = false;
          trace << "{\"profile\":" << profile << ",\"map\":" << map << ",\"task\":" << task
                << ",\"step\":" << step << ",\"vx\":" << command.linear
                << ",\"w\":" << command.angular
                << ",\"backend\":" << static_cast<int>(diagnostics.backend)
                << ",\"status\":" << static_cast<int>(diagnostics.status)
                << ",\"fallback\":" << diagnostics.fallback_level
                << ",\"maneuver\":" << static_cast<int>(diagnostics.maneuver)
                << ",\"intentional_stop\":" << diagnostics.intentional_stop
                << ",\"collision\":" << controller.CollisionImminent(pose, command, costmap) << '}';
        }
      }
    }
  }
  trace << ']';
  std::cout << std::setprecision(17);
  if (benchmark) {
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    std::cout << "{\"trace\":" << trace.str() << ",\"tick_p50_us\":" << Percentile(elapsed, .50)
              << ",\"tick_p95_us\":" << Percentile(elapsed, .95)
              << ",\"tick_p99_us\":" << Percentile(elapsed, .99)
              << ",\"peak_rss_kb\":" << usage.ru_maxrss << ",\"wall_seconds\":"
              << std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count()
              << "}\n";
  } else {
    std::cout << trace.str() << '\n';
  }
}
