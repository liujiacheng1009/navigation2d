#pragma once
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>

// Stable public include also binds the independently frozen pre-refactor core.
#include "navigation2d/application/navigation_config.h"
#include "navigation2d/sensor/observations.h"
namespace navigation2d::test {
// Each CTest scenario owns a separate map, including parallel processes.
class RecoveryWorld {
 public:
  RecoveryWorld()
      : path(std::filesystem::temp_directory_path() /
             ("navigation2d_recovery_" + std::to_string(getpid()) + ".json")) {
    std::ofstream map(path);
    map << R"({"width":50,"height":30,"resolution":0.1,"cells":[)";
    for (int index = 0; index < 1500; ++index) {
      map << (index ? ",0" : "0");
    }
    map << "]}";
    map.close();
    if (!map) {
      throw std::runtime_error("cannot write recovery test map");
    }
  }
  ~RecoveryWorld() {
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
  }
  RecoveryWorld(const RecoveryWorld&) = delete;
  RecoveryWorld& operator=(const RecoveryWorld&) = delete;
  static NavigationConfig Config() {
    NavigationConfig config;
    config.map_resolution = .1;
    config.planner = "astar";
    config.controller = "rpp";
    config.robot_radius = .15;
    config.progress_timeout = .12;
    config.recovery_duration = .06;
    config.control_period = .06;
    config.max_recovery_attempts = 2;
    return config;
  }
  static LaserScan Scan() {
    LaserScan scan{0., .1, .02, 5., {}};
    scan.ranges.assign(60, std::numeric_limits<double>::infinity());
    return scan;
  }
  const std::filesystem::path path;
};
}  // namespace navigation2d::test
