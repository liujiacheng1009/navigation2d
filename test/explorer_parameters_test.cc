#include "navigation2d/ros/exploration/explorer_parameters.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/application/algorithm_registry.h"
#include "navigation2d/test/temporary_directory.h"
#include "rclcpp/rclcpp.hpp"
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("explorer parameters contract failed");
  }
}
}  // namespace
int main(int argc, char** argv) {
  using namespace navigation2d;
  using namespace navigation2d::exploration_ros;
  const std::string mode = argc > 1 ? argv[1] : "normal";
  Require(mode == "normal" || mode == "defaults" || mode == "ground_truth" ||
          mode == "invalid_pose" || mode == "invalid_timeout" || mode == "invalid_speed" ||
          mode == "unknown_algorithm" || mode == "unknown_controller" ||
          mode == "custom_registry" || mode == "scoped_algorithms" || mode == "invalid_scoped" ||
          mode == "unsupported_body");
  if (argc > 1) {
    --argc;
    ++argv;
  }
  // Set each process's environment before ROS starts any background threads.
  if (mode == "defaults") {
    unsetenv("SWEEPNAV_EXPLORATION_RESULT");
    unsetenv("SWEEPNAV_EXPLORATION_TIMEOUT");
    unsetenv("SWEEPNAV_SIMULATION_SPEED");
  } else {
    setenv("SWEEPNAV_EXPLORATION_RESULT", "/tmp/parameters/result.json", 1);
    setenv("SWEEPNAV_EXPLORATION_TIMEOUT", mode == "invalid_timeout" ? "0" : "42.5", 1);
    setenv("SWEEPNAV_SIMULATION_SPEED", mode == "invalid_speed" ? "nan" : "2.0", 1);
  }
  rclcpp::init(argc, argv);
  const bool failure_expected = mode == "invalid_pose" || mode == "invalid_timeout" ||
                                mode == "invalid_speed" || mode == "unknown_algorithm" ||
                                mode == "unknown_controller" || mode == "invalid_scoped" ||
                                mode == "unsupported_body";
  bool rejected = false;
  try {
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> overrides;
    if (mode == "invalid_pose" || mode == "ground_truth") {
      overrides.emplace_back("pose_source", mode == "invalid_pose" ? "wrong" : "ground_truth");
    }
    if (mode == "unknown_algorithm") {
      overrides.emplace_back("explore.global_planner", "unknown");
    }
    if (mode == "unknown_controller") {
      overrides.emplace_back("entry.local_controller", "unknown");
    }
    if (mode == "unsupported_body") {
      overrides.emplace_back("explore.global_planner", "state_lattice");
      overrides.emplace_back("explore.preserve_planner_heading", true);
    }
    if (mode == "custom_registry") {
      overrides.emplace_back("explore.global_planner", "custom_global");
      overrides.emplace_back("explore.local_controller", "custom_local");
    }
    test::TemporaryDirectory temporary;
    if (mode == "scoped_algorithms" || mode == "invalid_scoped") {
      const auto file = temporary.path() / "algorithms.yaml";
      std::filesystem::copy_file("/opt/sweepnav/navigation2d/config/navigation2d.yaml", file);
      std::ofstream output(file, std::ios::app);
      output << "\nlocal_controllers:\n  rpp:\n    lookahead_time: "
             << (mode == "invalid_scoped" ? "nan" : "2.5") << "\n";
      output.close();
      overrides.emplace_back("algorithm_parameters_file", file.string());
    }
    options.parameter_overrides(overrides);
    auto node = std::make_shared<rclcpp::Node>("explorer_parameters_test", options);
    int constructions = 0;
    auto algorithms = *AlgorithmRegistry::Builtins();
    if (mode == "custom_registry") {
      algorithms.RegisterPlanner("custom_global",
                                 [&](const auto&) -> std::unique_ptr<GlobalPlanner> {
                                   ++constructions;
                                   return {};
                                 });
      algorithms.RegisterController("custom_local",
                                    [&](const auto&) -> std::unique_ptr<LocalController> {
                                      ++constructions;
                                      return {};
                                    });
    }
    const auto parameters = ReadExplorerParameters(*node, algorithms);
    if (!failure_expected) {
      Require(constructions == 0);
      Require(parameters.pose_source ==
              (mode == "ground_truth" ? "ground_truth" : "localization2d"));
      const bool defaults = mode == "defaults";
      Require(parameters.max_duration_s == (defaults ? 600. : 42.5));
      Require(parameters.simulation_speed == (defaults ? 1. : 2.));
      Require(parameters.result_path ==
              (defaults ? "/tmp/exploration-result.json" : "/tmp/parameters/result.json"));
      Require(parameters.snapshot_path == (defaults ? "/tmp/exploration_snapshot.json"
                                                    : "/tmp/parameters/exploration_snapshot.json"));
      Require(parameters.navigation_profiles.explore.global ==
              (mode == "custom_registry" ? "custom_global" : "theta_star"));
      Require(parameters.navigation_profiles.explore.local ==
              (mode == "custom_registry" ? "custom_local" : "rpp"));
      Require(parameters.navigation_profiles.entry.global == "astar");
      Require(parameters.navigation_profiles.returning.global == "theta_star");
      if (mode == "scoped_algorithms") {
        const auto profile =
            exploration::BuildNavigationProfile({}, parameters.navigation_profiles.explore);
        Require(profile.local_controllers.at("rpp").at("lookahead_time") == "2.5");
      }
      const auto config = ReadExplorerConfig(*node);
      Require(config.minimum_frontier_cells == 6 && config.cover_residuals);
    }
  } catch (const std::exception& error) {
    if (!failure_expected) {
      throw;
    }
    rejected = true;
    const std::string message = error.what();
    if (mode == "invalid_pose") {
      Require(message.find("pose_source") != std::string::npos);
    }
    if (mode == "invalid_timeout") {
      Require(message.find("SWEEPNAV_EXPLORATION_TIMEOUT") != std::string::npos);
    }
    if (mode == "invalid_speed") {
      Require(message.find("SWEEPNAV_SIMULATION_SPEED") != std::string::npos);
    }
    if (mode == "unknown_algorithm") {
      Require(message.find("unknown planner") != std::string::npos);
    }
    if (mode == "unsupported_body") {
      Require(message.find("capability") != std::string::npos);
    }
    if (mode == "unknown_controller") {
      Require(message.find("unknown controller") != std::string::npos);
    }
  }
  rclcpp::shutdown();
  Require(rejected == failure_expected);
}
