#include "navigation2d/ros/exploration/explorer_parameters.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>

#include "navigation2d/application/algorithm_registry.h"
#include "navigation2d/application/navigation_configuration.h"
#include "navigation2d/configuration/vehicle_config.h"
#include "rclcpp/rclcpp.hpp"
namespace navigation2d::exploration_ros {
namespace {
double RequiredPositiveEnvironment(const char* name, double fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  const double parsed = std::stod(value);
  if (!std::isfinite(parsed) || parsed <= 0.) {
    throw std::runtime_error(std::string(name) + " must be positive");
  }
  return parsed;
}

void ReadSelection(rclcpp::Node& node, const std::string& prefix,
                   exploration::AlgorithmSelection& selection, const AlgorithmRegistry& algorithms,
                   const NavigationConfig& scoped) {
  selection.global =
      node.declare_parameter<std::string>(prefix + ".global_planner", selection.global);
  selection.local =
      node.declare_parameter<std::string>(prefix + ".local_controller", selection.local);
  selection.heading_policy =
      node.declare_parameter<bool>(prefix + ".preserve_planner_heading", false)
          ? RouteHeadingPolicy::kPreservePlannerHeading
          : RouteHeadingPolicy::kLegacyForwardProjection;
  selection.global_parameters = scoped.global_planners;
  selection.local_parameters = scoped.local_controllers;
  const auto validation = exploration::BuildNavigationProfile({.resolution = scoped.map_resolution,
                                                               .entry = prefix == "entry",
                                                               .returning = prefix == "return"},
                                                              selection);
  algorithms.Validate(validation);
  if (selection.heading_policy == RouteHeadingPolicy::kPreservePlannerHeading &&
      algorithms.PlannerTraits(selection.global).reverse_paths) {
    algorithms.Validate(validation, {.reverse_paths = true});
  }
}
}  // namespace
FrontierExplorerConfig ReadExplorerConfig(rclcpp::Node& node) {
  navigation2d::FrontierExplorerConfig config;
  // Maze doorways often produce short but useful frontier components.  A
  // larger threshold silently drops those branches before they reach the
  // planner; noise is handled by the stability observer below.
  config.minimum_frontier_cells = 6;
  // Metre distances and the chassis outline come from config/navigation2d.yaml.
  const std::string vehicle_config = node.declare_parameter<std::string>(
      "vehicle_config", "/opt/sweepnav/navigation2d/config/navigation2d.yaml");
  navigation2d::ApplyVehicleConfig(vehicle_config, config);
  config.cover_residuals = true;
  return config;
}
ExplorerParameters ReadExplorerParameters(rclcpp::Node& node, const AlgorithmRegistry& algorithms) {
  ExplorerParameters parameters;
  parameters.pose_source = node.declare_parameter<std::string>("pose_source", "localization2d");
  if (parameters.pose_source != "ground_truth" && parameters.pose_source != "localization2d") {
    throw std::runtime_error("pose_source must be 'ground_truth' or 'localization2d'");
  }
  parameters.result_path = std::getenv("SWEEPNAV_EXPLORATION_RESULT") != nullptr
                               ? std::getenv("SWEEPNAV_EXPLORATION_RESULT")
                               : "/tmp/exploration-result.json";
  parameters.snapshot_path =
      (std::filesystem::path(parameters.result_path).parent_path() / "exploration_snapshot.json")
          .string();
  parameters.max_duration_s = RequiredPositiveEnvironment("SWEEPNAV_EXPLORATION_TIMEOUT", 600.);
  parameters.simulation_speed = RequiredPositiveEnvironment("SWEEPNAV_SIMULATION_SPEED", 1.);
  const auto parameter_file = node.declare_parameter<std::string>("algorithm_parameters_file", "");
  const auto scoped = parameter_file.empty() ? NavigationConfig{}
                                             : LoadNavigationConfig(parameter_file, algorithms);
  ReadSelection(node, "explore", parameters.navigation_profiles.explore, algorithms, scoped);
  ReadSelection(node, "entry", parameters.navigation_profiles.entry, algorithms, scoped);
  ReadSelection(node, "return", parameters.navigation_profiles.returning, algorithms, scoped);
  return parameters;
}
}  // namespace navigation2d::exploration_ros
