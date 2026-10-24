#pragma once
#include <string>

#include "navigation2d/application/exploration/navigation_profile.h"
#include "navigation2d/exploration/exploration_config.h"
namespace rclcpp {
class Node;
}
namespace navigation2d {
class AlgorithmRegistry;
}
namespace navigation2d::exploration_ros {
struct ExplorerParameters {
  std::string pose_source;
  std::string result_path;
  std::string snapshot_path;
  double max_duration_s = 600.;
  double simulation_speed = 1.;
  exploration::NavigationProfiles navigation_profiles;
};
FrontierExplorerConfig ReadExplorerConfig(rclcpp::Node&);
// Startup only. Validate selections against explicit assembly dependencies;
// reading configuration never constructs a planner or controller.
ExplorerParameters ReadExplorerParameters(rclcpp::Node&, const AlgorithmRegistry&);
}  // namespace navigation2d::exploration_ros
