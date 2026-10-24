#include "navigation2d/planning/planner_factory.h"

#include "navigation2d/application/algorithm_registry.h"

namespace navigation2d {
std::unique_ptr<GlobalPlanner> MakeGlobalPlanner(const std::string& name, double clearance) {
  NavigationConfig config;
  config.planner = name;
  config.robot_radius = clearance;
  return AlgorithmRegistry::Builtins()->CreatePlanner(config);
}
std::unique_ptr<GlobalPlanner> MakeGlobalPlanner(const NavigationConfig& config) {
  return AlgorithmRegistry::Builtins()->CreatePlanner(config);
}
}  // namespace navigation2d
