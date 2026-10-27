#include "navigation2d/configuration/navigation_config.h"

#include <stdexcept>
#include "navigation2d/configuration/builtin_parameters.h"

#include "navigation2d/application/builtin_algorithm_catalog.h"
#include "navigation2d/configuration/navigation_validation.h"
#include "navigation2d/configuration/navigation_yaml.h"
namespace navigation2d {
NavigationConfig NavigationConfig::Load(const std::string& filename) {
  auto config = configuration_detail::ReadNavigationYaml(filename);
  if (!builtin_algorithms::HasPlanner(config.planner) ||
      !builtin_algorithms::HasController(config.controller)) {
    throw std::runtime_error("unknown planner or controller selection");
  }
  configuration_detail::ValidateNavigationBounds(config);
  ValidateBuiltinParameterSets(config);
  return config;
}
}  // namespace navigation2d
