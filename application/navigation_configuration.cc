#include "navigation2d/application/navigation_configuration.h"

#include "navigation2d/application/algorithm_registry.h"
#include "navigation2d/configuration/navigation_validation.h"
#include "navigation2d/configuration/navigation_yaml.h"
namespace navigation2d {
NavigationConfig LoadNavigationConfig(const std::string& filename,
                                      const AlgorithmRegistry& algorithms) {
  auto config = configuration_detail::ReadNavigationYaml(filename);
  algorithms.Validate(config);
  configuration_detail::ValidateNavigationBounds(config);
  return config;
}
}  // namespace navigation2d
