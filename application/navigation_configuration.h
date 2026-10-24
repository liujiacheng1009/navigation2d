#pragma once
#include <string>

#include "navigation2d/configuration/navigation_config.h"
namespace navigation2d {
class AlgorithmRegistry;
// Startup assembly: validate algorithm IDs against the supplied registry,
// without constructing planners/controllers or introducing a global registry.
[[nodiscard]] NavigationConfig LoadNavigationConfig(const std::string&, const AlgorithmRegistry&);
}  // namespace navigation2d
