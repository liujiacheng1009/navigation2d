#pragma once
#include "navigation2d/configuration/navigation_config.h"
namespace navigation2d {
// Compatibility adapter for builtins whose constructors still accept the
// shared legacy config. Only the selected algorithm's declared keys override
// that config; common geometry and safety settings cannot be overridden here.
NavigationConfig ResolveBuiltinParameters(const NavigationConfig& common, const std::string& id,
                                          bool planner);
void ValidateBuiltinParameterSets(const NavigationConfig& config);
}  // namespace navigation2d
