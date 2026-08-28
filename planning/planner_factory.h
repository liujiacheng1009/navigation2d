#pragma once

#include <memory>
#include <string>

#include "navigation2d/planning/global_planner.h"
#include "navigation2d/application/navigation_config.h"

namespace navigation2d {

std::unique_ptr<GlobalPlanner> MakeGlobalPlanner(const std::string& name,
                                                 double clearance);
std::unique_ptr<GlobalPlanner> MakeGlobalPlanner(const NavigationConfig& config);

}  // namespace navigation2d
