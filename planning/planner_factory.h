#pragma once

#include <memory>
#include <string>

#include "navigation2/planning/global_planner.h"

namespace navigation2d {

std::unique_ptr<GlobalPlanner> MakeGlobalPlanner(const std::string& name,
                                                 double clearance);

}  // namespace navigation2d
