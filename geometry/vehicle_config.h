#pragma once

#include "navigation2d/exploration/frontier_explorer.h"

#include <string>

namespace navigation2d {

// Loads the robot and exploration sections of config/navigation2d.yaml.
// Missing keys, unknown keys inside those sections, and a chassis that
// is not strictly inside the planning circle are fatal. Also installs the
// process robot profile used by kPlanningRadius() and the other accessors.
void ApplyVehicleConfig(const std::string& path, FrontierExplorerConfig& config);

}  // namespace navigation2d
