#pragma once

#include <vector>

#include "navigation2d/control/dynamic_safety_parameters.h"
#include "navigation2d/control/local_controller.h"

namespace navigation2d {

// Conservative, solver-independent safety check.  It remains active when an
// optimizer fails or a legacy controller is selected.
bool DynamicCollisionImminent(const Pose2d& pose, Twist2d command,
                              const std::vector<PredictedObstacle>& obstacles,
                              const NavigationConfig& config);

bool DynamicCollisionAt(const Pose2d& pose, double time,
                        const std::vector<PredictedObstacle>& obstacles,
                        const NavigationConfig& config);
double MinimumDynamicTtc(const Pose2d& pose, Twist2d command,
                         const std::vector<PredictedObstacle>& obstacles,
                         const NavigationConfig& config);

// Typed inputs for algorithms that do not own the shared configuration.
bool DynamicCollisionImminent(const Pose2d& pose, Twist2d command,
                              const std::vector<PredictedObstacle>& obstacles,
                              const DynamicSafetyParameters& config);

bool DynamicCollisionAt(const Pose2d& pose, double time,
                        const std::vector<PredictedObstacle>& obstacles,
                        const DynamicSafetyParameters& config);
double MinimumDynamicTtc(const Pose2d& pose, Twist2d command,
                         const std::vector<PredictedObstacle>& obstacles,
                         const DynamicSafetyParameters& config);

}  // namespace navigation2d
