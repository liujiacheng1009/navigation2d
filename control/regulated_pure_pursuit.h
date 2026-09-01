#pragma once

#include <cstddef>

#include "navigation2d/control/local_controller.h"
#include "navigation2d/application/navigation_config.h"
#include "navigation2d/control/local_controller.h"

namespace navigation2d {

class RegulatedPurePursuit final : public LocalController {
 public:
  explicit RegulatedPurePursuit(NavigationConfig config) : config_(config) {}
  Twist2d Compute(const Path& path, const Pose2d& pose, Twist2d current,
                  const LayeredCostmap& costmap,
                  const std::vector<PredictedObstacle>& dynamic_obstacles = {}) const override;
  bool CollisionImminent(const Pose2d& pose, Twist2d command,
                         const LayeredCostmap& costmap) const override;
  ControllerDiagnostics Diagnostics() const override {
    ControllerDiagnostics diagnostics;
    diagnostics.backend = ControllerBackend::kRpp;
    diagnostics.status = maneuver_ == ControllerManeuver::kRouteInfeasible ?
        ControllerSolveStatus::kUnsafe : ControllerSolveStatus::kSuccess;
    diagnostics.fallback_level = fallback_level_;
    diagnostics.maneuver = maneuver_;
    diagnostics.intentional_stop = intentional_stop_;
    return diagnostics;
  }

 private:
  enum class PursuitMode {
    kTracking,
    kStopping,
    kRotateToPath,
    kRouteInfeasible,
  };

  NavigationConfig config_;
  // Path progress is a topological state, not an unconstrained nearest-point
  // query. Retaining it prevents a self-near path from jumping backwards.
  mutable std::size_t progress_index_ = 0;
  mutable std::size_t path_size_ = 0;
  mutable double path_start_x_ = 0., path_start_y_ = 0.;
  mutable double path_goal_x_ = 0., path_goal_y_ = 0.;
  mutable int fallback_level_ = 0;
  mutable PursuitMode mode_ = PursuitMode::kTracking;
  mutable ControllerManeuver maneuver_ = ControllerManeuver::kTracking;
  mutable bool intentional_stop_ = false;
  mutable double recovery_heading_ = 0.;
  mutable int stopped_cycles_ = 0;
  mutable int aligned_cycles_ = 0;
};

}  // namespace navigation2d
