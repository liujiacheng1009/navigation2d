#pragma once
#include "navigation2d/control/local_controller.h"
#include "navigation2d/control/pursuit_alignment.h"
#include "navigation2d/control/pursuit_parameters.h"
#include "navigation2d/control/pursuit_path_progress.h"
namespace navigation2d {
struct NavigationConfig;
class RegulatedPurePursuit final : public LocalController {
 public:
  explicit RegulatedPurePursuit(NavigationConfig config);
  Twist2d Compute(const Path&, const Pose2d&, Twist2d current, const LayeredCostmap&,
                  const std::vector<PredictedObstacle>& dynamic_obstacles = {}) const override;
  bool CollisionImminent(const Pose2d&, Twist2d, const LayeredCostmap&) const override;
  ControllerDiagnostics Diagnostics() const override { return alignment_.Diagnostics(); }

 private:
  struct ParameterTag {};
  explicit RegulatedPurePursuit(pursuit_internal::Parameters, ParameterTag);
  pursuit_internal::TrackingParameters config_;
  mutable pursuit_internal::PathProgress progress_;
  mutable pursuit_internal::Alignment alignment_;
};
}  // namespace navigation2d
