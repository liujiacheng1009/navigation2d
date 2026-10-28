#pragma once

#include <memory>
#include <vector>

#include "navigation2d/control/mpc_parameters.h"
#include "navigation2d/control/local_controller.h"

namespace navigation2d {

class AcadosMpcBackend;
class MppiController;
class RegulatedPurePursuit;

// Deterministic production cascade: acados MPCC, Nav2-derived MPPI, then
// regulated pursuit or a stop. It owns no ROS dependency.
class MpcController final : public LocalController {
 public:
  explicit MpcController(const NavigationConfig& config);
  ~MpcController() override;
  Twist2d Compute(const Path& path, const Pose2d& pose, Twist2d current,
                  const LayeredCostmap& costmap,
                  const std::vector<PredictedObstacle>& dynamic_obstacles = {}) const override;
  bool CollisionImminent(const Pose2d& pose, Twist2d command,
                         const LayeredCostmap& costmap) const override;
  ControllerDiagnostics Diagnostics() const override { return diagnostics_; }
  void SetGuidanceCandidates(std::vector<GuidanceCandidate> candidates) override;

 private:
  MpcParameters config_;
  mutable std::unique_ptr<AcadosMpcBackend> acados_;
  mutable std::vector<std::unique_ptr<AcadosMpcBackend>> additional_acados_;
  mutable std::unique_ptr<MppiController> mppi_;
  mutable std::unique_ptr<RegulatedPurePursuit> rpp_;
  mutable ControllerDiagnostics diagnostics_;
  mutable std::vector<GuidanceCandidate> guidance_candidates_;
};

}  // namespace navigation2d
