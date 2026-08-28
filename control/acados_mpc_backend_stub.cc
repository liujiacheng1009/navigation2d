#include "navigation2d/control/acados_mpc_backend.h"

namespace navigation2d {

struct AcadosMpcBackend::Impl {};

AcadosMpcBackend::AcadosMpcBackend(const NavigationConfig&)
    : impl_(std::make_unique<Impl>()) {}
AcadosMpcBackend::~AcadosMpcBackend() = default;

bool AcadosMpcBackend::available() const { return false; }

std::optional<Twist2d> AcadosMpcBackend::Solve(
    const Path&, const Pose2d&, Twist2d,
    const std::vector<PredictedObstacle>&) const {
  return std::nullopt;
}

}  // namespace navigation2d
