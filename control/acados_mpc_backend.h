#pragma once
#include <concepts>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "navigation2d/control/acados_parameters.h"
#include "navigation2d/control/local_controller.h"

namespace navigation2d {

class AcadosMpcBackend {
 public:
  explicit AcadosMpcBackend(const NavigationConfig& config);
  // Constrained deduction preserves the legacy Constructor({}) entry point.
  template <std::same_as<AcadosParameters> Parameters>
  explicit AcadosMpcBackend(Parameters config)
      : AcadosMpcBackend(std::move(config), ParameterTag{}) {}
  ~AcadosMpcBackend();
  AcadosMpcBackend(const AcadosMpcBackend&) = delete;
  AcadosMpcBackend& operator=(const AcadosMpcBackend&) = delete;

  bool available() const;
  std::optional<Twist2d> Solve(const Path& path, const Pose2d& pose, Twist2d current,
                               const LayeredCostmap& costmap,
                               const std::vector<PredictedObstacle>& obstacles) const;
  ControllerDiagnostics Diagnostics() const;

 private:
  struct ParameterTag {};
  explicit AcadosMpcBackend(AcadosParameters config, ParameterTag);
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace navigation2d
