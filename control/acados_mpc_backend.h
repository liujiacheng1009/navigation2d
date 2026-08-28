#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/control/local_controller.h"

namespace navigation2d {

class AcadosMpcBackend {
 public:
  explicit AcadosMpcBackend(const NavigationConfig& config);
  ~AcadosMpcBackend();
  AcadosMpcBackend(const AcadosMpcBackend&) = delete;
  AcadosMpcBackend& operator=(const AcadosMpcBackend&) = delete;

  bool available() const;
  std::optional<Twist2d> Solve(const Path& path, const Pose2d& pose, Twist2d current,
                               const std::vector<PredictedObstacle>& obstacles) const;
  ControllerDiagnostics Diagnostics() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace navigation2d
