#pragma once
#include <optional>

#include "navigation2d/control/local_controller.h"
#include "navigation2d/control/pursuit_parameters.h"
namespace navigation2d::pursuit_internal {
// Owns the constructive stop/rotate/resume state, including two-cycle
// hysteresis. No route/map references are retained. One control-loop writer.
class Alignment {
 public:
  explicit Alignment(AlignmentParameters parameters) : config_(parameters) {}
  void BeginCycle() {
    fallback_level_ = 0;
    maneuver_ = ControllerManeuver::kTracking;
    intentional_stop_ = false;
  }
  void NewPath() {
    mode_ = Mode::kTracking;
    recovery_heading_ = 0.;
    stopped_cycles_ = 0;
    aligned_cycles_ = 0;
  }
  // nullopt delegates to ordinary tracking; a contained zero is a real stop.
  std::optional<Twist2d> Continue(const Path&, std::size_t nearest, const Pose2d&, Twist2d,
                                  const LayeredCostmap&);
  Twist2d StartAndBrake(const Path&, std::size_t nearest, const Pose2d&, Twist2d,
                        const LayeredCostmap&);
  ControllerDiagnostics Diagnostics() const {
    ControllerDiagnostics diagnostics;
    diagnostics.backend = ControllerBackend::kRpp;
    diagnostics.status = maneuver_ == ControllerManeuver::kRouteInfeasible
                             ? ControllerSolveStatus::kUnsafe
                             : ControllerSolveStatus::kSuccess;
    diagnostics.fallback_level = fallback_level_;
    diagnostics.maneuver = maneuver_;
    diagnostics.intentional_stop = intentional_stop_;
    return diagnostics;
  }

 private:
  enum class Mode { kTracking, kStopping, kRotateToPath, kRouteInfeasible };
  bool ForwardProbeSafe(double heading, const Pose2d&, const LayeredCostmap&) const;
  std::optional<double> SelectHeading(const Path&, std::size_t nearest, const Pose2d&,
                                      const LayeredCostmap&) const;
  bool Begin(const Path&, std::size_t nearest, const Pose2d&, const LayeredCostmap&);
  Twist2d Brake(const Pose2d&, Twist2d, const LayeredCostmap&);
  AlignmentParameters config_;
  Mode mode_ = Mode::kTracking;
  int fallback_level_ = 0;
  ControllerManeuver maneuver_ = ControllerManeuver::kTracking;
  bool intentional_stop_ = false;
  double recovery_heading_ = 0.;
  int stopped_cycles_ = 0, aligned_cycles_ = 0;
};
}  // namespace navigation2d::pursuit_internal
