#pragma once

#include <cstddef>
#include <optional>
#include <string>

#include "navigation2d/control/obstacle_prediction.h"
#include "navigation2d/costmap/layered_costmap.h"
#include "navigation2d/execution/algorithm_budget.h"
#include "navigation2d/geometry/path_2d.h"
#include "navigation2d/geometry/twist_2d.h"

namespace navigation2d {

enum class ControllerBackend { kNone, kAcados, kMppi, kRpp, kDwa };
enum class ControllerSolveStatus {
  kNotRun,
  kSuccess,
  kUnavailable,
  kFailure,
  kDeadlineMiss,
  kUnsafe
};
enum class ControllerManeuver {
  kTracking,
  kStopping,
  kRotateToPath,
  kRouteInfeasible,
};

struct ControllerDiagnostics {
  ControllerBackend backend = ControllerBackend::kNone;
  ControllerSolveStatus status = ControllerSolveStatus::kNotRun;
  double solve_us = 0.;
  double solver_us = 0.;
  double kkt_residual = 0.;
  int iterations = 0;
  bool deadline_miss = false;
  int fallback_level = 0;
  ControllerManeuver maneuver = ControllerManeuver::kTracking;
  // A zero command can be a deliberate part of a convergent controller
  // manoeuvre (for example braking before an in-place turn).  The navigation
  // watchdog must distinguish that from a controller which has no command.
  bool intentional_stop = false;
  double min_ttc_s = 0.;
  std::size_t path_search_evaluations = 0;
};

struct GuidanceCandidate {
  int topology_id = 0;
  Path path;
  double age_s = 0.;
};

enum class LocalControlStatus {
  kSuccess,
  kIntentionalStop,
  kInfeasible,
  kCancelled,
  kBudgetExceeded,
  kInvalidInput,
  kInvalidOutput,
  kBackendError
};
struct LocalControlRequest {
  // Path must already be validated and share the pose/costmap frame. Borrowed
  // inputs remain stable until the synchronous call returns; no ownership transfers.
  const Path& path;
  const Pose2d& pose;
  Twist2d measured_velocity;
  const LayeredCostmap& costmap;
  const std::vector<PredictedObstacle>& dynamic_obstacles;
  AlgorithmBudget budget{};
};
struct LocalControlResult {
  LocalControlStatus status = LocalControlStatus::kSuccess;
  Twist2d command;
  std::string reason;
  std::optional<ControllerDiagnostics> diagnostics{};
  // Success includes a valid zero command. IntentionalStop is an explicit
  // backend decision; Infeasible and failures must never carry executable motion.
  [[nodiscard]] bool has_command() const {
    return status == LocalControlStatus::kSuccess || status == LocalControlStatus::kIntentionalStop;
  }
};

class LocalController {
 public:
  virtual ~LocalController() = default;
  virtual Twist2d Compute(const Path& path, const Pose2d& pose, Twist2d current,
                          const LayeredCostmap& costmap,
                          const std::vector<PredictedObstacle>& dynamic_obstacles = {}) const;
  LocalControlResult ComputeWithContext(const LocalControlRequest& request) const;
  virtual bool CollisionImminent(const Pose2d& pose, Twist2d command,
                                 const LayeredCostmap& costmap) const = 0;
  virtual ControllerDiagnostics Diagnostics() const { return {}; }
  virtual void SetGuidanceCandidates(std::vector<GuidanceCandidate>) {}

 protected:
  // The public boundary rejects non-finite motion and discards cancelled or
  // late results even when a backend overrides this cooperative hook.
  virtual LocalControlResult ComputeRequest(const LocalControlRequest& request) const;
};

}  // namespace navigation2d
