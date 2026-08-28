#pragma once

#include "navigation2d/costmap/layered_costmap.h"
#include "navigation2d/control/obstacle_prediction.h"
#include "navigation2d/planning/global_planner.h"

namespace navigation2d {

struct Twist2d {
  double linear = 0.;
  double angular = 0.;
};

enum class ControllerBackend { kNone, kAcados, kMppi, kRpp, kDwa };
enum class ControllerSolveStatus { kNotRun, kSuccess, kUnavailable, kFailure, kDeadlineMiss, kUnsafe };

struct ControllerDiagnostics {
  ControllerBackend backend = ControllerBackend::kNone;
  ControllerSolveStatus status = ControllerSolveStatus::kNotRun;
  double solve_us = 0.;
  double solver_us = 0.;
  double kkt_residual = 0.;
  int iterations = 0;
  bool deadline_miss = false;
  int fallback_level = 0;
  double min_ttc_s = 0.;
};

struct GuidanceCandidate {
  int topology_id = 0;
  Path path;
  double age_s = 0.;
};

class LocalController {
 public:
  virtual ~LocalController() = default;
  virtual Twist2d Compute(const Path& path, const Pose2d& pose, Twist2d current,
                           const LayeredCostmap& costmap,
                           const std::vector<PredictedObstacle>& dynamic_obstacles = {}) const = 0;
  virtual bool CollisionImminent(const Pose2d& pose, Twist2d command,
                                 const LayeredCostmap& costmap) const = 0;
  virtual ControllerDiagnostics Diagnostics() const { return {}; }
  virtual void SetGuidanceCandidates(std::vector<GuidanceCandidate>) {}
};

}  // namespace navigation2d
