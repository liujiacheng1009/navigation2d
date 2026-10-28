#pragma once
#include <optional>

#include "navigation2d/application/execution_watchdog.h"
#include "navigation2d/application/navigation_motion.h"
#include "navigation2d/control/local_controller.h"
namespace navigation2d {
class NavigationAlgorithms;
struct TrackingInput {
  NavigationCycleObservation robot;
  const Path& path;
  const PlannedPathMetadata& metadata;
  const LayeredCostmap& costmap;
  double path_progress_m;
  const AlgorithmBudget& budget;
};
struct TrackingCycle {
  LocalControlResult control;
  std::optional<EvaluatedMotion> motion{};
  NavigationFailure failure = NavigationFailure::kNone;
  // TTC is evaluated only after all original early watchdog exits pass.
  std::optional<double> minimum_ttc_s{};
};
// One synchronous tracking cycle. Does not commit goal/phase/failure effects
// or edit a navigation snapshot; mutable dependencies retain their own state.
TrackingCycle RunTrackingCycle(NavigationAlgorithms& algorithms, ExecutionWatchdog& watchdog,
                               CommandSafetyPipeline& safety, const TrackingInput& input);
}  // namespace navigation2d
