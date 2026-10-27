#pragma once
#include <cstddef>
#include <string>

#include "navigation2d/application/exploration/tour_order.h"
namespace navigation2d {
class FrontierExplorer;
}
namespace navigation2d::exploration {
class EntryCoordinator;
class TourExecution;
class RetreatExecution;
class RouteFailureHistory;
// One synchronous map-frame observation (metres); only diagnostics consume
// path progress/length. The caller owns the stall/time classification.
struct AlcoveObservation {
  TourPose robot;
  double goal_distance_m, front_range_m, side_range_m;
  double path_progress_m, path_length_m;
};
class AlcoveExitActions {
 public:
  virtual ~AlcoveExitActions() = default;
  virtual void ReportUnfinishedPocket(std::size_t unknown_cells, bool frontier_open) = 0;
  virtual void ReportExhaustedPocket(const AlcoveObservation&) = 0;
  virtual void CancelRecoveryNavigation() = 0;
  virtual bool PassableMouthGoal(const ExplorationGoal&) const = 0;
  virtual void LockEntryMouth(const ExplorationGoal&) = 0;
  virtual void PrepareDoorjambRetry() = 0;
  virtual bool StartEntryProbe() = 0;
  virtual bool EscapeOrShortRetreat(const char*, const char*) = 0;
  virtual void SelectAfterEntry() = 0;
  virtual void PublishStatus(const std::string&, const std::string&) = 0;
};
// A stalled alcove keeps unfinished local work nominatable. Only the legacy
// exhausted-pocket branch cools all three anchors and abandons the current tour.
// Owners are borrowed for this call; no mutable shared context is retained.
void ExecuteAlcoveExit(EntryCoordinator&, FrontierExplorer&, TourExecution&, RetreatExecution&,
                       RouteFailureHistory&, const ExplorationGoal&, const AlcoveObservation&,
                       AlcoveExitActions&);
}  // namespace navigation2d::exploration
