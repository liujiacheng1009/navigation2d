#pragma once
#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d::exploration {
enum class EntryArrival {
  kBeforeProbe,
  kCrawl,
  kDriving,
  kNearProbe,
  kNavigationSucceeded,
  kSelection
};
enum class EntryNoticeKind {
  kApproaching,
  kUnavailable,
  kEntered,
  kTooNear,
  kPlanRejected,
  kProbing
};
struct EntryNotice {
  EntryNoticeKind kind;
  ExplorationGoal mouth, target;
  double robot_x = 0., robot_y = 0., range_m = 0.;
  double minimum_depth_m = 0., gap_m = 0., free_depth_m = 0.;
  int attempts = 0;
  bool easy = false;
  EntryArrival arrival = EntryArrival::kBeforeProbe;
};
// Synchronous capabilities. Implementations must not defer calls or retain
// references. The executor owns ordering; adapters own navigation and outputs.
class EntryActions {
 public:
  virtual ~EntryActions() = default;
  virtual bool EntryMouthCooling(const ExplorationGoal&) = 0;
  virtual void PrepareEntryPlan(double trail_budget_m) = 0;
  virtual bool NavigateEntry(const ExplorationGoal&) = 0;
  virtual void CoolEnteredMouth(const ExplorationGoal&, double seconds) = 0;
  virtual void QueueActiveRegionFinish() = 0;
  virtual void SelectAfterEntry() = 0;
  virtual void MarkRegionFinishPending() = 0;
  virtual void ReportEntry(const EntryNotice&) = 0;
};
}  // namespace navigation2d::exploration
