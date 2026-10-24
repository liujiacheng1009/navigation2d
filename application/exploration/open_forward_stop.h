#pragma once
#include "navigation2d/application/exploration/entry_recovery_execution.h"
#include "navigation2d/application/exploration/goal_execution_watch.h"
namespace navigation2d::exploration {
struct OpenForwardStopObservation {
  double path_clearance_m, side_range_m;
};
class OpenForwardStopActions : public EntryRecoveryActions {
 public:
  virtual bool OpenForwardWallScrape(double path_clearance_m, double side_range_m) const = 0;
  virtual bool BreakWallScrapeThrash(const char* reason) = 0;
  virtual bool ReplanActiveGoalOnLatestMap(const char* reason) = 0;
  virtual void DeferViewpointSelection(double seconds, const char* message) = 0;
};
// Caller has identified an exploring, open-forward monitor STOP. A locked
// entrance never falls through to open-space replan/peel. Clock and live path
// queries retain their original lazy order; no references are stored.
void ExecuteOpenForwardStop(EntryCoordinator&, const FrontierExplorer&, GoalRecoverySession&,
                            const GoalExecutionWatch&, const GoalExecutionClock&,
                            OpenForwardStopObservation, OpenForwardStopActions&);
}  // namespace navigation2d::exploration
