#pragma once
#include "navigation2d/application/exploration/forward_blocked.h"
#include "navigation2d/application/exploration/open_forward_stop.h"
#include "navigation2d/application/navigation_state.h"
namespace navigation2d::exploration {
// Immediate navigation/observation/report capabilities for one failed cycle.
// Collaborator state stays in its domain owner. No argument is retained.
class GoalFailureActions : public OpenForwardStopActions,
                           public ForwardBlockedActions,
                           public EntryImmobilityActions {
 public:
  void CancelRecoveryNavigation() override = 0;
  void SelectAfterEntry() override = 0;
  void PublishStatus(const std::string&, const std::string&) override = 0;
  void AbandonLockedEntryMouth(const std::string&) override = 0;
  bool StartEntryProbe() override = 0;
  bool EscapeOrShortRetreat(const char*, const char*) override = 0;
  virtual void RecoverStalledReturn() = 0;
  virtual void CoolImmobileRoute() = 0;
  virtual bool NavigateImmobileGoal() = 0;
  // Schedule the historical 0.4 s cooldown without publishing a status.
  virtual void DelayImmobilitySelection() = 0;
  virtual void RequestImmobilityReplay() = 0;
  virtual bool TryRescueDistantMouths(const char* reason) = 0;
  virtual const ExplorationGoal& ActiveGoal() const = 0;
  virtual double ActivePathMargin(double sample) const = 0;
  virtual bool IsCornerTipStall(double clearance_m, double front_m, double side_m) const = 0;
  virtual bool IsPinchGeometryStall(double clearance_m, double front_m, double side_m) const = 0;
  virtual void ReportExecutionFailure(const NavigationState&) = 0;
  virtual bool NoteFirstSegmentFailure(const NavigationState&) = 0;
  virtual bool TryReturnSegmentFallback() = 0;
  virtual bool FailurePoseReady() const = 0;
  // Clear route cooling and remember the failure diagnostic before crawling.
  virtual void PrepareNoMotionEntry(const NavigationState&) = 0;
  virtual void DemoteActiveGoal(const char* reason) = 0;
  virtual void BeginReturn(bool force_partial) = 0;
  virtual bool StartBreadcrumbTowardHome() = 0;
  virtual void FinishCompleting() = 0;
  virtual void Finish(bool success, const std::string& message) = 0;
};
}  // namespace navigation2d::exploration
