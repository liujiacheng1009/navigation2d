#pragma once
#include <optional>

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
  // Unknown observation cells already opened and reachable in planning.
  // Read lazily after discarding the failed route; absence of pose means zero.
  virtual std::size_t ReachableUnknownForRecovery() const = 0;
  virtual std::optional<TourPose> TourRobot() const = 0;
  virtual void LockEntryMouth(const ExplorationGoal&) = 0;
  virtual void ReportMappedRejectThrash(std::size_t local_unknown) = 0;
  virtual void DelayMappedRejectSelection() = 0;
  virtual void StopSelectionMotion() = 0;
  virtual void ReportPinchRecovery(const char* reason) = 0;
  // Clear published paths before the displayed target; navigation is cancelled.
  virtual void ClearRecoveryDisplay() = 0;
  virtual void RecoverStalledReturn() = 0;
  virtual void CoolActiveRecoveryRoute() = 0;
  virtual bool NavigateImmobileGoal() = 0;
  // Schedule the historical 0.4 s cooldown without publishing a status.
  virtual void DelayRecoverySelection() = 0;
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
