#pragma once
#include <cstdint>
#include <optional>
#include <string>

#include "navigation2d/application/exploration/entry_selection.h"
#include "navigation2d/application/exploration/selection_exhaustion.h"
#include "navigation2d/application/exploration/viewpoint_rejection_executor.h"
namespace navigation2d::exploration {
enum class SelectionSkip { kResolved, kDemoted };
struct ViewpointPlanFailure {
  bool start_occupied;
  const std::string& reason;  // Borrow until the next navigation attempt.
  std::uint64_t path_signature;
};
// One synchronous selection transaction. Existing entry, exhaustion and
// rejection capabilities keep their contracts. No action retains arguments.
class ViewpointSelectionActions : public EntrySelectionActions,
                                  public SelectionExhaustionActions,
                                  public ViewpointRejectionActions,
                                  public TourConstraints {
 public:
  virtual void BuildTour() = 0;
  virtual std::optional<TourPose> TourRobot() const = 0;
  virtual void ReportSelectionRescue(const char* reason, bool no_distant_mouth) = 0;
  // Cancel the route, clear its published paths/goal, then clear recovery.
  virtual void DiscardRescueNavigation() = 0;
  // Publish RECOVERING first, then schedule selection after the 0.6 s hold.
  virtual void DeferRescueRecovery(const char* reason) = 0;
  virtual void SelectNextGoal() = 0;
  virtual bool HasRescueNavigation() const = 0;
  virtual bool NavigateViewpoint(const ExplorationGoal&) = 0;
  virtual ViewpointPlanFailure LastViewpointFailure() const = 0;
  virtual void ViewpointAccepted() = 0;
  virtual void PreparePinchedViewpoint(const std::string& reason) = 0;
  virtual bool EscapeOrShortRetreat(const char* escape, const char* retreat) = 0;
  virtual void ClearSelectionRouteCooling() = 0;
  virtual void LockEntryMouth(const ExplorationGoal&) = 0;
  virtual void DeferViewpointRetry() = 0;
  virtual void ReportSelectionSkip(SelectionSkip, const ExplorationGoal&) = 0;
};
}  // namespace navigation2d::exploration
