#pragma once
#include <cstddef>

#include "navigation2d/application/exploration/tour_order.h"
namespace navigation2d::exploration {
class CompletionExecution;
class TourExecution;
struct ExhaustedSelection {
  TourPose robot;
  int attempted_plans;
};
// Synchronous effects at the exhausted-selection boundary; no references are
// retained. The mission's event loop owns calls and its clock.
class SelectionExhaustionActions {
 public:
  virtual ~SelectionExhaustionActions() = default;
  virtual void StopSelectionMotion() = 0;
  virtual void NoteKnownProgress(std::size_t) = 0;
  virtual void QueueActiveRegionFinish() = 0;
  virtual void DeferBlockedSelection(const char* message) = 0;
  virtual void SelectionStatus(const char* message) = 0;
  virtual void ClearBlockedSelection() = 0;
  virtual bool SelectionFrontSwingClear() const = 0;
  virtual bool TipPinchedNow() const = 0;
  virtual bool TryRescueDistantMouths(const char* reason) = 0;
  virtual bool ShouldKeepWaitingForFrontiers() const = 0;
  virtual void StartCompletingProbe() = 0;
  virtual void BeginReturnOrRescue(bool partial) = 0;
  virtual void BeginReturn(bool partial) = 0;
};
// Owns only evidence-growth and empty/no-executable selection budgets. Entry,
// tour, residual-probe and mission state remain with their existing owners.
class SelectionExhaustion {
 public:
  void InitializeKnown(std::size_t known) { last_known_cells_ = known; }
  void GoalAccepted() { empty_cycles_ = 0; }
  void ResetBlocked() { blocked_cycles_ = 0; }
  int blocked_cycles() const { return blocked_cycles_; }
  void Resolve(FrontierExplorer&, CompletionExecution&, TourExecution&, ExhaustedSelection,
               SelectionExhaustionActions&);

 private:
  void ResolveBlocked(FrontierExplorer&, CompletionExecution&, TourExecution&, ExhaustedSelection,
                      SelectionExhaustionActions&);
  void WaitForMap(FrontierExplorer&, CompletionExecution&, int attempted_plans,
                  SelectionExhaustionActions&);
  int empty_cycles_ = 0, blocked_cycles_ = 0;
  std::size_t last_known_cells_ = 0;
};
}  // namespace navigation2d::exploration
