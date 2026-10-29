#pragma once
#include "navigation2d/application/exploration/mission_lifecycle.h"
#include <cstddef>

#include "navigation2d/application/exploration/tour_order.h"
namespace navigation2d::exploration {

class TourExecution;
enum class CompletionNoticeKind {
  kEmpty,
  kThresholdCell,
  kEntryProbe,
  kUnplannable,
  kCandidatesReady
};
struct CompletionNotice {
  CompletionNoticeKind kind;
  ExplorationGoal goal;
  std::size_t raw_cells = 0, candidates = 0, tour_size = 0;
  int rounds = 0;
};
// Synchronous external actions; no node/context reference or retained callbacks.
class CompletionActions {
 public:
  virtual ~CompletionActions() = default;
  virtual void ClearCompletionRouteCooling() = 0;
  virtual bool NavigateCompletionGoal(const ExplorationGoal&) = 0;
  virtual bool BreakMappedObstacleThrash() = 0;
  virtual void LockEntryMouth(const ExplorationGoal&) = 0;
  virtual bool StartEntryProbe() = 0;
  virtual void ClearEntryMouthLock() = 0;
  virtual bool TipPinchedNow() const = 0;
  virtual bool TryRescueDistantMouths(const char* reason) = 0;
  virtual bool ShouldKeepWaitingForFrontiers() const = 0;
  virtual void BeginReturn(bool partial) = 0;
  virtual void CompletionStatus(const char* message) = 0;
  virtual void ReportCompletion(const CompletionNotice&) = 0;
  virtual void BuildTour() = 0;
  virtual void StartCompletingProbe() = 0;
  virtual void SelectNextGoal() = 0;
};
// Owns the bounded residual-probe budget and its start/finish action ordering.
// Inputs are a single event-loop snapshot: world-frame metres and yaw radians.
// Explorer, mission and tour keep their own state and are never retained here.
class CompletionExecution {
 public:
  void Start(FrontierExplorer&, MissionLifecycle&, TourExecution&, TourPose, CompletionActions&);
  void Finish(FrontierExplorer&, MissionLifecycle&, TourExecution&, CompletionActions&);
  // Partial return first gives the mission's bounded distant-mouth rescue a
  // chance; normal completion returns directly. Actions remain synchronous.
  void ReturnOrRescue(bool partial, CompletionActions&);
  [[nodiscard]] bool CanProbe() const noexcept { return rounds_ < 3; }
  [[nodiscard]] bool ShouldWait(bool has_exploring_work, int no_executable_cycles) const noexcept {
    return has_exploring_work && (CanProbe() || no_executable_cycles < 8);
  }
  void ResetBudget() noexcept { rounds_ = 0; }

 private:
  int rounds_ = 0;
};
}  // namespace navigation2d::exploration
