#pragma once
#include <optional>
#include <utility>

#include "navigation2d/application/exploration/trail_navigation.h"
namespace navigation2d::exploration {
class RetreatActions {
 public:
  virtual ~RetreatActions() = default;
  virtual bool RetreatPinched() const = 0;
  virtual void ClearRetreatCooling() = 0;
  virtual void PrepareRetreatEvidence(bool pinched) = 0;
  virtual bool NavigateRetreat(const ExplorationGoal&) = 0;
  virtual void CancelRetreatEvidence() = 0;
};
// Owns the active retreat purpose and the attempted-cell latch. Resetting the
// retry latch does not finish an active goal; navigation completion does not
// silently allow another retreat from the same cell.
class RetreatExecution {
 public:
  bool Start(DrivenTrail, TourPose, std::pair<int, int> cell, bool entry, RetreatActions&);
  [[nodiscard]] bool active() const { return active_; }
  void Finish() { active_ = false; }
  void AllowRetry() { attempted_cell_.reset(); }

 private:
  bool active_ = false;
  std::optional<std::pair<int, int>> attempted_cell_;
};
}  // namespace navigation2d::exploration
