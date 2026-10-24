#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <utility>

#include "navigation2d/application/exploration/tour_order.h"
namespace navigation2d::exploration {
// Owns the committed task sequence and its revision. Consumers borrow a const
// view; all cursor transitions and replacement policies remain here.
class TourExecution {
 public:
  void Build(FrontierExplorer&, TourPose);
  void QueueActiveRoom(FrontierExplorer&, TourPose, TourConstraints&);
  void AfterMouthFailure(FrontierExplorer&, const ExplorationGoal&, std::optional<TourPose>,
                         TourConstraints&);
  void AfterWallTip(FrontierExplorer&, const ExplorationGoal&, std::optional<TourPose>,
                    TourConstraints&);
  bool PrioritizeDistant(FrontierExplorer&, TourPose, TourConstraints&);
  void SkipResolved(FrontierExplorer&);
  // Advance/Defer require a current task, just as a tracked handoff does.
  void Advance() { goals_.pop_front(); }
  void DeferCurrent(ExplorationGoal goal) {
    goals_.pop_front();
    goals_.push_back(std::move(goal));
  }
  void RetryLater(const ExplorationGoal& goal) { goals_.push_back(goal); }
  void HandoffThrough(std::size_t count);
  void Suspend() { goals_.clear(); }
  const std::deque<ExplorationGoal>& goals() const { return goals_; }
  std::uint64_t revision() const { return revision_; }

 private:
  void Commit(TourOrder&&);
  std::deque<ExplorationGoal> goals_;
  std::uint64_t revision_ = 0;
};
}  // namespace navigation2d::exploration
