#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>

#include "navigation2d/exploration/frontier_types.h"
#include "navigation2d/geometry/twist_2d.h"
namespace navigation2d {
class FrontierExplorer;
}
namespace navigation2d::exploration {
struct EntryEvidence {
  std::size_t known_cells;
  std::uint64_t map_revision;
};
struct EntryCrawlObservation {
  double x, y, yaw;  // World frame, metres/radians.
  bool segment_blocked;
};
struct EntryCrawlStep {
  enum class State { kBlocked, kDriving, kLastCommand };
  State state;
  Twist2d command;
};
// Entry state has one owner. Geometry may query the read-only exploration
// map; this class has no ROS, publication or navigation-commit responsibility.
class EntryCoordinator {
 public:
  static bool SameMouth(const ExplorationGoal& a, const ExplorationGoal& b);
  void Lock(const ExplorationGoal& mouth, bool easy, EntryEvidence evidence);
  void Clear();
  // Lock-time evidence and live fallback share the real-door policy. A latched
  // easy mouth survives later map flooding that removes its corridor label.
  void LockObserved(const ExplorationGoal&, const FrontierExplorer&, EntryEvidence);
  [[nodiscard]] bool EasyNow(const FrontierExplorer&) const;
  [[nodiscard]] int AttemptBudget(const FrontierExplorer& explorer) const {
    return EasyNow(explorer) ? 14 : 6;
  }
  [[nodiscard]] static bool Passable(const ExplorationGoal&, const FrontierExplorer&);
  void SetPending(std::optional<ExplorationGoal> goal) { pending_ = std::move(goal); }
  void ClearPending() { pending_.reset(); }
  void CountAttempt() { ++attempts_; }
  void NoteEvidence(EntryEvidence evidence) { evidence_ = evidence; }
  void StartCrawl() { crawl_ticks_ = 1; }
  // Caller checks the live segment before requesting motion, then publishes
  // this command before handling kLastCommand (legacy 41st tick semantics).
  EntryCrawlStep AdvanceCrawl(EntryCrawlObservation robot, const ExplorationGoal& target);
  void StopCrawl() { crawl_ticks_ = 0; }
  const std::optional<ExplorationGoal>& pending() const { return pending_; }
  const std::optional<ExplorationGoal>& locked() const { return locked_; }
  // Synchronous read-only borrow; pending takes precedence over the lock.
  // Any entry-state update may invalidate the returned pointer.
  [[nodiscard]] const ExplorationGoal* ActiveMouth() const& noexcept {
    if (pending_) {
      return &*pending_;
    }
    return locked_ ? &*locked_ : nullptr;
  }
  const ExplorationGoal* ActiveMouth() const&& = delete;
  int attempts() const { return attempts_; }
  int crawl_ticks() const { return crawl_ticks_; }
  bool easy() const { return easy_; }
  EntryEvidence evidence() const { return evidence_; }
  double Depth(double rx, double ry) const;
  std::optional<double> InteriorDepth(double rx, double ry,
                                      const ExplorationGoal& active_goal) const;
  static bool EnoughDepth(double depth, double gap) { return depth > (gap >= .50 ? .30 : .48); }
  bool Satisfied(double rx, double ry, const ExplorationGoal& active_goal, double gap) const;
  bool Satisfied(double rx, double ry, const ExplorationGoal& active_goal,
                 const FrontierExplorer& explorer) const;

 private:
  std::optional<ExplorationGoal> pending_, locked_;
  int attempts_ = 0;
  int crawl_ticks_ = 0;
  bool easy_ = false;
  EntryEvidence evidence_{};
};
}  // namespace navigation2d::exploration
