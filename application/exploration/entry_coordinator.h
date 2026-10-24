#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>

#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/geometry/pose_2d.h"
namespace navigation2d::exploration {
struct EntryEvidence {
  std::size_t known_cells;
  std::uint64_t map_revision;
};
// Entry state has one owner. Callers provide map-derived facts; this class
// neither queries ROS nor publishes commands or commits navigation goals.
class EntryCoordinator {
 public:
  void Lock(const ExplorationGoal& mouth, bool easy, EntryEvidence evidence);
  void Clear();
  void SetPending(std::optional<ExplorationGoal> goal) { pending_ = std::move(goal); }
  void ClearPending() { pending_.reset(); }
  void CountAttempt() { ++attempts_; }
  void CancelAttempt() { --attempts_; }
  void NoteEvidence(EntryEvidence evidence) { evidence_ = evidence; }
  void StartCrawl() { crawl_ticks_ = 1; }
  void TickCrawl() { ++crawl_ticks_; }
  void StopCrawl() { crawl_ticks_ = 0; }
  const std::optional<ExplorationGoal>& pending() const { return pending_; }
  const std::optional<ExplorationGoal>& locked() const { return locked_; }
  int attempts() const { return attempts_; }
  int crawl_ticks() const { return crawl_ticks_; }
  bool easy() const { return easy_; }
  EntryEvidence evidence() const { return evidence_; }
  double Depth(double rx, double ry) const;
  std::optional<double> InteriorDepth(double rx, double ry,
                                      const ExplorationGoal& active_goal) const;
  static bool EnoughDepth(double depth, double gap) { return depth > (gap >= .50 ? .30 : .48); }
  bool Satisfied(double rx, double ry, const ExplorationGoal& active_goal, double gap) const;

 private:
  std::optional<ExplorationGoal> pending_, locked_;
  int attempts_ = 0;
  int crawl_ticks_ = 0;
  bool easy_ = false;
  EntryEvidence evidence_{};
};
}  // namespace navigation2d::exploration
