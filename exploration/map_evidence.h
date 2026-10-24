#pragma once
#include <utility>

#include "navigation2d/exploration/planning_grid_view.h"
namespace navigation2d::frontier_internal {
// Owns mapper observation and the planning snapshot separately. Frontier
// identity/completion use observation; geometry queries use planning. Each
// update preserves the original single full-grid copy followed by a move.
// Notify synchronously after observation commit and before planning commit;
// an observer exception intentionally leaves the prior planning snapshot.
class MapEvidence {
 public:
  template <typename OnObserved>
  void Update(ExplorationGrid&& map, OnObserved&& observed) {
    observation_ = map;
    std::forward<OnObserved>(observed)(observation_);
    planning_ = std::move(map);
  }
  const ExplorationGrid& observation() const& { return observation_; }
  const ExplorationGrid& observation() const&& = delete;
  const ExplorationGrid& planning() const& { return planning_; }
  const ExplorationGrid& planning() const&& = delete;
  ObservationGridView observation_view() const& { return ObservationGridView(observation_); }
  ObservationGridView observation_view() const&& = delete;
  PlanningGridView planning_view(double footprint_m) const& {
    return PlanningGridView(planning_, observation_, footprint_m);
  }
  PlanningGridView planning_view(double) const&& = delete;

 private:
  ExplorationGrid observation_;
  ExplorationGrid planning_;
};
}  // namespace navigation2d::frontier_internal
