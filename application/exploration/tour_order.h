#pragma once
#include <deque>
#include <optional>

#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d {
class FrontierExplorer;
}
namespace navigation2d::exploration {
struct TourPose {
  double x, y, yaw;
};  // World frame, metres/radians.
// Synchronous read-only constraints; implementations may query the mission
// clock. No node or mutable mission context is retained by ordering policies.
class TourConstraints {
 public:
  virtual ~TourConstraints() = default;
  virtual bool RecentlyDemoted(const ExplorationGoal&) = 0;
  virtual bool GoalIsBehindTrail(const ExplorationGoal&) const = 0;
};
// Reuse the original partition buffers; a wall-tip reorder needs two queues,
// while mouth failure alone defers local tasks behind the driven trail.
struct TourOrder {
  std::deque<ExplorationGoal> priority_local, far;
  std::optional<std::deque<ExplorationGoal>> deferred_local;
};
TourOrder MouthTourOrder(const std::deque<ExplorationGoal>&, FrontierExplorer&,
                         const ExplorationGoal& mouth, std::optional<TourPose>, TourConstraints&);
TourOrder WallTipTourOrder(const std::deque<ExplorationGoal>&, FrontierExplorer&,
                           const ExplorationGoal& tip, std::optional<TourPose>, TourConstraints&);
}  // namespace navigation2d::exploration
