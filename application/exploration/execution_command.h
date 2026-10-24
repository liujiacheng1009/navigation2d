#pragma once
#include <optional>

#include "navigation2d/application/exploration/tour_order.h"
#include "navigation2d/geometry/twist_2d.h"
namespace navigation2d::exploration {
// Only supplied for a live, non-retreat, non-succeeded mouth approach. All
// coordinates are in the world frame; pose is a synchronous value snapshot.
struct EntryApproach {
  ExplorationGoal mouth, target;
  TourPose robot;
};
class ExecutionCommandEvidence {
 public:
  virtual ~ExecutionCommandEvidence() = default;
  virtual double CommandFrontRange() const = 0;
  virtual bool CommandSegmentCutsWall(double x, double y, const ExplorationGoal&) const = 0;
};
// Apply mouth crawl only without a safety hold. The final lidar body gate
// always runs after any compensation and before the caller publishes.
[[nodiscard]] Twist2d ExecutionCommand(Twist2d requested, bool safety_held,
                                       const std::optional<EntryApproach>&,
                                       const ExecutionCommandEvidence&);
}  // namespace navigation2d::exploration
