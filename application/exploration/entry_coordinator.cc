#include "navigation2d/application/exploration/entry_coordinator.h"

#include <cmath>
namespace navigation2d::exploration {
void EntryCoordinator::Lock(const ExplorationGoal& mouth, bool easy, EntryEvidence evidence) {
  locked_ = mouth;
  pending_ = mouth;
  attempts_ = 0;
  crawl_ticks_ = 0;
  easy_ = easy;
  evidence_ = evidence;
}
void EntryCoordinator::Clear() {
  locked_.reset();
  pending_.reset();
  attempts_ = 0;
  crawl_ticks_ = 0;
  easy_ = false;
}
double EntryCoordinator::Depth(double rx, double ry) const {
  if (!pending_) {
    return 0.;
  }
  const double direction_x = pending_->frontier_x - pending_->x;
  const double direction_y = pending_->frontier_y - pending_->y;
  const double length = std::hypot(direction_x, direction_y);
  if (length < .05) {
    return 0.;
  }
  return ((rx - pending_->frontier_x) * direction_x + (ry - pending_->frontier_y) * direction_y) /
         length;
}

std::optional<double> EntryCoordinator::InteriorDepth(double rx, double ry,
                                                      const ExplorationGoal& active_goal) const {
  if (!pending_ || attempts_ < 1) {
    return std::nullopt;
  }
  // Must be well inside the room — 0.22 m past the jambs was "just over the
  // doorway then leave" (kitchen white wall still unmapped).
  const ExplorationGoal& mouth = locked_.value_or(*pending_);
  const double fx = mouth.frontier_x;
  const double fy = mouth.frontier_y;
  if (std::hypot(rx - fx, ry - fy) > 1.35) {
    return std::nullopt;
  }
  double into_x = active_goal.x - fx;
  double into_y = active_goal.y - fy;
  double length = std::hypot(into_x, into_y);
  if (length < .12) {
    into_x = fx - mouth.x;
    into_y = fy - mouth.y;
    length = std::hypot(into_x, into_y);
  }
  if (length < .05) {
    return std::nullopt;
  }
  const double ux = into_x / length, uy = into_y / length;
  const double lateral = std::abs((rx - fx) * (-uy) + (ry - fy) * ux);
  if (lateral > .60) {
    return std::nullopt;
  }
  const double robot_side = (rx - fx) * ux + (ry - fy) * uy;
  // Wide easy doors: a shallow mid-gap poke (~0.30 m) is enough to mark
  // entered. Narrow mouths still need ~0.48 m so we do not peek-and-leave.
  return robot_side;
}

bool EntryCoordinator::Satisfied(double rx, double ry, const ExplorationGoal& active_goal,
                                 double gap) const {
  const auto depth = InteriorDepth(rx, ry, active_goal);
  return depth && EnoughDepth(*depth, gap);
}
}  // namespace navigation2d::exploration
