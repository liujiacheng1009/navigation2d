#pragma once
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <unordered_set>
#include <vector>

#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/geometry/pose_2d.h"

namespace navigation2d::exploration {
// Borrowed for one synchronous call. Range storage stays owned by the ROS
// adapter; no conversion/allocation is required for safety checks.
struct ScanEvidenceView {
  double angle_min = 0., angle_increment = 0., range_min = 0., range_max = 0.;
  std::span<const float> ranges;
};
struct SnapshotGeometry {
  int width = 0, height = 0;
  double resolution = 0., origin_x = 0., origin_y = 0.;
};
struct PlanningSnapshot {
  SnapshotGeometry grid;
  std::vector<std::uint8_t> free_mask;
  std::uint64_t map_revision = 0;
  double trail_budget_m = 0.;
};
// Cells are used while preparing/serializing a plan, not retained for its
// execution lifetime. Only geometry and the evidence mask survive commit.
struct PreparedSnapshot {
  PlanningSnapshot snapshot;
  std::vector<std::int8_t> cells;
};
struct SnapshotRequest {
  Pose2d robot;
  ExplorationGoal goal;
  std::optional<ExplorationGoal> entry;
  std::optional<ScanEvidenceView> scan;
  std::span<const std::pair<double, double>> trail;
  double trail_budget_m = 0.;
  bool returning = false;
  std::uint64_t map_revision = 0;
};
// Preserves current body, ray-interior, doorway and driven-trail policies.
// The input observation is never changed; all evidence expires with this plan.
class PlanningSnapshotBuilder {
 public:
  PreparedSnapshot Build(const ExplorationGrid& map, const SnapshotRequest& request,
                         PlanningSnapshot reusable = {});
 private:
  std::unordered_set<std::size_t> driven_corridor_;
};
// One-shot convenience for deterministic tests and callers without a session.
PreparedSnapshot BuildPlanningSnapshot(const ExplorationGrid& map, const SnapshotRequest& request);
}  // namespace navigation2d::exploration
