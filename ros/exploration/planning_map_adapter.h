#pragma once
#include <string>
#include <utility>

#include "navigation2d/application/exploration/planning_snapshot.h"
namespace navigation2d::exploration_io {
enum class SnapshotWriteStatus { kWritten, kStreamFailed };
// File/snapshot preparation only. Owns reusable evidence and the last prepared
// map's frame/path, including rejected plans (legacy dispatch semantics).
// Inputs and views are synchronous; snapshot views expire at the next Prepare.
class PlanningMapAdapter {
 public:
  explicit PlanningMapAdapter(std::string prefix) : prefix_(std::move(prefix)) {}
  SnapshotWriteStatus Prepare(const ExplorationGrid& map,
                              const exploration::SnapshotRequest& request);
  const std::string& path() const { return path_; }
  double origin_x() const { return origin_x_; }
  double origin_y() const { return origin_y_; }
  const exploration::PlanningSnapshot& snapshot() const { return snapshot_; }

 private:
  std::string prefix_, path_;
  double origin_x_ = 0., origin_y_ = 0.;
  exploration::PlanningSnapshotBuilder builder_;
  exploration::PlanningSnapshot snapshot_;
};
}  // namespace navigation2d::exploration_io
