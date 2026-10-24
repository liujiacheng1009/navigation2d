#include "navigation2d/application/exploration/route_failure_history.h"
namespace navigation2d::exploration {
void RouteFailureHistory::ObserveCell(std::pair<int, int> cell) {
  if (!cell_ || *cell_ != cell) {
    cell_ = cell;
    failures_at_pose_ = 0;
    signatures_.clear();
  }
}
void RouteFailureHistory::ObserveMapGrowth(std::size_t known_cells) {
  if (known_cells >= last_known_cells_ + 500) {
    signatures_.clear();
    last_known_cells_ = known_cells;
  }
}
int RouteFailureHistory::NoteFailure(std::uint64_t signature) {
  if (signature != 0) {
    signatures_.insert(signature);
  }
  return ++failures_at_pose_;
}
}  // namespace navigation2d::exploration
