#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>
namespace navigation2d::exploration {
// One owner for cooled route signatures and the current pose-cell failure
// budget. Clearing signatures does not reset failure count or map-growth
// anchor; moving cells resets only the cell budget and signature exclusions.
class RouteFailureHistory {
 public:
  void ObserveCell(std::pair<int, int> cell);
  void ObserveMapGrowth(std::size_t known_cells);
  void Cool(std::uint64_t signature) { signatures_.insert(signature); }
  void ClearSignatures() { signatures_.clear(); }
  int NoteFailure(std::uint64_t signature);
  // This budget is independent of cell/map-growth and signature clearing.
  // Only a prepared route or consumed two-reject recovery resets it.
  void NoteMappedReject() { ++mapped_rejects_; }
  void ResetMappedRejects() { mapped_rejects_ = 0; }
  bool TakeMappedThrash() {
    if (mapped_rejects_ < 2) {
      return false;
    }
    mapped_rejects_ = 0;
    return true;
  }
  [[nodiscard]] int failures_at_pose() const { return failures_at_pose_; }
  [[nodiscard]] std::size_t size() const { return signatures_.size(); }
  [[nodiscard]] bool Contains(std::uint64_t signature) const {
    return signatures_.contains(signature);
  }
  // Owned handoff snapshot; later cooling does not invalidate the request.
  [[nodiscard]] std::vector<std::uint64_t> CooledSignatures() const {
    return std::vector<std::uint64_t>(signatures_.begin(), signatures_.end());
  }

 private:
  std::unordered_set<std::uint64_t> signatures_;
  std::optional<std::pair<int, int>> cell_;
  std::size_t last_known_cells_ = 0;
  int failures_at_pose_ = 0;
  int mapped_rejects_ = 0;
};
}  // namespace navigation2d::exploration
