#pragma once
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>
namespace navigation2d::costmap_internal {
// Master-grid changes only: no-op observations do not consume a revision.
// Retains the last 64 change sets. Older queries conservatively return every
// cell; current/future queries are empty. Queries return sorted unique indices.
class GridChangeHistory {
 public:
  // Both views have the same extent; neither is retained after this call.
  void Record(std::span<const std::uint8_t> previous, std::span<const std::uint8_t> current);
  std::uint64_t revision() const { return revision_; }
  std::vector<int> Since(std::uint64_t revision, std::size_t cells) const;

 private:
  struct ChangeSet {
    std::uint64_t revision;
    std::vector<int> cells;
  };
  std::uint64_t revision_ = 0;
  std::vector<ChangeSet> changes_;
};
}  // namespace navigation2d::costmap_internal
