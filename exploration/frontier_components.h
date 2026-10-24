#pragma once
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d::frontier_internal {
// Owns the ephemeral boundary mask, visited marks and row-major component
// cursor. Positive dimensions and complete, matching evidence/travel fields
// are required. Discover borrows evidence only for that call. Next returns each
// 8-connected component in original seed/BFS order.
class FrontierComponents {
 public:
  FrontierComponents(int width, int height);
  std::size_t Discover(const ExplorationGrid& observation, std::span<const int> travel);
  [[nodiscard]] std::optional<std::vector<int>> Next();

 private:
  int width_, height_, next_seed_ = 0;
  bool discovered_ = false;
  std::vector<unsigned char> frontier_, visited_;
};
struct ClusterAnchor {
  double mean_col, mean_row;
  int representative;
  double frontier_x, frontier_y;
};
// Nonempty component required; a real member nearest the centroid is used.
[[nodiscard]] ClusterAnchor LocateCluster(std::span<const int>, const ExplorationGrid& planning);
}  // namespace navigation2d::frontier_internal
