#pragma once
#include <algorithm>
#include <cstddef>

#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d::frontier_internal {
// Borrowed, read-only mapper evidence. The grid outlives this view.
// Cell queries require dimensions matching storage; CountFrontiers rejects
// malformed shapes, while KnownCells deliberately counts received evidence.
class ObservationGridView {
 public:
  explicit ObservationGridView(const ExplorationGrid& grid) : grid_(grid) {}
  ObservationGridView(const ExplorationGrid&&) = delete;
  bool Free(int col, int row) const {
    return col >= 0 && row >= 0 && col < grid_.width && row < grid_.height &&
           grid_.cells[static_cast<std::size_t>(row) * grid_.width + col] == 0;
  }
  bool Frontier(int col, int row) const {
    if (!Free(col, row)) {
      return false;
    }
    constexpr int dx[] = {-1, 1, 0, 0};
    constexpr int dy[] = {0, 0, -1, 1};
    for (int direction = 0; direction < 4; ++direction) {
      const int x = col + dx[direction], y = row + dy[direction];
      if (x >= 0 && y >= 0 && x < grid_.width && y < grid_.height &&
          grid_.cells[static_cast<std::size_t>(y) * grid_.width + x] < 0) {
        return true;
      }
    }
    return false;
  }
  std::size_t CountFrontiers() const {
    if (grid_.width <= 0 || grid_.height <= 0 ||
        grid_.cells.size() != static_cast<std::size_t>(grid_.width * grid_.height)) {
      return 0;
    }
    std::size_t count = 0;
    for (int row = 0; row < grid_.height; ++row) {
      for (int col = 0; col < grid_.width; ++col) {
        if (Frontier(col, row)) {
          ++count;
        }
      }
    }
    return count;
  }
  std::size_t KnownCells() const {
    return static_cast<std::size_t>(std::count_if(grid_.cells.begin(), grid_.cells.end(),
                                                  [](std::int8_t value) { return value >= 0; }));
  }

 private:
  const ExplorationGrid& grid_;
};
}  // namespace navigation2d::frontier_internal
