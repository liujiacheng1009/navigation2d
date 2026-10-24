#pragma once
#include <algorithm>
#include <cmath>
#include <utility>

#include "navigation2d/exploration/observation_grid_view.h"
#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::frontier_internal {
// Borrowed planning geometry. Unknown occupancy and cell-box clearance retain
// their distinct contracts; the observation grid authorizes viewpoints.
// Callers provide positive resolution and dimensions matching cell storage.
class PlanningGridView {
 public:
  PlanningGridView(const ExplorationGrid& planning, const ExplorationGrid& observation,
                   double footprint_clearance_m)
      : planning_(planning),
        observation_(observation),
        footprint_clearance_m_(footprint_clearance_m) {}
  PlanningGridView(const ExplorationGrid&&, const ExplorationGrid&, double) = delete;
  PlanningGridView(const ExplorationGrid&, const ExplorationGrid&&, double) = delete;
  bool Free(int col, int row) const {
    return col >= 0 && row >= 0 && col < planning_.width && row < planning_.height &&
           planning_.cells[static_cast<std::size_t>(row) * planning_.width + col] == 0;
  }
  bool ClearAt(int col, int row, double clearance) const;
  bool SafeViewpoint(int col, int row) const;
  std::pair<double, double> CellCenter(int col, int row) const {
    return {planning_.origin_x + (col + .5) * planning_.resolution,
            planning_.origin_y + (row + .5) * planning_.resolution};
  }
  bool HasFreeLineOfSight(int from_col, int from_row, int to_col, int to_row) const {
    // A standoff pose is useful only when its lidar can actually observe the
    // free/unknown boundary that created the task.  Euclidean-only selection
    // used to choose poses across the corner of a shelf: reachable by a long
    // detour, but unable to reveal that frontier once reached.
    int x = from_col, y = from_row;
    const int dx = std::abs(to_col - from_col), sx = from_col < to_col ? 1 : -1;
    const int dy = -std::abs(to_row - from_row), sy = from_row < to_row ? 1 : -1;
    int error = dx + dy;
    while (true) {
      if (!Free(x, y)) {
        return false;
      }
      if (x == to_col && y == to_row) {
        return true;
      }
      const int twice_error = 2 * error;
      if (twice_error >= dy) {
        error += dy;
        x += sx;
      }
      if (twice_error <= dx) {
        error += dx;
        y += sy;
      }
      if (x < 0 || y < 0 || x >= planning_.width || y >= planning_.height) {
        return false;
      }
    }
  }

 private:
  const ExplorationGrid& planning_;
  const ExplorationGrid& observation_;
  double footprint_clearance_m_;
};
}  // namespace navigation2d::frontier_internal
