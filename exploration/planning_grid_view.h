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
  bool ClearAt(int col, int row, double clearance) const {
    // Match LayeredCostmap::lethal: continuous distance to occupied cells with
    // a half-cell box margin. The old discrete disk accepted poses the planner
    // then rejected (and vice versa) because GridSearch uses this geometry.
    if (col < 0 || row < 0 || col >= planning_.width || row >= planning_.height) {
      return false;
    }
    if (planning_.cells[static_cast<std::size_t>(row) * planning_.width + col] != 0) {
      return false;
    }
    const auto [wx, wy] = CellCenter(col, row);
    const double half = .5 * planning_.resolution;
    const int cells = static_cast<int>(std::ceil(
        (std::max(0., clearance) + std::sqrt(.5) * planning_.resolution) / planning_.resolution));
    for (int dy = -cells; dy <= cells; ++dy) {
      for (int dx = -cells; dx <= cells; ++dx) {
        const int x = col + dx, y = row + dy;
        if (x < 0 || y < 0 || x >= planning_.width || y >= planning_.height) {
          return false;
        }
        // Unknown is unobserved space, not a jamb — same as costmap lethal().
        if (planning_.cells[static_cast<std::size_t>(y) * planning_.width + x] <= 0) {
          continue;
        }
        const auto [ox, oy] = CellCenter(x, y);
        const double nearest_x = std::max(std::abs(wx - ox) - half, 0.);
        const double nearest_y = std::max(std::abs(wy - oy) - half, 0.);
        if (std::hypot(nearest_x, nearest_y) <= clearance) {
          return false;
        }
      }
    }
    return true;
  }
  bool SafeViewpoint(int col, int row) const {
    // The search graph stays on known free cells. Unknown may lie inside the
    // clearance disk, but the viewpoint itself is not placed in unknown space.
    if (!Free(col, row)) {
      return false;
    }
    // Tour goals must sit on mapper-observed free (planning map == observation).
    if (!ObservationGridView(observation_).Free(col, row)) {
      return false;
    }
    if (ClearAt(col, row, footprint_clearance_m_)) {
      return true;
    }
    // Narrow-passage candidate: permit only the physical footprint radius when
    // the free run is corridor-shaped and its measured width is in the range
    // where a robot can fit. Open areas retain the normal clearance contract.
    const int probe = static_cast<int>(std::ceil(1.4 / planning_.resolution));
    int horizontal = 0, vertical = 0;
    for (int x = col; x >= 0 && x > col - probe && Free(x, row); --x) {
      ++horizontal;
    }
    for (int x = col + 1; x < planning_.width && x <= col + probe && Free(x, row); ++x) {
      ++horizontal;
    }
    for (int y = row; y >= 0 && y > row - probe && Free(col, y); --y) {
      ++vertical;
    }
    for (int y = row + 1; y < planning_.height && y <= row + probe && Free(col, y); ++y) {
      ++vertical;
    }
    const int narrow_min = static_cast<int>(std::ceil(kNarrowMinWidth() / planning_.resolution));
    const int narrow_max = static_cast<int>(std::floor(kNarrowMaxWidth() / planning_.resolution));
    const bool narrow_h = horizontal >= narrow_min && horizontal <= narrow_max;
    const bool narrow_v = vertical >= narrow_min && vertical <= narrow_max;
    // A kitchen mouth often has BOTH axes in the door band (short vestibule).
    // The old XOR treated that as open space and demanded footprint_clearance
    // (.30), which sealed 0.50–0.60 m free throats after SLAM thickened jambs.
    const int smaller = std::min(horizontal, vertical);
    const bool corridor = narrow_h || narrow_v || (smaller >= narrow_min && smaller <= narrow_max);
    // Wide open space keeps footprint_clearance (typically .30 with inflation).
    // Corridor / doorway cells must use kPlanningRadius(): requiring
    // .30 m half-width rejects a physical 0.70 m kitchen door after SLAM
    // thickens the jambs, so entry probes never reach the room side.
    const double corridor_clearance =
        std::max(kNarrowClearance(), std::min(footprint_clearance_m_, kPlanningRadius()));
    return corridor && ClearAt(col, row, corridor_clearance);
  }
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
