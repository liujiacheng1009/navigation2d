#include "navigation2d/planning/grid_search.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>
#include "navigation2d/planning/search_core.h"

namespace navigation2d::planning_internal {
namespace {
constexpr std::array<std::array<int, 2>, 8> kDirections{{
    {{1, 0}}, {{-1, 0}}, {{0, 1}}, {{0, -1}},
    {{1, 1}}, {{1, -1}}, {{-1, 1}}, {{-1, -1}}}};

// Inflation already encodes distance to the nearest obstacle in the master
// costmap. Treat that cost as a continuous clearance penalty instead of only
// using it as a binary collision guard. A longer path through the middle of a
// wide room is preferable to a shortest path that rides the inflated edge;
// narrow passages remain usable because their only available cells retain the
// same penalty and are still selected when no higher-clearance route exists.
// Prefer corridor center over wall-hugging shortest paths.  Narrow doors still
// win when they are the only connection: every cell there carries a similar
// penalty, so the relative ranking stays length-dominated. Open rooms with a
// single speck used to keep shortest chords on the inflated rim (weight 32);
// a stronger preference pulls routes into free space when it is available.
constexpr double kClearancePenaltyWeight = 72.0;

double TraversalCellCost(std::uint8_t cost) {
  if (cost == kFree) return 1.;
  const double normalized = static_cast<double>(cost) / 252.;
  return 1. + kClearancePenaltyWeight * normalized * normalized;
}
}  // namespace

bool HasLineOfSight(const LayeredCostmap& costmap, int from, int to,
                    double clearance) {
  return SegmentTraversalCost(costmap, from, to, clearance).has_value();
}

std::optional<double> SegmentTraversalCost(const LayeredCostmap& costmap, int from, int to,
                                           double clearance) {
  const Grid2d& grid = costmap.grid();
  const int x0 = from % grid.width(), y0 = from / grid.width();
  const int x1 = to % grid.width(), y1 = to / grid.width();
  // Supercover Bresenham: visit every cell the chord touches. Uniform
  // parametric sampling used to skip single-cell walls on long Theta*
  // visibility edges, producing densified paths that cut through obstacles.
  const int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
  int x = x0, y = y0;
  int err = dx - dy;
  double accumulated = 0.;
  int visited = 0;
  const int max_steps = dx + dy + 2;
  for (int step = 0; step < max_steps; ++step) {
    const auto [wx, wy] = grid.CellCenter(x, y);
    // Unknown is not a wall, but a line of sight must not cross it.
    if (costmap.cost(x, y) == kUnknown || costmap.lethal(wx, wy, clearance))
      return std::nullopt;
    accumulated += TraversalCellCost(costmap.cost(x, y));
    ++visited;
    if (x == x1 && y == y1) break;
    const int e2 = 2 * err;
    // When the ray crosses a grid corner, step through both adjacent cells so
    // a diagonal cannot slip between two lethal cells that meet at a point.
    if (e2 >= -dy && e2 <= dx && dx > 0 && dy > 0) {
      const int cx = x + sx, cy = y + sy;
      if (cx != x1 || cy != y1) {
        const auto [cwx, cwy] = grid.CellCenter(cx, y);
        if (costmap.cost(cx, y) == kUnknown || costmap.lethal(cwx, cwy, clearance))
          return std::nullopt;
        const auto [cwx2, cwy2] = grid.CellCenter(x, cy);
        if (costmap.cost(x, cy) == kUnknown || costmap.lethal(cwx2, cwy2, clearance))
          return std::nullopt;
      }
    }
    if (e2 >= -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }
  if (x != x1 || y != y1) return std::nullopt;
  const double length_cells = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
  if (visited <= 0) return std::nullopt;
  return accumulated * (length_cells / static_cast<double>(visited));
}

Path GridSearch(const LayeredCostmap& costmap, const Pose2d& start,
                const Pose2d& goal, double clearance, const Heuristic& heuristic,
                const ParentRelaxation& relax_parent) {
  const Grid2d& grid = costmap.grid();
  const auto [sx, sy] = grid.ToCell(X(start), Y(start));
  const auto [gx, gy] = grid.ToCell(X(goal), Y(goal));
  const auto reject_occupied = [&](const Pose2d& pose, const char* which,
                                   double pose_clearance) {
    if (!costmap.lethal(X(pose), Y(pose), pose_clearance)) return;
    const double nearest = costmap.nearestLethalDistance(X(pose), Y(pose));
    std::ostringstream message;
    message << which << " occupied";
    if (std::isfinite(nearest)) {
      message << ", nearest obstacle " << std::fixed << std::setprecision(2) << nearest << " m";
    }
    throw std::runtime_error(message.str());
  };
  reject_occupied(start, "start", clearance);
  // Path search keeps the half-cell raster margin in `clearance`.  The
  // terminal pose does not: explorers place doorway goals with ClearAt(robot
  // radius) (~0.22 m).  Requiring the inflated planning radius (~0.255 m)
  // rejected every 0.70 m kitchen entry with "goal occupied, nearest 0.23 m".
  const double goal_clearance =
      std::max(0., clearance - std::sqrt(.5) * grid.resolution());
  reject_occupied(goal, "goal", goal_clearance);
  if (gx >= 0 && gy >= 0 && gx < grid.width() && gy < grid.height() &&
      costmap.cost(gx, gy) == kUnknown)
    throw std::runtime_error("goal occupied, cell is unknown");
  const int source = sy * grid.width() + sx, target = gy * grid.width() + gx;
  const auto result = BestFirstSearch(
      grid.width() * grid.height(), source,
      [target](int state) { return state == target; },
      [&](int state) {
        return heuristic(state % grid.width(), state / grid.width(), gx, gy);
      },
      [&](int current, int current_parent, const SuccessorVisitor& visit) {
    const int x = current % grid.width(), y = current / grid.width();
    for (const auto& d : kDirections) {
      const int nx = x + d[0], ny = y + d[1];
      if (nx < 0 || ny < 0 || nx >= grid.width() || ny >= grid.height()) continue;
      // Step onto a free cell beside unknown. Do not enter the unknown cell.
      if (costmap.cost(nx, ny) == kUnknown) continue;
      const auto [wx, wy] = grid.CellCenter(nx, ny);
      if (costmap.lethal(wx, wy, clearance)) continue;
      if (d[0] && d[1]) {
        if (costmap.cost(x + d[0], y) == kUnknown || costmap.cost(x, y + d[1]) == kUnknown)
          continue;
        const auto [ax, ay] = grid.CellCenter(x + d[0], y);
        const auto [bx, by] = grid.CellCenter(x, y + d[1]);
        if (costmap.lethal(ax, ay, clearance) || costmap.lethal(bx, by, clearance)) continue;
      }
      const int next = ny * grid.width() + nx;
      const double traversal = TraversalCellCost(costmap.cost(nx, ny));
      const double transition = (d[0] && d[1] ? std::sqrt(2.) : 1.) * traversal;
      visit({next, transition, current});
      if (relax_parent && current_parent >= 0) {
        const auto parent_transition = relax_parent(costmap, current_parent, next);
        if (parent_transition) visit({next, *parent_transition, current_parent});
      }
    }
  });
  if (!result.found) throw std::runtime_error("no path");
  const auto states = RestoreStatePath(result, source, target);
  Path reversed{goal};
  for (auto iterator = states.rbegin(); iterator != states.rend(); ++iterator) {
    const int cell = *iterator;
    if (cell == source) continue;
    const auto [x, y] = grid.CellCenter(cell % grid.width(), cell / grid.width());
    reversed.push_back(MakePose2d(x, y, 0.));
  }
  reversed.push_back(start);
  std::reverse(reversed.begin(), reversed.end());
  AssignPathOrientations(&reversed);
  return reversed;
}

void AssignPathOrientations(Path* path) {
  if (path == nullptr || path->size() < 2) return;
  const double goal_yaw = Yaw(path->back());
  for (std::size_t index = 0; index + 1 < path->size(); ++index) {
    const auto delta = (*path)[index + 1].translation() - (*path)[index].translation();
    const double yaw = delta.norm() > 1e-9 ? std::atan2(delta.y(), delta.x())
                                           : (index == 0 ? Yaw((*path)[index]) : Yaw((*path)[index - 1]));
    (*path)[index] = MakePose2d(X((*path)[index]), Y((*path)[index]), yaw);
  }
  path->back() = MakePose2d(X(path->back()), Y(path->back()), goal_yaw);
}

Path DensifyPath(const Path& path, double spacing) {
  if (path.empty()) return {};
  Path dense{path.front()};
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double length = (path[i].translation() - path[i - 1].translation()).norm();
    const int samples = std::max(1, static_cast<int>(std::ceil(length / spacing)));
    for (int sample = 1; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / samples;
      const Eigen::Vector2d point =
          path[i - 1].translation() + ratio * (path[i].translation() - path[i - 1].translation());
      dense.push_back(MakePose2d(point.x(), point.y(), 0.));
    }
  }
  dense.back() = MakePose2d(X(dense.back()), Y(dense.back()), Yaw(path.back()));
  AssignPathOrientations(&dense);
  return dense;
}

}  // namespace navigation2d::planning_internal
