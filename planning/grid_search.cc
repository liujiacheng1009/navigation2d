#include "navigation2d/planning/grid_search.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

namespace navigation2d::planning_internal {
namespace {
struct Entry {
  double priority;
  int cell;
  bool operator>(const Entry& rhs) const { return priority > rhs.priority; }
};
constexpr std::array<std::array<int, 2>, 8> kDirections{{
    {{1, 0}}, {{-1, 0}}, {{0, 1}}, {{0, -1}},
    {{1, 1}}, {{1, -1}}, {{-1, 1}}, {{-1, -1}}}};
}  // namespace

bool HasLineOfSight(const LayeredCostmap& costmap, int from, int to,
                    double clearance) {
  const Grid2d& grid = costmap.grid();
  const int x0 = from % grid.width(), y0 = from / grid.width();
  const int x1 = to % grid.width(), y1 = to / grid.width();
  const int samples = std::max(
      1, static_cast<int>(std::ceil(std::hypot(x1 - x0, y1 - y0) * 2.)));
  for (int sample = 1; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / samples;
    const int x = static_cast<int>(std::lround(x0 + ratio * (x1 - x0)));
    const int y = static_cast<int>(std::lround(y0 + ratio * (y1 - y0)));
    const auto [wx, wy] = grid.CellCenter(x, y);
    if (costmap.lethal(wx, wy, clearance) || costmap.cost(x, y) > 128) return false;
  }
  return true;
}

Path GridSearch(const LayeredCostmap& costmap, const Pose2d& start,
                const Pose2d& goal, double clearance, const Heuristic& heuristic,
                const ParentRelaxation& relax_parent) {
  const Grid2d& grid = costmap.grid();
  const auto [sx, sy] = grid.ToCell(X(start), Y(start));
  const auto [gx, gy] = grid.ToCell(X(goal), Y(goal));
  if (costmap.lethal(X(start), Y(start), clearance) ||
      costmap.lethal(X(goal), Y(goal), clearance))
    throw std::runtime_error("start or goal is occupied");
  const int size = grid.width() * grid.height();
  const int source = sy * grid.width() + sx, target = gy * grid.width() + gx;
  std::vector<double> distance(size, std::numeric_limits<double>::infinity());
  std::vector<int> parent(size, -1);
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  distance[source] = 0.;
  open.push({heuristic(sx, sy, gx, gy), source});
  while (!open.empty()) {
    const Entry current = open.top();
    open.pop();
    const int x = current.cell % grid.width(), y = current.cell / grid.width();
    if (current.priority != distance[current.cell] + heuristic(x, y, gx, gy)) continue;
    if (current.cell == target) break;
    for (const auto& d : kDirections) {
      const int nx = x + d[0], ny = y + d[1];
      if (nx < 0 || ny < 0 || nx >= grid.width() || ny >= grid.height()) continue;
      const auto [wx, wy] = grid.CellCenter(nx, ny);
      if (costmap.lethal(wx, wy, clearance)) continue;
      if (d[0] && d[1]) {
        const auto [ax, ay] = grid.CellCenter(x + d[0], y);
        const auto [bx, by] = grid.CellCenter(x, y + d[1]);
        if (costmap.lethal(ax, ay, clearance) || costmap.lethal(bx, by, clearance)) continue;
      }
      const int next = ny * grid.width() + nx;
      const double traversal = 1. + static_cast<double>(costmap.cost(nx, ny)) / 252.;
      int predecessor = current.cell;
      double candidate = distance[current.cell] +
          (d[0] && d[1] ? std::sqrt(2.) : 1.) * traversal;
      if (relax_parent && parent[current.cell] >= 0)
        relax_parent(costmap, parent[current.cell], next, current.cell,
                     distance[parent[current.cell]], traversal, &predecessor,
                     &candidate);
      if (candidate < distance[next]) {
        distance[next] = candidate;
        parent[next] = predecessor;
        open.push({candidate + heuristic(nx, ny, gx, gy), next});
      }
    }
  }
  if (parent[target] < 0 && source != target) throw std::runtime_error("no path");
  Path reversed{goal};
  for (int cell = target; cell != source; cell = parent[cell]) {
    const auto [x, y] = grid.CellCenter(cell % grid.width(), cell / grid.width());
    reversed.push_back(MakePose2d(x, y, 0.));
  }
  reversed.push_back(start);
  std::reverse(reversed.begin(), reversed.end());
  return reversed;
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
  return dense;
}

}  // namespace navigation2d::planning_internal
