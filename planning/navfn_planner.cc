#include "navigation2/planning/navfn_planner.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace navigation2d {
namespace {
struct Entry { double cost; int cell; bool operator>(const Entry& rhs) const { return cost > rhs.cost; } };
constexpr std::array<std::array<int, 2>, 8> kDirections{{
    {{1, 0}}, {{-1, 0}}, {{0, 1}}, {{0, -1}},
    {{1, 1}}, {{1, -1}}, {{-1, 1}}, {{-1, -1}}}};
}

Path NavFnPlanner::Plan(const Grid2d& grid, const Pose2d& start, const Pose2d& goal) const {
  const auto [sx, sy] = grid.ToCell(start.x, start.y);
  const auto [gx, gy] = grid.ToCell(goal.x, goal.y);
  if (grid.collides(start.x, start.y, clearance_) || grid.collides(goal.x, goal.y, clearance_))
    throw std::runtime_error("start or goal is occupied");
  const int size = grid.width() * grid.height();
  const int source = sy * grid.width() + sx, target = gy * grid.width() + gx;
  std::vector<double> distance(size, std::numeric_limits<double>::infinity());
  std::vector<int> parent(size, -1);
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  distance[source] = 0.; open.push({0., source});
  while (!open.empty()) {
    const Entry current = open.top(); open.pop();
    if (current.cost != distance[current.cell]) continue;
    if (current.cell == target) break;
    const int x = current.cell % grid.width(), y = current.cell / grid.width();
    for (const auto& d : kDirections) {
      const int nx = x + d[0], ny = y + d[1];
      if (nx < 0 || ny < 0 || nx >= grid.width() || ny >= grid.height()) continue;
      const auto [wx, wy] = grid.CellCenter(nx, ny);
      if (grid.collides(wx, wy, clearance_)) continue;
      if (d[0] && d[1]) {
        const auto [ax, ay] = grid.CellCenter(x + d[0], y);
        const auto [bx, by] = grid.CellCenter(x, y + d[1]);
        if (grid.collides(ax, ay, clearance_) || grid.collides(bx, by, clearance_)) continue;
      }
      const int next = ny * grid.width() + nx;
      const double candidate = current.cost + (d[0] && d[1] ? std::sqrt(2.) : 1.);
      if (candidate < distance[next]) { distance[next] = candidate; parent[next] = current.cell; open.push({candidate, next}); }
    }
  }
  if (parent[target] < 0 && source != target) throw std::runtime_error("no path");
  Path reversed{{goal}};
  for (int cell = target; cell != source; cell = parent[cell]) {
    const auto [x, y] = grid.CellCenter(cell % grid.width(), cell / grid.width());
    reversed.push_back({x, y, 0.});
  }
  reversed.push_back(start);
  std::reverse(reversed.begin(), reversed.end());
  return reversed;
}

}  // namespace navigation2d
