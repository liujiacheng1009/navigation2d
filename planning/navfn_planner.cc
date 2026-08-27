#include "navigation2/planning/navfn_planner.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace navigation2d {
namespace {
struct Entry { double priority; int cell; bool operator>(const Entry& rhs) const { return priority > rhs.priority; } };
constexpr std::array<std::array<int, 2>, 8> kDirections{{
    {{1, 0}}, {{-1, 0}}, {{0, 1}}, {{0, -1}},
    {{1, 1}}, {{1, -1}}, {{-1, 1}}, {{-1, -1}}}};
}

Path NavFnPlanner::Plan(const LayeredCostmap& costmap, const Pose2d& start, const Pose2d& goal) const {
  const Grid2d& grid = costmap.grid();
  const auto [sx, sy] = grid.ToCell(start.x, start.y);
  const auto [gx, gy] = grid.ToCell(goal.x, goal.y);
  if (costmap.lethal(start.x, start.y, clearance_) || costmap.lethal(goal.x, goal.y, clearance_))
    throw std::runtime_error("start or goal is occupied");
  const int size = grid.width() * grid.height();
  const int source = sy * grid.width() + sx, target = gy * grid.width() + gx;
  std::vector<double> distance(size, std::numeric_limits<double>::infinity());
  std::vector<int> parent(size, -1);
  const auto line_of_sight = [&](int from, int to) {
    const int x0 = from % grid.width(), y0 = from / grid.width();
    const int x1 = to % grid.width(), y1 = to / grid.width();
    const int samples = std::max(1, static_cast<int>(
        std::ceil(std::hypot(x1 - x0, y1 - y0) * 2.)));
    for (int sample = 1; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / samples;
      const int x = static_cast<int>(std::lround(x0 + ratio * (x1 - x0)));
      const int y = static_cast<int>(std::lround(y0 + ratio * (y1 - y0)));
      const auto [wx, wy] = grid.CellCenter(x, y);
      if (costmap.lethal(wx, wy, clearance_) || costmap.cost(x, y) > 128) return false;
    }
    return true;
  };
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  distance[source] = 0.;
  open.push({algorithm_ == "dijkstra" ? 0. : std::hypot(gx - sx, gy - sy), source});
  while (!open.empty()) {
    const Entry current = open.top(); open.pop();
    const int current_x = current.cell % grid.width(), current_y = current.cell / grid.width();
    const double heuristic = algorithm_ == "dijkstra" ? 0. : std::hypot(gx - current_x, gy - current_y);
    if (current.priority != distance[current.cell] + heuristic) continue;
    if (current.cell == target) break;
    const int x = current_x, y = current_y;
    for (const auto& d : kDirections) {
      const int nx = x + d[0], ny = y + d[1];
      if (nx < 0 || ny < 0 || nx >= grid.width() || ny >= grid.height()) continue;
      const auto [wx, wy] = grid.CellCenter(nx, ny);
      if (costmap.lethal(wx, wy, clearance_)) continue;
      if (d[0] && d[1]) {
        const auto [ax, ay] = grid.CellCenter(x + d[0], y);
        const auto [bx, by] = grid.CellCenter(x, y + d[1]);
        if (costmap.lethal(ax, ay, clearance_) || costmap.lethal(bx, by, clearance_)) continue;
      }
      const int next = ny * grid.width() + nx;
      const double traversal = 1. + static_cast<double>(costmap.cost(nx, ny)) / 252.;
      int predecessor = current.cell;
      double candidate = distance[current.cell] +
          (d[0] && d[1] ? std::sqrt(2.) : 1.) * traversal;
      if (algorithm_ == "theta_star" && parent[current.cell] >= 0 &&
          line_of_sight(parent[current.cell], next)) {
        predecessor = parent[current.cell];
        const int px = predecessor % grid.width(), py = predecessor / grid.width();
        candidate = distance[predecessor] + std::hypot(nx - px, ny - py) * traversal;
      }
      if (candidate < distance[next]) {
        distance[next] = candidate; parent[next] = predecessor;
        const double next_heuristic = algorithm_ == "dijkstra" ? 0. : std::hypot(gx - nx, gy - ny);
        open.push({candidate + next_heuristic, next});
      }
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
  if (algorithm_ == "theta_star" && reversed.size() > 2) {
    Path smoothed{reversed.front()};
    std::size_t anchor = 0;
    while (anchor + 1 < reversed.size()) {
      std::size_t farthest = anchor + 1;
      for (std::size_t candidate = anchor + 2; candidate < reversed.size(); ++candidate) {
        const double length = std::hypot(reversed[candidate].x - reversed[anchor].x,
                                         reversed[candidate].y - reversed[anchor].y);
        const int samples = std::max(1, static_cast<int>(std::ceil(length / (grid.resolution() * .5))));
        bool visible = true;
        for (int sample = 1; sample <= samples; ++sample) {
          const double ratio = static_cast<double>(sample) / samples;
          const double x = reversed[anchor].x + ratio * (reversed[candidate].x - reversed[anchor].x);
          const double y = reversed[anchor].y + ratio * (reversed[candidate].y - reversed[anchor].y);
          const auto [cell_x, cell_y] = grid.ToCell(x, y);
          if (costmap.lethal(x, y, clearance_) || costmap.cost(cell_x, cell_y) > 128) {
            visible = false; break;
          }
        }
        if (!visible) break;
        farthest = candidate;
      }
      smoothed.push_back(reversed[farthest]); anchor = farthest;
    }
    Path dense{smoothed.front()};
    for (std::size_t i = 1; i < smoothed.size(); ++i) {
      const double length = std::hypot(smoothed[i].x - smoothed[i - 1].x,
                                       smoothed[i].y - smoothed[i - 1].y);
      const int samples = std::max(1, static_cast<int>(std::ceil(length / (2. * grid.resolution()))));
      for (int sample = 1; sample <= samples; ++sample) {
        const double ratio = static_cast<double>(sample) / samples;
        dense.push_back({smoothed[i - 1].x + ratio * (smoothed[i].x - smoothed[i - 1].x),
                         smoothed[i - 1].y + ratio * (smoothed[i].y - smoothed[i - 1].y), 0.});
      }
    }
    dense.back().yaw = goal.yaw;
    return dense;
  }
  return reversed;
}

}  // namespace navigation2d
