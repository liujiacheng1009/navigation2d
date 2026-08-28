#include "navigation2d/planning/obstacle_heuristic.h"

#include <array>
#include <cmath>
#include <limits>
#include <queue>

namespace navigation2d::planning_internal {
namespace {
struct Entry {
  double cost;
  int cell;
  bool operator>(const Entry& other) const { return cost > other.cost; }
};
constexpr std::array<std::array<int, 2>, 8> kDirections{{
    {{1, 0}}, {{-1, 0}}, {{0, 1}}, {{0, -1}},
    {{1, 1}}, {{1, -1}}, {{-1, 1}}, {{-1, -1}}}};
}  // namespace

ObstacleHeuristic::ObstacleHeuristic(const LayeredCostmap& costmap, int goal_x, int goal_y,
                                     double clearance, double cost_penalty)
    : width_(costmap.grid().width()), height_(costmap.grid().height()),
      goal_x_(goal_x), goal_y_(goal_y), clearance_(clearance),
      cost_penalty_(cost_penalty), digest_(costmap.digest()),
      costs_(width_ * height_, std::numeric_limits<double>::infinity()) {
  if (goal_x < 0 || goal_y < 0 || goal_x >= width_ || goal_y >= height_) return;
  const int goal = goal_y * width_ + goal_x;
  costs_[goal] = 0.;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  open.push({0., goal});
  while (!open.empty()) {
    const Entry current = open.top(); open.pop();
    if (current.cost != costs_[current.cell]) continue;
    const int x = current.cell % width_, y = current.cell / width_;
    for (const auto& direction : kDirections) {
      const int nx = x + direction[0], ny = y + direction[1];
      if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) continue;
      const auto [wx, wy] = costmap.grid().CellCenter(nx, ny);
      if (costmap.lethal(wx, wy, clearance_)) continue;
      const double geometric = direction[0] && direction[1] ? std::sqrt(2.) : 1.;
      const double normalized = static_cast<double>(costmap.cost(nx, ny)) / 252.;
      const double transition = geometric * costmap.grid().resolution() *
                                (1. + cost_penalty_ * normalized);
      const int next = ny * width_ + nx;
      const double candidate = current.cost + transition;
      if (candidate < costs_[next]) {
        costs_[next] = candidate;
        open.push({candidate, next});
      }
    }
  }
}

bool ObstacleHeuristic::Matches(const LayeredCostmap& costmap, int goal_x, int goal_y,
                                double clearance, double cost_penalty) const {
  return digest_ == costmap.digest() && goal_x_ == goal_x && goal_y_ == goal_y &&
         clearance_ == clearance && cost_penalty_ == cost_penalty;
}

double ObstacleHeuristic::cost(int x, int y) const {
  if (x < 0 || y < 0 || x >= width_ || y >= height_)
    return std::numeric_limits<double>::infinity();
  return costs_[y * width_ + x];
}

}  // namespace navigation2d::planning_internal
