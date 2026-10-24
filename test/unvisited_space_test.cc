#include "navigation2d/exploration/unvisited_space.h"

#include <cmath>
#include <limits>
#include <source_location>
#include <stdexcept>
#include <string>
namespace {
void Require(bool condition, std::source_location location = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("unvisited space contract at " + std::to_string(location.line()));
  }
}
}  // namespace
int main() {
  using namespace navigation2d;
  using namespace navigation2d::frontier_internal;
  ExplorationGrid map{20, 20, .1, 0., 0., std::vector<std::int8_t>(400, 0)};
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<std::pair<double, double>> trail{{.55, .55}};
  const auto distance = FreeSpaceDistances(map, trail, inf, 0, nullptr);
  Require(distance[5 * 20 + 5] == 0. && distance[5 * 20 + 6] == .1);
  Require(distance[6 * 20 + 6] == std::sqrt(2.) * .1);
  const auto limited = FreeSpaceDistances(map, trail, .1, 0, nullptr);
  Require(limited[5 * 20 + 6] == .1 && std::isinf(limited[6 * 20 + 6]));
  std::vector<double> ascent(400);
  for (int row = 0; row < 20; ++row) {
    for (int col = 0; col < 20; ++col) {
      ascent[row * 20 + col] = col;
    }
  }
  const auto ascending = FreeSpaceDistances(map, trail, inf, 0, &ascent);
  Require(std::isinf(ascending[5 * 20 + 4]) && ascending[5 * 20 + 6] == .1);
  const std::vector<std::pair<double, double>> seeds{{1.55, .55}};
  const auto goals = UnvisitedSpaceGoalsAt(map, map, 0., trail, .3, .01, seeds, .5);
  Require(goals.size() == 1 && goals[0].information_gain > .01);
  Require(UnvisitedSpaceGoalsAt(map, map, 0., {}, .3, .01, seeds, .5).empty());
  Require(UnvisitedSpaceGoalsAt(map, map, 0., trail, .3, .01, {}, .5).empty());
  // A seed in an observed island cannot authorize an unreachable visit goal.
  auto islands = map;
  for (int row = 0; row < 20; ++row) {
    islands.cells[row * 20 + 10] = 100;
  }
  Require(UnvisitedSpaceGoalsAt(islands, islands, 0., trail, .3, .01, seeds, .5).empty());
}
