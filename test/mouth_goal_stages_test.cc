#include "navigation2d/exploration/mouth_goal_stages.h"

#include <source_location>
#include <stdexcept>
#include <string>
namespace {
void Require(bool value, std::source_location location = std::source_location::current()) {
  if (!value) {
    throw std::runtime_error("mouth goal contract at " + std::to_string(location.line()));
  }
}
navigation2d::ExplorationGoal Goal(double frontier, double gain, int cells, double score) {
  navigation2d::ExplorationGoal goal;
  goal.x = frontier + .5;
  goal.y = goal.frontier_y = 1.;
  goal.frontier_x = frontier;
  goal.information_gain = gain;
  goal.frontier_cells = cells;
  goal.score = score;
  return goal;
}
}  // namespace
int main() {
  using namespace navigation2d;
  using namespace navigation2d::frontier_internal;
  auto a = Goal(.5, 2., 4, 100.);
  auto b = Goal(.7, 3., 2, 1.);
  b.region_id = 73;
  b.yaw = .7;
  b.leftover_approach = true;
  auto distant = Goal(4., 1., 1, 0.);
  auto merged = MergeMouthGoals({a, b, distant});
  Require(merged.size() == 2 && merged[0].x == b.x && merged[1].x == distant.x);
  Require(merged[0].information_gain == 3. && merged[0].frontier_cells == 4);
  Require(merged[0].region_id == 73 && merged[0].yaw == .7 && merged[0].leftover_approach);
  // Equal gain uses cell count before score, retaining the exact winning task.
  auto c = b;
  c.information_gain = 2.;
  c.frontier_cells = 5;
  c.score = -100.;
  Require(MergeMouthGoals({a, c})[0].x == c.x);
  c.frontier_cells = 4;
  c.score = 101.;
  Require(MergeMouthGoals({a, c})[0].x == c.x);
  c = a;
  c.x += 1.5;
  c.frontier_x += 1.5;
  Require(MergeMouthGoals({a, c}).size() == 1);
  c.x += .0001;
  c.frontier_x += .0001;
  Require(MergeMouthGoals({a, c}).size() == 2);
  ExplorationGrid map{60, 30, .1, 0., 0., std::vector<std::int8_t>(1800, 0)};
  const auto unchanged = PlaceMouthGoals(merged, map, map, .1, 0., 1.);
  Require(unchanged[0].x == merged[0].x && unchanged[0].yaw == .7);
  auto approach = Goal(1., 1., 3, 1.);
  approach.x = .8;
  const auto advanced = PlaceMouthGoals({approach}, map, map, .1, .3, 1.);
  Require(advanced.size() == 1 && advanced[0].x > approach.x);
  auto unseen = map;
  unseen.cells.assign(1800, -1);
  const auto held = PlaceMouthGoals({approach}, map, unseen, .1, .3, 1.);
  Require(held[0].x == approach.x && held[0].frontier_x == approach.frontier_x);
}
