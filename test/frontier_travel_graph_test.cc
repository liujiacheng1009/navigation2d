#include "navigation2d/exploration/frontier_travel_graph.h"

#include <cassert>

#include "navigation2d/geometry/robot_geometry.h"

namespace {
using namespace navigation2d;
using namespace navigation2d::frontier_internal;
ExplorationGrid Map() {
  ExplorationGrid grid;
  grid.width = 80;
  grid.height = 60;
  grid.resolution = .05;
  grid.cells.assign(4800, 0);
  return grid;
}
int Index(int col, int row) { return row * 80 + col; }
}  // namespace
int main() {
  using namespace navigation2d::frontier_internal;
  navigation2d::RobotProfile robot;
  robot.planning_radius = .255;
  navigation2d::InstallRobotProfile(robot);
  auto map = Map();
  auto observed = map;
  auto graph = BuildFrontierTravelGraph(map, observed, .30, 1.025, 1.525);
  assert(graph && graph->safe.size() == 4800 && graph->travel_cells.size() == 4800);
  assert(graph->safe[Index(20, 30)] && graph->travel_cells[Index(20, 30)] == 0);
  assert(graph->travel_cells[Index(25, 35)] == 10);
  // Sealed wall: Euclidean proximity must not join disconnected rooms.
  for (int row = 0; row < 60; ++row) {
    map.cells[Index(40, row)] = 100;
  }
  observed = map;
  graph = BuildFrontierTravelGraph(map, observed, .30, 1.025, 1.525);
  assert(graph && graph->travel_cells[Index(60, 30)] == -1);
  // Discovery can see through a narrow discovery gap; it never grants the
  // centre of that gap execution clearance merely because BFS reached it.
  for (int row = 26; row <= 34; ++row) {
    map.cells[Index(40, row)] = 0;
  }
  observed = map;
  graph = BuildFrontierTravelGraph(map, observed, .30, 1.025, 1.525);
  assert(graph && !graph->safe[Index(40, 30)]);
  assert(graph->travel_cells[Index(60, 30)] >= 0);
  // No execution-safe start is an ordinary empty result, before bootstrap.
  map.cells.assign(4800, 100);
  observed = map;
  assert(!BuildFrontierTravelGraph(map, observed, .30, 1.025, 1.525));
  // Bootstrap replaces distances only when raw frontiers are unreachable.
  // It may traverse known free cells beside a jamb, leaving the mask alone.
  map = Map();
  observed = map;
  observed.cells[Index(60, 30)] = -1;
  std::vector<int> distances(4800, -1);
  distances[Index(20, 30)] = 0;
  ApplyBootstrapFrontierTravel(map, observed, {20, 30, 20}, distances);
  assert(distances[Index(59, 30)] == 39);
  const auto unchanged = distances;
  ApplyBootstrapFrontierTravel(map, observed, {20, 30, 20}, distances);
  assert(distances == unchanged);
  // Mature evidence disables the small-map bootstrap fallback.
  map.width = 200;
  map.height = 100;
  map.cells.assign(20000, 0);
  observed = map;
  observed.cells[60 + 30 * 200] = -1;
  distances.assign(20000, -1);
  distances[20 + 30 * 200] = 0;
  ApplyBootstrapFrontierTravel(map, observed, {20, 30, 20}, distances);
  assert(distances[59 + 30 * 200] == -1);
}
