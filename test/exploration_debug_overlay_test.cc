#include <algorithm>
#include <source_location>
#include <stdexcept>
#include <vector>

#include "navigation2d/application/exploration/debug_overlay.h"

namespace {
void Require(bool condition,
             const std::source_location location = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("debug overlay contract failed at line " +
                             std::to_string(location.line()));
  }
}
}  // namespace

int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  ExplorationGrid grid{20, 20, .1, 10., -5., std::vector<std::int8_t>(400, 0)};
  grid.cells[10 * 20 + 10] = 100;
  grid.cells[10 * 20 + 11] = -1;  // Unknown immediately beside the obstacle.
  grid.cells[0] = -1;
  grid.cells[1] = 50;
  const auto original = grid.cells;
  auto input = grid.cells;
  PaintPlanningDebugInput(grid, MakePose2d(.05, .05, 0.), input);
  Require(grid.cells == original);          // Display evidence cannot modify observation.
  Require(input[0] == 0 && input[1] == 0);  // Under-body display disk.
  Require(input[210] == 100 && input[211] == -1);
  auto inflated = input;
  PaintInflatedDebugMap(grid, input, inflated);
  Require(input[209] == 0 && inflated[209] == 99);
  Require(inflated[210] == 100 && inflated[211] == -1);
  Require(inflated[0] == 0);                         // Beyond inflation support.
  Require(inflated[232] > 0 && inflated[232] < 99);  // Soft decay, no hard wall.
  Require(inflated[252] <= inflated[232]);

  FrontierDebugSnapshot snap;
  Require(SerializeFrontierDebug(snap) == R"({"cells":[],"cluster_goals":[],"goals":[]})");
  snap.cells.push_back({1.25, -2., 7});
  snap.cluster_goals.push_back({1., 2., 3., 4., 7, .5, 8., 9});
  snap.goals = snap.cluster_goals;
  const std::string expected =
      R"({"cells":[[1.250000,-2.000000,7]],"cluster_goals":[{"x":1.000000,"y":2.000000,"fx":3.000000,"fy":4.000000,"cid":7,"score":0.500000,"gain":8.000000,"n":9}],"goals":[{"x":1.000000,"y":2.000000,"fx":3.000000,"fy":4.000000,"cid":7,"score":0.500000,"gain":8.000000,"n":9}]})";
  Require(SerializeFrontierDebug(snap) == expected);
  snap.cells.assign(4001, {1., 2., 0});
  const auto sampled = SerializeFrontierDebug(snap);
  // Original stride is floor(N/4000)+1 above the cap: 4001 -> 2001 cells.
  Require(std::count(sampled.begin(), sampled.end(), '[') == 2004);
}
