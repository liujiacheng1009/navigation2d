#include "navigation2d/control/acados_stage_parameters.h"

#include <cassert>
#include <cmath>
#include <limits>
#include <span>
#include <vector>

int main() {
  using namespace navigation2d;
  using namespace navigation2d::control_internal;
  const AcadosObstacleParameters config{.2, .2, .03, 2.};
  const auto empty = BuildAcadosStageParameters<46>({{1., 2.}, .3, .4, 0.}, {}, {}, config);
  assert(empty[0] == 1. && empty[1] == 2. && empty[2] == .3 && empty[3] == .4);
  assert(empty[4] == 1e6 && empty[5] == 1e6 && empty[6] == 1. && empty[8] == 0.);
  assert(empty[9] == 1e6 && empty[13] == 0.);
  for (std::size_t i = 14; i < empty.size(); ++i) {
    assert(empty[i] == 0.);
  }

  // The approaching obstacle is farther away now but crosses the reference.
  // Negative/stale/nonfinite ages are excluded; exactly-at-timeout is valid.
  const std::vector<PredictedObstacle> obstacles{
      {.5, .5, 0., 0., .1, 0., 0., 0.},
      {2., 1., -1., -.5, .05, .01, .02, .2},
      {0., 0., 0., 0., .9, 0., 0., -.01},
      {0., 0., 0., 0., .9, 0., 0., .201},
      {0., 0., 0., 0., .9, 0., 0., std::numeric_limits<double>::quiet_NaN()}};
  std::vector<Halfspace2d> walls;
  for (int i = 0; i < 10; ++i) {
    walls.push_back({{1., -1.}, 3. + i});
  }
  const auto stage =
      BuildAcadosStageParameters<46>({{0., 0.}, 0., 0., .4}, obstacles, walls, config);
  assert(std::abs(stage[4] - 1.4) < 1e-12 && std::abs(stage[5] - .7) < 1e-12);
  assert(std::abs(stage[6] - .30) < 1e-12 && std::abs(stage[7] - .32) < 1e-12);
  assert(stage[8] == 1. && stage[9] == .5 && stage[10] == .5 && stage[13] == 1.);
  assert(stage[14] == 1. && stage[15] == -1. && stage[16] == 3. && stage[17] == 1.);
  assert(stage[42] == 1. && stage[43] == -1. && stage[44] == 10. && stage[45] == 1.);
  const auto capped = BuildAcadosStageParameters<41>({{0., 0.}, 0., 0., .4}, obstacles, {}, config);
  assert(capped[4] == stage[4] && capped[5] == stage[5] && capped[8] == 1.);
  for (std::size_t i = 9; i < capped.size(); ++i) {
    assert(capped[i] == 0.);
  }
  // A single partial corridor must leave the remaining planes disabled.
  const auto partial = BuildAcadosStageParameters<41>(
      {{0., 0.}, 0., 0., 0.}, {}, std::span<const Halfspace2d>(walls).first(1), config);
  assert(partial[9] == 1. && partial[10] == -1. && partial[11] == 3. && partial[12] == 1.);
  for (std::size_t i = 13; i < partial.size(); ++i) {
    assert(partial[i] == 0.);
  }
}
