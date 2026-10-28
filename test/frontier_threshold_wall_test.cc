#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/test/frontier_test_scenes.h"

int main() {
  // PathCutsLiveOccupied: entry must reject through-wall chords even when the
  // robot stands on the threshold (old 0.35 m under-robot disk hid them).
  {
    navigation2d::ExplorationGrid doorway;
    doorway.width = 40;
    doorway.height = 40;
    doorway.resolution = .05;
    doorway.origin_x = 0.;
    doorway.origin_y = 0.;
    doorway.cells.assign(static_cast<std::size_t>(doorway.width * doorway.height), 100);
    const auto dpaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row) {
        for (int col = col0; col < col1; ++col) {
          doorway.cells[static_cast<std::size_t>(row * doorway.width + col)] = value;
        }
      }
    };
    // South hall free; north pocket free; full-width structural wall at row 20.
    dpaint(4, 36, 4, 20, 0);
    dpaint(4, 36, 21, 36, 0);
    dpaint(4, 36, 20, 21, 100);  // full-width structural bar

    const double robot_x = (20 + .5) * doorway.resolution;  // 1.025
    const double robot_y = (18 + .5) * doorway.resolution;  // 0.925
    // Short GOAL–CARROT chord straight through the wall (~0.30 m), entirely
    // inside the old 0.35 m exemption disk around the robot.
    const std::vector<std::pair<double, double>> through_wall = {
        {robot_x, robot_y},
        {robot_x, (22 + .5) * doorway.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(doorway, through_wall, robot_x, robot_y));
    // Same chord with the old loose under-robot radius would miss the wall —
    // prove the default 0.22 m still catches it, and that 0.35 m would not
    // be an acceptable production default.
    assert(navigation2d::PathCutsLiveOccupied(doorway, through_wall, robot_x, robot_y,
                                              navigation2d::kPlanningRadius(),
                                              navigation2d::kPlanningRadius()));
    assert(!navigation2d::PathCutsLiveOccupied(doorway, through_wall, robot_x, robot_y,
                                               navigation2d::kPlanningRadius(), .40));

    // Path that goes around the wall stays legal.
    const std::vector<std::pair<double, double>> around = {
        {robot_x, robot_y},
        {(8 + .5) * doorway.resolution, robot_y},
        {(8 + .5) * doorway.resolution, (24 + .5) * doorway.resolution},
        {robot_x, (24 + .5) * doorway.resolution},
    };
    assert(!navigation2d::PathCutsLiveOccupied(doorway, around, robot_x, robot_y));
  }
}
