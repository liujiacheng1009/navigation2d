#include "navigation2d/test/ros_loop_room.h"

#include <cassert>
#include <cmath>
#include <numbers>
#include <utility>
using navigation2d::test::RosLoopRoom;
int main() {
  RosLoopRoom clear, obstacle(true);
  assert(clear.Cell(0, 20, false) == 100 && clear.Cell(100, 40, false) == 0);
  assert(clear.Cell(150, 40, false) == -1 && clear.Cell(150, 40, true) == 0);
  assert(obstacle.Cell(82, 50, false) == 100);
  assert(std::abs(clear.Range(1.15, 1.5, 0.) - 2.83) < 1e-12);
  assert(std::abs(obstacle.Range(1.15, 1.5, 0.) - .45) < 1e-12);
  assert(std::abs(obstacle.Range(2.4, 1.5, std::numbers::pi) - .54) < 1e-12);
  assert(std::abs(obstacle.Range(2.4, 1.5, 0.) - 1.58) < 1e-12);
  assert(std::abs(obstacle.Range(1.15, 2.5, 0.) - 2.83) < 1e-12);
  assert(!clear.HitsObstacle(1.72, 1.5, .207));
  assert(!obstacle.HitsObstacle(1.35, 1.5, .207));
  assert(obstacle.HitsObstacle(1.4, 1.5, .207));
  assert(obstacle.HitsObstacle(1.72, 1.5, .207));
  assert(!obstacle.HitsObstacle(1.5, 2.3, .207));
  assert(obstacle.HitsObstacle(1.5, 2.2, .207));
  // Independent occupancy marcher checks the analytic lidar against the map
  // used by planners. All rays and samples are retained, including axis rays.
  for (const auto& room : {clear, obstacle}) {
    for (auto origin : {std::pair{1.15, 1.5}, std::pair{2.8, 1.5}, std::pair{2., 2.5}}) {
      for (int ray = 0; ray < 180; ++ray) {
        const double angle = ray * 2. * std::numbers::pi / 180.;
        double hit = 8.;
        for (double range = 0.; range < 8.; range += .002) {
          const int x =
              static_cast<int>(std::floor((origin.first + range * std::cos(angle)) / .02));
          const int y =
              static_cast<int>(std::floor((origin.second + range * std::sin(angle)) / .02));
          if (room.Cell(x, y, true) == 100) {
            hit = range;
            break;
          }
        }
        assert(std::abs(hit - room.Range(origin.first, origin.second, angle)) < .0021);
      }
    }
  }
}
