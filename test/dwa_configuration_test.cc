#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/control/dwa_controller.h"
#include "navigation2d/test/planner_test_maps.h"

int main() {
  using namespace navigation2d;
  const auto pose = MakePose2d(1., 1., .15);
  Path path;
  for (int point = 0; point <= 20; ++point) {
    path.push_back(MakePose2d(1. + point * .08, 1. + .2 * std::sin(point * .1), 0.));
  }
  for (int scene = 0; scene < 48; ++scene) {
    NavigationConfig config;
    config.robot_radius = .08 + .02 * (scene % 4);
    config.desired_linear_velocity = .15 + .03 * (scene % 5);
    config.max_angular_velocity = .4 + .1 * (scene % 6);
    config.max_linear_acceleration = .2 + .05 * (scene % 7);
    config.max_angular_acceleration = .5 + .1 * (scene % 4);
    config.collision_horizon = .2 + .1 * (scene % 5);
    config.control_period = .04 + .01 * (scene % 3);
    config.dwa_horizon = .3 + .1 * (scene % 7);
    config.dwa_linear_samples = 1 + scene % 5;
    config.dwa_angular_samples = 1 + scene % 9;
    config.dwa_path_weight = .5 + .2 * (scene % 3);
    config.dwa_goal_weight = .4 + .2 * (scene % 4);
    config.dwa_obstacle_weight = 1. + .3 * (scene % 5);
    config.dwa_velocity_weight = .1 + .1 * (scene % 6);
    auto map = test::CommonPlannerMap("dwa_configuration");
    if (scene % 3 != 0) {
      map.MarkObstacle(1.15 + .1 * (scene % 4), 1.05);
    }
    DwaController controller(config);
    const Twist2d current{.02 * (scene % 4), .04 * (scene % 5 - 2)};
    const auto command = controller.Compute(path, pose, current, map);
    assert(std::isfinite(command.linear) && std::isfinite(command.angular));
    assert(command.linear >= 0. && command.linear <= config.desired_linear_velocity);
    assert(std::abs(command.angular) <= config.max_angular_velocity);
    const bool collision = controller.CollisionImminent(pose, {.25, .1}, map);
    // Construction takes a value snapshot, including when the caller later
    // changes unrelated algorithm settings or the DWA settings themselves.
    config.dwa_horizon = 9.;
    config.mppi_batch_size = 3;
    const auto repeated = controller.Compute(path, pose, current, map);
    assert(command.linear == repeated.linear && command.angular == repeated.angular);
    assert(collision == controller.CollisionImminent(pose, {.25, .1}, map));
    std::cout << scene << ' ' << std::hexfloat << command.linear << ' ' << command.angular << ' '
              << collision << '\n';
  }
}
