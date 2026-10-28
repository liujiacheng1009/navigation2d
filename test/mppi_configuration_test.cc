#include <cassert>
#include <cmath>
#include <iostream>

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/control/dynamic_safety.h"
#include "navigation2d/control/mppi_controller.h"
#include "navigation2d/test/planner_test_maps.h"

int main() {
  using namespace navigation2d;
  Path path;
  for (int point = 0; point <= 30; ++point) {
    path.push_back(MakePose2d(.8 + point * .08, .8 + .15 * std::sin(point * .1), 0.));
  }
  for (int scene = 0; scene < 12; ++scene) {
    NavigationConfig config;
    config.robot_radius = .08 + .02 * (scene % 3);
    config.desired_linear_velocity = .16 + .02 * (scene % 4);
    config.max_reverse_velocity = .05 + .01 * (scene % 3);
    config.max_angular_velocity = .4 + .1 * (scene % 4);
    config.max_linear_acceleration = .3 + .1 * (scene % 3);
    config.max_angular_acceleration = .6 + .2 * (scene % 4);
    config.control_period = .04 + .02 * (scene % 3);
    config.collision_horizon = .3 + .1 * (scene % 3);
    config.mppi_seed = 100 + scene;
    config.mppi_time_steps = 8 + scene % 5;
    config.mppi_batch_size = 16 + 8 * (scene % 4);
    config.mppi_iterations = 1 + scene % 2;
    config.mppi_temperature = .2 + .1 * (scene % 3);
    config.mppi_vx_std = .06 + .01 * (scene % 4);
    config.mppi_wz_std = .2 + .05 * (scene % 3);
    config.mppi_gamma = .01 + .002 * (scene % 4);
    config.mppi_cost_weight = 2. + .2 * (scene % 3);
    config.mppi_goal_weight = 3. + scene % 4;
    config.mppi_goal_angle_weight = 1. + .1 * (scene % 3);
    config.mppi_path_align_weight = 8. + scene % 3;
    config.mppi_path_follow_weight = 3. + scene % 4;
    config.mppi_path_angle_weight = 1. + .2 * (scene % 3);
    config.mppi_prefer_forward_weight = 2. + scene % 3;
    config.mppi_smoothness_weight = .5 + .1 * (scene % 4);
    config.mpc_dynamic_safety_margin = .02 + .01 * (scene % 3);
    config.mpc_dynamic_sigma_scale = 1. + .3 * (scene % 4);
    config.dynamic_prediction_timeout = .2 + .1 * (scene % 3);
    auto map = test::CommonPlannerMap("mppi_configuration");
    if (scene % 3 != 0) {
      map.MarkObstacle(1.4, .95);
    }
    MppiController controller(config);
    const auto limits = config;
    config.mppi_seed = 999;
    config.mppi_time_steps = 1;
    Twist2d current{};
    for (int tick = 0; tick < 6; ++tick) {
      const auto pose = MakePose2d(.8 + .03 * tick, .8, .04 * tick);
      std::vector<PredictedObstacle> obstacles{{1.2, .5, 0., .3, .08, .01, .02}};
      obstacles.front().age_s = .1 * (scene % 5 - 1);
      const auto command = controller.Compute(path, pose, current, map, obstacles);
      assert(std::isfinite(command.linear) && std::isfinite(command.angular));
      assert(command.linear >= -limits.max_reverse_velocity);
      assert(command.linear <= limits.desired_linear_velocity);
      assert(std::abs(command.angular) <= limits.max_angular_velocity);
      std::cout << scene << ' ' << tick << ' ' << std::hexfloat << command.linear << ' '
                << command.angular << ' ' << controller.CollisionImminent(pose, command, map) << ' '
                << MinimumDynamicTtc(pose, {.2, .1}, obstacles, limits) << '\n';
      current = command;
      // Changed geometry resets the warm start at exactly the existing point.
      if (tick == 2) {
        path.back() = MakePose2d(3.2, .9 + scene * .001, 0.);
      }
    }
  }
}
