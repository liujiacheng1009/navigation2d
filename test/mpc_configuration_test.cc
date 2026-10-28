#include <cassert>
#include <cmath>
#include <iostream>

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/control/mpc_controller.h"
#include "navigation2d/test/planner_test_maps.h"

int main() {
  using namespace navigation2d;
  Path path;
  for (int point = 0; point <= 24; ++point) {
    path.push_back(MakePose2d(.8 + point * .08, .8 + .1 * std::sin(point * .1), 0.));
  }
  for (int scene = 0; scene < 12; ++scene) {
    NavigationConfig config;
    config.mpc_solver = scene % 2 ? "acados" : "mppi";
    // This is a deterministic parameter-ownership regression, not a timing
    // benchmark: budget/deadline fault behavior has separate contract tests.
    config.mpc_deadline = 10.;
    config.guidance_max_candidates = 1 + scene % 2;
    config.guidance_candidate_timeout = .2 + .1 * (scene % 3);
    config.robot_radius = .08 + .02 * (scene % 3);
    config.desired_linear_velocity = .2 + .02 * (scene % 3);
    config.min_approach_velocity = .03 + .01 * (scene % 3);
    config.max_reverse_velocity = .08 + .01 * (scene % 3);
    config.max_angular_velocity = 1.;
    config.control_period = .04 + .02 * (scene % 3);
    config.collision_horizon = .3 + .1 * (scene % 3);
    config.mpc_dynamic_safety_margin = .02 + .01 * (scene % 3);
    config.mpc_dynamic_sigma_scale = 1. + .2 * (scene % 3);
    config.dynamic_prediction_timeout = .2 + .1 * (scene % 3);
    config.mppi_seed = 200 + scene;
    config.mppi_time_steps = 10;
    config.mppi_batch_size = 24;
    auto map = test::CommonPlannerMap("mpc_configuration");
    if (scene % 3 != 0) {
      map.MarkObstacle(1.4, .95);
    }
    MpcController controller(config);
    config.mpc_solver = "changed after construction";
    config.guidance_max_candidates = 0;
    Twist2d current{};
    for (int tick = 0; tick < 4; ++tick) {
      const auto pose = MakePose2d(.8 + .03 * tick, .8, .04 * tick);
      const std::vector<PredictedObstacle> obstacles{{1.4, .5, 0., .1, .08, .01, .02}};
      const auto command = controller.Compute(path, pose, current, map, obstacles);
      const auto diagnostics = controller.Diagnostics();
      assert(std::isfinite(command.linear) && std::isfinite(command.angular));
      std::cout << scene << ' ' << tick << ' ' << std::hexfloat << command.linear << ' '
                << command.angular << ' ' << controller.CollisionImminent(pose, command, map) << ' '
                << static_cast<int>(diagnostics.backend) << ' ' << diagnostics.fallback_level
                << '\n';
      current = command;
    }
  }
}
