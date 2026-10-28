#include <cassert>
#include <cmath>
#include <iostream>

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/planning/path_smoother.h"
#include "navigation2d/planning/state_lattice_planner.h"
#include "navigation2d/test/planner_test_maps.h"

int main() {
  using namespace navigation2d;
  const auto start = MakePose2d(.6, .6, 0.);
  const auto goal = MakePose2d(3.5, 2.8, .5);
  Path curved{start, MakePose2d(1., .7, 0.), MakePose2d(1.6, .6, 0.), MakePose2d(2.3, .9, 0.),
              MakePose2d(3., 1.4, 0.)};
  for (int scene = 0; scene < 12; ++scene) {
    NavigationConfig config;
    config.robot_radius = .08 + .02 * (scene % 3);
    config.lattice_yaw_bins = scene % 2 ? 16 : 32;
    config.lattice_primitive_length = .1 + .05 * (scene % 3);
    config.lattice_allow_reverse = scene % 2;
    config.lattice_reverse_penalty = 1. + .2 * (scene % 3);
    config.lattice_rotation_cost = .1 + .02 * (scene % 4);
    config.lattice_cost_penalty = .5 + .3 * (scene % 5);
    config.lattice_max_expansions = scene % 4 ? 10000 : 1;
    config.lattice_max_planning_time = 10.;
    config.lattice_initial_heuristic_weight = 1.5 + .5 * (scene % 3);
    config.lattice_heuristic_weight_decrement = .5;
    config.smoother_enabled = scene % 3 != 0;
    config.smoother_max_iterations = 10 + scene;
    config.smoother_data_weight = .1 + .02 * (scene % 3);
    config.smoother_smooth_weight = .1 + .03 * (scene % 4);
    config.smoother_obstacle_weight = .2 + .05 * (scene % 3);
    config.smoother_min_clearance = .15 + .01 * (scene % 4);
    config.smoother_max_deviation = .1 + .01 * (scene % 3);
    config.smoother_max_curvature = 3. + scene % 4;
    config.smoother_max_step = .01 + .005 * (scene % 3);
    config.smoother_tolerance = .0001 * (1 + scene % 3);
    if (scene % 3 == 2) {
      config.footprint = {{-.1, -.08}, {.1, -.08}, {.1, .08}, {-.1, .08}};
    }
    auto map = test::CommonPlannerMap("lattice_configuration");
    StateLatticePlanner planner(config);
    planning_internal::ConstrainedPathSmoother smoother(config);
    const auto radius = config.robot_radius;
    config.lattice_yaw_bins = 7;
    config.smoother_enabled = !config.smoother_enabled;
    config.footprint.clear();
    for (const auto& path :
         {planner.Plan(map, start, goal), smoother.Smooth(curved, map, radius)}) {
      assert(!path.empty());
      std::cout << scene << ' ' << path.size();
      for (const auto& point : path) {
        assert(std::isfinite(X(point)) && std::isfinite(Y(point)) && std::isfinite(Yaw(point)));
        std::cout << ' ' << std::hexfloat << X(point) << ' ' << Y(point) << ' ' << Yaw(point);
      }
      std::cout << '\n';
    }
  }
}
