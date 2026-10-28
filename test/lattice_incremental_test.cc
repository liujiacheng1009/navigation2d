#include <cassert>
#include <cmath>

#include "navigation2d/planning/collision_checker.h"
#include "navigation2d/planning/obstacle_heuristic.h"
#include "navigation2d/planning/state_lattice_planner.h"
#include "navigation2d/test/planner_test_maps.h"

int main() {
  auto map = navigation2d::test::CommonPlannerMap("lattice");
  const navigation2d::planning_internal::DistanceField field(map);
  navigation2d::NavigationConfig lattice_config;
  lattice_config.map_resolution = .1;
  lattice_config.robot_radius = .12;
  lattice_config.lattice_yaw_bins = 16;
  lattice_config.lattice_primitive_length = .3;
  lattice_config.lattice_max_planning_time = 1.;
  navigation2d::StateLatticePlanner lattice(lattice_config);
  const auto lattice_goal = navigation2d::MakePose2d(4., 3., M_PI_2);
  const auto lattice_path = lattice.Plan(map, navigation2d::MakePose2d(.8, .8, 0.), lattice_goal);
  const auto first_lattice_diagnostics = lattice.Diagnostics();
  assert(!first_lattice_diagnostics.obstacle_heuristic_cache_hit);
  assert(first_lattice_diagnostics.expansions > 0);
  assert(first_lattice_diagnostics.suboptimality_bound >= 1. &&
         first_lattice_diagnostics.suboptimality_bound <= 2.5);
  const auto repeated_lattice_path =
      lattice.Plan(map, navigation2d::MakePose2d(.8, .8, 0.), lattice_goal);
  assert(lattice.Diagnostics().obstacle_heuristic_cache_hit);
  assert(repeated_lattice_path.size() == lattice_path.size());
  assert(lattice_path.size() > 3);
  assert(std::abs(navigation2d::Yaw(lattice_path.back()) - M_PI_2) < 1e-12);
  for (const auto& sample : lattice_path) {
    assert(field.CircleCollisionFree(navigation2d::X(sample), navigation2d::Y(sample),
                                     lattice_config.robot_radius));
  }
  const auto [heuristic_goal_x, heuristic_goal_y] = map.grid().ToCell(4., 3.);
  navigation2d::planning_internal::ObstacleHeuristic obstacle_heuristic(
      map, heuristic_goal_x, heuristic_goal_y, lattice_config.robot_radius, 2.);
  const auto [heuristic_start_x, heuristic_start_y] = map.grid().ToCell(.8, .8);
  assert(std::isfinite(obstacle_heuristic.cost(heuristic_start_x, heuristic_start_y)));
  assert(obstacle_heuristic.Matches(map, heuristic_goal_x, heuristic_goal_y,
                                    lattice_config.robot_radius, 2.));
  map.MarkObstacle(3.5, 3.5);
  assert(!obstacle_heuristic.Matches(map, heuristic_goal_x, heuristic_goal_y,
                                     lattice_config.robot_radius, 2.));
  const auto repaired_lattice_path =
      lattice.Plan(map, navigation2d::MakePose2d(.9, .8, 0.), lattice_goal);
  assert(lattice.Diagnostics().incremental_reuse);
  assert(lattice.Diagnostics().repaired_states > 0);
  const navigation2d::planning_internal::DistanceField repaired_field(map);
  for (const auto& sample : repaired_lattice_path) {
    assert(repaired_field.CircleCollisionFree(navigation2d::X(sample), navigation2d::Y(sample),
                                              lattice_config.robot_radius));
  }
}
