#include <cassert>
#include <cmath>
#include <fstream>
#include <vector>

#include "navigation2d/planning/astar_planner.h"
#include "navigation2d/planning/collision_checker.h"
#include "navigation2d/planning/grid_search.h"
#include "navigation2d/planning/search_core.h"
#include "navigation2d/planning/motion_primitives.h"
#include "navigation2d/planning/state_lattice_planner.h"

int main() {
  using navigation2d::planning_internal::BestFirstSearch;
  using navigation2d::planning_internal::RestoreStatePath;
  using navigation2d::planning_internal::SearchOptions;
  // The generic core is independent of grid geometry and deterministically
  // chooses state 1 when two equal-cost branches are inserted in that order.
  const auto graph = BestFirstSearch(
      4, 0, [](int state) { return state == 3; }, [](int) { return 0.; },
      [](int state, int, const auto& visit) {
        if (state == 0) { visit({1, 1.}); visit({2, 1.}); }
        if (state == 1 || state == 2) visit({3, 1.});
      });
  assert(graph.found);
  assert((RestoreStatePath(graph, 0, 3) == std::vector<int>{0, 1, 3}));
  SearchOptions limited;
  limited.max_expansions = 1;
  const auto truncated = BestFirstSearch(
      4, 0, [](int state) { return state == 3; }, [](int) { return 0.; },
      [](int state, int, const auto& visit) { if (state == 0) visit({1, 1.}); }, limited);
  assert(!truncated.found && truncated.diagnostics.expansion_limit_reached);

  const char* map_path = "/tmp/navigation2d_global_planner_test.json";
  std::ofstream output(map_path);
  output << R"({"width":50,"height":40,"resolution":0.1,"cells":[)";
  for (int index = 0; index < 2000; ++index) output << (index ? ",0" : "0");
  output << "]}";
  output.close();

  navigation2d::NavigationConfig config;
  navigation2d::LayeredCostmap map(navigation2d::Grid2d::Load(map_path), config);
  map.MarkObstacle(2., 2.);
  assert(!map.lethal(2.26, 2.05, 0.));
  assert(map.lethal(2.26, 2.05, .22));
  const navigation2d::planning_internal::DistanceField field(map);
  assert(field.distance(2., 2.) == 0.);
  assert(std::abs(field.distance(2.5, 2.) - .5) < .11);
  assert(!field.CircleCollisionFree(2.26, 2.05, .22));
  assert(field.CircleCollisionFree(3., 3., .22));

  const std::vector<Eigen::Vector2d> rectangle{
      {-.30, -.12}, {.30, -.12}, {.30, .12}, {-.30, .12}};
  const navigation2d::planning_internal::FootprintLookup footprint(rectangle, 16, .1);
  assert(footprint.CollisionFree(map, navigation2d::MakePose2d(3., 3., 0.)));
  assert(!footprint.CollisionFree(map, navigation2d::MakePose2d(2.25, 2., 0.)));
  assert(!footprint.SweptCollisionFree(map, {
      navigation2d::MakePose2d(3., 3., 0.),
      navigation2d::MakePose2d(2.25, 2., 0.)}));

  const navigation2d::planning_internal::DifferentialDrivePrimitiveSet primitives(
      16, .1, .3, true);
  for (int bin = 0; bin < primitives.yaw_bins(); ++bin) {
    const auto& motions = primitives.FromYawBin(bin);
    assert(motions.size() == 10);
    bool has_forward = false, has_reverse = false, has_rotate = false;
    for (const auto& motion : motions) {
      assert(!motion.samples.empty());
      assert(motion.samples.back().translation().isApprox(
          Eigen::Vector2d(motion.dx_cells * .1, motion.dy_cells * .1), 1e-12));
      has_forward = has_forward || motion.type == navigation2d::planning_internal::MotionType::kForward;
      has_reverse = has_reverse || motion.type == navigation2d::planning_internal::MotionType::kReverse;
      has_rotate = has_rotate || motion.type == navigation2d::planning_internal::MotionType::kRotate;
    }
    assert(has_forward && has_reverse && has_rotate);
  }

  navigation2d::NavigationConfig lattice_config;
  lattice_config.map_resolution = .1;
  lattice_config.robot_radius = .12;
  lattice_config.lattice_yaw_bins = 16;
  lattice_config.lattice_primitive_length = .3;
  lattice_config.lattice_max_planning_time = 1.;
  navigation2d::StateLatticePlanner lattice(lattice_config);
  const auto lattice_goal = navigation2d::MakePose2d(4., 3., M_PI_2);
  const auto lattice_path = lattice.Plan(
      map, navigation2d::MakePose2d(.8, .8, 0.), lattice_goal);
  assert(lattice_path.size() > 3);
  assert(std::abs(navigation2d::Yaw(lattice_path.back()) - M_PI_2) < 1e-12);
  for (const auto& sample : lattice_path)
    assert(field.CircleCollisionFree(navigation2d::X(sample), navigation2d::Y(sample),
                                     lattice_config.robot_radius));

  const auto& grid = map.grid();
  const auto near_start = grid.ToCell(1.5, 2.3);
  const auto near_end = grid.ToCell(2.5, 2.3);
  const auto far_start = grid.ToCell(1.5, 3.0);
  const auto far_end = grid.ToCell(2.5, 3.0);
  const int near_from = near_start.second * grid.width() + near_start.first;
  const int near_to = near_end.second * grid.width() + near_end.first;
  const int far_from = far_start.second * grid.width() + far_start.first;
  const int far_to = far_end.second * grid.width() + far_end.first;
  const auto near_cost = navigation2d::planning_internal::SegmentTraversalCost(
      map, near_from, near_to, 0.);
  const auto far_cost = navigation2d::planning_internal::SegmentTraversalCost(
      map, far_from, far_to, 0.);
  assert(near_cost && far_cost && *near_cost > *far_cost);

  navigation2d::AStarPlanner planner(config.robot_radius);
  const double goal_yaw = 1.1;
  const auto path = planner.Plan(map, navigation2d::MakePose2d(.8, .8, 0.),
                                navigation2d::MakePose2d(4., 3., goal_yaw));
  assert(path.size() > 2);
  assert(std::abs(navigation2d::Yaw(path.front())) > 1e-3);
  assert(std::abs(navigation2d::Yaw(path.back()) - goal_yaw) < 1e-12);
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const auto delta = path[index + 1].translation() - path[index].translation();
    if (delta.norm() <= 1e-9) continue;
    const double tangent = std::atan2(delta.y(), delta.x());
    const double error = std::atan2(std::sin(navigation2d::Yaw(path[index]) - tangent),
                                    std::cos(navigation2d::Yaw(path[index]) - tangent));
    assert(std::abs(error) < 1e-9);
  }
}
