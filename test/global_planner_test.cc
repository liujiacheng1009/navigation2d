#include <cassert>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/planning/astar_planner.h"
#include "navigation2d/planning/collision_checker.h"
#include "navigation2d/planning/grid_search.h"
#include "navigation2d/planning/search_core.h"
#include "navigation2d/planning/motion_primitives.h"
#include "navigation2d/planning/incremental_search.h"
#include "navigation2d/planning/obstacle_heuristic.h"
#include "navigation2d/planning/path_smoother.h"
#include "navigation2d/planning/state_lattice_planner.h"
#include "navigation2d/planning/theta_star_planner.h"

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
  const auto anytime = navigation2d::planning_internal::AnytimeRepairingAStar(
      5, 0, 4, [](int state) { return state == 4 ? 0. : 1.; },
      [](int state, int, const auto& visit) {
        if (state == 0) { visit({1, 4.}); visit({2, 1.}); }
        if (state == 1) visit({4, 1.});
        if (state == 2) visit({3, 1.});
        if (state == 3) visit({4, 1.});
      }, 2.5, .5);
  assert(anytime.found && anytime.suboptimality_bound == 1.);
  assert(anytime.cost_to_come[4] == 3.);
  std::vector<std::vector<std::pair<int, double>>> incremental_edges{
      {{1, 1.}, {2, 2.}}, {{3, 1.}}, {{3, 1.}}, {}};
  const auto successors = [&](int state, const auto& visit) {
    for (const auto& [next, cost] : incremental_edges[state]) visit({next, cost});
  };
  const auto predecessors = [&](int state, const auto& visit) {
    for (int source = 0; source < 4; ++source)
      for (const auto& [next, cost] : incremental_edges[source])
        if (next == state) visit({source, cost});
  };
  navigation2d::planning_internal::DStarLite incremental(
      4, 3, successors, predecessors,
      [](int, int) { return 0.; });
  auto incremental_result = incremental.Plan(0, {});
  assert(incremental_result.found);
  assert((incremental_result.path == std::vector<int>{0, 1, 3}));
  incremental_edges[1][0].second = 10.;
  incremental_result = incremental.Plan(0, {1});
  assert(incremental_result.found && incremental_result.reused);
  assert((incremental_result.path == std::vector<int>{0, 2, 3}));
  incremental_result = incremental.Plan(2, {});
  assert((incremental_result.path == std::vector<int>{2, 3}));

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
  const auto first_lattice_diagnostics = lattice.Diagnostics();
  assert(!first_lattice_diagnostics.obstacle_heuristic_cache_hit);
  assert(first_lattice_diagnostics.expansions > 0);
  assert(first_lattice_diagnostics.suboptimality_bound >= 1. &&
         first_lattice_diagnostics.suboptimality_bound <= 2.5);
  const auto repeated_lattice_path = lattice.Plan(
      map, navigation2d::MakePose2d(.8, .8, 0.), lattice_goal);
  assert(lattice.Diagnostics().obstacle_heuristic_cache_hit);
  assert(repeated_lattice_path.size() == lattice_path.size());
  assert(lattice_path.size() > 3);
  assert(std::abs(navigation2d::Yaw(lattice_path.back()) - M_PI_2) < 1e-12);
  for (const auto& sample : lattice_path)
    assert(field.CircleCollisionFree(navigation2d::X(sample), navigation2d::Y(sample),
                                     lattice_config.robot_radius));
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
  const auto repaired_lattice_path = lattice.Plan(
      map, navigation2d::MakePose2d(.9, .8, 0.), lattice_goal);
  assert(lattice.Diagnostics().incremental_reuse);
  assert(lattice.Diagnostics().repaired_states > 0);
  const navigation2d::planning_internal::DistanceField repaired_field(map);
  for (const auto& sample : repaired_lattice_path)
    assert(repaired_field.CircleCollisionFree(navigation2d::X(sample), navigation2d::Y(sample),
                                              lattice_config.robot_radius));

  navigation2d::NavigationConfig smoother_config;
  smoother_config.smoother_max_curvature = 100.;
  smoother_config.smoother_min_clearance = .2;
  const navigation2d::Path zigzag{
      navigation2d::MakePose2d(3., 1., 0.),
      navigation2d::MakePose2d(3.2, 1.2, 0.),
      navigation2d::MakePose2d(3.4, 1., 0.)};
  const auto smooth = navigation2d::planning_internal::ConstrainedPathSmoother(
      smoother_config).Smooth(zigzag, map, .18);
  assert(smooth.size() == zigzag.size());
  assert(navigation2d::Y(smooth[1]) < navigation2d::Y(zigzag[1]));
  assert((smooth[1].translation() - zigzag[1].translation()).norm() <=
         smoother_config.smoother_max_deviation + 1e-12);
  assert(navigation2d::X(smooth.front()) == navigation2d::X(zigzag.front()));
  assert(navigation2d::X(smooth.back()) == navigation2d::X(zigzag.back()));

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

  // Open floor ending in unknown. The viewpoint is 0.20 m from the unknown
  // edge, inside the 0.255 m clearance that used to reject every frontier.
  const char* frontier_path = "/tmp/navigation2d_frontier_goal_test.json";
  std::ofstream frontier_output(frontier_path);
  frontier_output << R"({"width":40,"height":20,"resolution":0.05,"cells":[)";
  for (int i = 0; i < 800; ++i) {
    const int col = i % 40;
    frontier_output << (i ? "," : "") << (col >= 30 ? 255 : 0);
  }
  frontier_output << "]}";
  frontier_output.close();
  navigation2d::LayeredCostmap frontier_map(
      navigation2d::Grid2d::Load(frontier_path), config);
  navigation2d::AStarPlanner frontier_planner(.255);
  const auto frontier_start = navigation2d::MakePose2d(.40, .50, 0.);
  const auto frontier_goal = navigation2d::MakePose2d(1.30, .50, 0.);
  const auto frontier_route = frontier_planner.Plan(frontier_map, frontier_start, frontier_goal);
  assert(frontier_route.size() > 2);
  for (const auto& sample : frontier_route) {
    const auto [col, row] = frontier_map.grid().ToCell(
        navigation2d::X(sample), navigation2d::Y(sample));
    assert(frontier_map.cost(col, row) != navigation2d::kUnknown);
  }
  bool goal_rejected = false;
  try {
    frontier_planner.Plan(frontier_map, frontier_start,
                          navigation2d::MakePose2d(1.55, .50, 0.));
  } catch (const std::runtime_error& error) {
    goal_rejected = std::string(error.what()).find("goal occupied") != std::string::npos;
  }
  assert(goal_rejected);

  // Wide corridor: a wall-scraping densified chord must be lifted off the
  // south wall toward open space (soft 0.28 m target, not footprint scrape).
  const char* corridor_path = "/tmp/navigation2d_corridor_clearance_test.json";
  {
    std::ofstream corridor_output(corridor_path);
    constexpr int kW = 60, kH = 40;
    corridor_output << R"({"width":60,"height":40,"resolution":0.05,"cells":[)";
    for (int i = 0; i < kW * kH; ++i) {
      const int col = i % kW, row = i / kW;
      const bool wall = row == 0 || row == kH - 1 || col == 0 || col == kW - 1;
      corridor_output << (i ? "," : "") << (wall ? 100 : 0);
    }
    corridor_output << "]}";
  }
  navigation2d::NavigationConfig corridor_config;
  corridor_config.robot_radius = .22;
  corridor_config.inflation_radius = .30;
  corridor_config.smoother_min_clearance = .28;
  corridor_config.smoother_obstacle_weight = .55;
  corridor_config.smoother_data_weight = .10;
  corridor_config.smoother_max_deviation = .18;
  navigation2d::LayeredCostmap corridor_map(
      navigation2d::Grid2d::Load(corridor_path), corridor_config);
  navigation2d::Path hugging{
      navigation2d::MakePose2d(.50, .26, 0.),
      navigation2d::MakePose2d(1.00, .26, 0.),
      navigation2d::MakePose2d(1.50, .26, 0.),
      navigation2d::MakePose2d(2.00, .26, 0.),
      navigation2d::MakePose2d(2.50, .26, 0.)};
  const auto lifted = navigation2d::planning_internal::ConstrainedPathSmoother(corridor_config)
                          .Smooth(hugging, corridor_map, .22);
  assert(navigation2d::Y(lifted[2]) > navigation2d::Y(hugging[2]) + .02);
  const navigation2d::planning_internal::DistanceField corridor_field(corridor_map);
  assert(corridor_field.distance(navigation2d::X(lifted[2]), navigation2d::Y(lifted[2])) >=
         .275);

  // Clearance-aware traversal: equal-length segment farther from the wall is
  // cheaper, so Theta* prefers the open lane over the inflated edge.
  navigation2d::ThetaStarPlanner theta(.22);
  const auto open_lane = theta.Plan(corridor_map,
                                    navigation2d::MakePose2d(.40, 1.00, 0.),
                                    navigation2d::MakePose2d(2.60, 1.00, 0.));
  double mid_y_sum = 0.;
  int mid_count = 0;
  for (const auto& sample : open_lane) {
    if (navigation2d::X(sample) < .80 || navigation2d::X(sample) > 2.20) continue;
    mid_y_sum += navigation2d::Y(sample);
    ++mid_count;
  }
  assert(mid_count > 0);
  assert(std::abs(mid_y_sum / mid_count - 1.00) < .25);

  // 0.50 m free width (half-clearance 0.25): EffectivePlanningRadius ≈ 0.255
  // used to reject the goal while explorers only require robot_radius 0.22.
  const char* door_path = "/tmp/navigation2d_door_goal_clearance_test.json";
  {
    std::ofstream door_output(door_path);
    constexpr int kW = 40, kH = 20;
    door_output << R"({"width":40,"height":20,"resolution":0.05,"cells":[)";
    for (int i = 0; i < kW * kH; ++i) {
      const int col = i % kW, row = i / kW;
      // Horizontal corridor rows 5..14 inclusive => 10 cells = 0.50 m.
      const bool wall = row < 5 || row > 14 || col == 0 || col == kW - 1;
      door_output << (i ? "," : "") << (wall ? 100 : 0);
    }
    door_output << "]}";
  }
  navigation2d::NavigationConfig door_config;
  door_config.robot_radius = .22;
  door_config.inflation_radius = .30;
  navigation2d::LayeredCostmap door_map(navigation2d::Grid2d::Load(door_path), door_config);
  const double planning_clearance =
      door_config.robot_radius + std::sqrt(.5) * door_map.grid().resolution();
  assert(planning_clearance > .25);
  navigation2d::ThetaStarPlanner door_planner(planning_clearance);
  const auto door_route = door_planner.Plan(
      door_map, navigation2d::MakePose2d(.40, .50, 0.),
      navigation2d::MakePose2d(1.60, .50, 0.));
  assert(door_route.size() > 2);

  // Open room with a single central speck offset from the straight chord.
  // Clearance-aware Theta* must keep mid-path samples away from the speck
  // rim instead of hugging its inflation edge across the room.
  const char* speck_path = "/tmp/navigation2d_open_room_speck_test.json";
  {
    std::ofstream speck_output(speck_path);
    constexpr int kW = 80, kH = 60;
    speck_output << R"({"width":80,"height":60,"resolution":0.05,"cells":[)";
    for (int i = 0; i < kW * kH; ++i) {
      const int col = i % kW, row = i / kW;
      const bool wall = row == 0 || row == kH - 1 || col == 0 || col == kW - 1;
      // Speck near y=1.05 (row 21), x=2.0 — south of the room centerline so a
      // wall-hugging chord along y≈1.05 would scrape inflation.
      const bool speck = col >= 39 && col <= 41 && row >= 20 && row <= 22;
      speck_output << (i ? "," : "") << ((wall || speck) ? 100 : 0);
    }
    speck_output << "]}";
  }
  navigation2d::NavigationConfig speck_config;
  speck_config.robot_radius = .22;
  speck_config.inflation_radius = .30;
  speck_config.smoother_min_clearance = .50;
  speck_config.smoother_obstacle_weight = 1.4;
  speck_config.smoother_data_weight = .10;
  speck_config.smoother_max_deviation = .55;
  navigation2d::LayeredCostmap speck_map(navigation2d::Grid2d::Load(speck_path),
                                         speck_config);
  navigation2d::ThetaStarPlanner speck_planner(.22);
  auto speck_route = speck_planner.Plan(
      speck_map, navigation2d::MakePose2d(.40, 1.50, 0.),
      navigation2d::MakePose2d(3.60, 1.50, 0.));
  speck_route = navigation2d::planning_internal::ConstrainedPathSmoother(speck_config)
                    .Smooth(speck_route, speck_map, .22);
  const navigation2d::planning_internal::DistanceField speck_field(speck_map);
  constexpr double kSpeckX = 2.00, kSpeckY = 1.05;
  double min_clearance_near_speck = std::numeric_limits<double>::infinity();
  double speck_mid_y_sum = 0.;
  int speck_mid_count = 0;
  for (const auto& sample : speck_route) {
    const double x = navigation2d::X(sample);
    const double y = navigation2d::Y(sample);
    if (x < 1.20 || x > 2.80) continue;
    speck_mid_y_sum += y;
    ++speck_mid_count;
    const double clearance = speck_field.distance(x, y);
    if (std::hypot(x - kSpeckX, y - kSpeckY) < 1.20)
      min_clearance_near_speck = std::min(min_clearance_near_speck, clearance);
  }
  assert(speck_mid_count > 0);
  // Prefer the open lane toward room center (y≈1.5) over the speck rim.
  assert(speck_mid_y_sum / speck_mid_count > 1.25);
  assert(std::isfinite(min_clearance_near_speck));
  assert(min_clearance_near_speck >= .45);
}
