#include <cassert>
#include <cmath>
#include <fstream>

#include "navigation2d/planning/astar_planner.h"
#include "navigation2d/planning/grid_search.h"

int main() {
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
