#include <cassert>
#include <fstream>
#include <stdexcept>
#include <string>

#include "navigation2d/planning/astar_planner.h"

int main() {
  navigation2d::NavigationConfig config;
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
  navigation2d::LayeredCostmap frontier_map(navigation2d::Grid2d::Load(frontier_path), config);
  navigation2d::AStarPlanner frontier_planner(.255);
  const auto frontier_start = navigation2d::MakePose2d(.40, .50, 0.);
  const auto frontier_goal = navigation2d::MakePose2d(1.30, .50, 0.);
  const auto frontier_route = frontier_planner.Plan(frontier_map, frontier_start, frontier_goal);
  assert(frontier_route.size() > 2);
  for (const auto& sample : frontier_route) {
    const auto [col, row] =
        frontier_map.grid().ToCell(navigation2d::X(sample), navigation2d::Y(sample));
    assert(frontier_map.cost(col, row) != navigation2d::kUnknown);
  }
  bool goal_rejected = false;
  try {
    frontier_planner.Plan(frontier_map, frontier_start, navigation2d::MakePose2d(1.55, .50, 0.));
  } catch (const std::runtime_error& error) {
    goal_rejected = std::string(error.what()).find("goal occupied") != std::string::npos;
  }
  assert(goal_rejected);
}
