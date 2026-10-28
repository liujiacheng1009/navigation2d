#include <cassert>
#include <cmath>
#include <fstream>

#include "navigation2d/planning/theta_star_planner.h"

int main() {
  // 0.60 m free width (half-clearance 0.30): body radius 0.22 fits; the
  // half-cell planning margin (~0.255) must still find a route where a
  // 0.50 m throat used to reject every start as occupied.
  const char* door_path = "/tmp/navigation2d_door_goal_clearance_test.json";
  {
    std::ofstream door_output(door_path);
    constexpr int kW = 40, kH = 24;
    door_output << R"({"width":40,"height":24,"resolution":0.05,"cells":[)";
    for (int i = 0; i < kW * kH; ++i) {
      const int col = i % kW, row = i / kW;
      // Horizontal corridor rows 6..17 inclusive => 12 cells = 0.60 m.
      const bool wall = row < 6 || row > 17 || col == 0 || col == kW - 1;
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
  assert(planning_clearance < .30);
  navigation2d::ThetaStarPlanner door_planner(planning_clearance);
  const auto door_route = door_planner.Plan(door_map, navigation2d::MakePose2d(.40, .60, 0.),
                                            navigation2d::MakePose2d(1.60, .60, 0.));
  assert(door_route.size() > 2);
}
