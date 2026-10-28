#include <cassert>
#include <fstream>

#include "navigation2d/planning/grid_search.h"
#include "navigation2d/planning/theta_star_planner.h"

int main() {
  // Thin wall: every horizontal Theta* visibility chord across a 1-cell seal
  // must be rejected. Parametric LOS sampling used to skip that seal.
  const char* thin_wall_path = "/tmp/navigation2d_thin_wall_los_test.json";
  {
    std::ofstream thin_output(thin_wall_path);
    constexpr int kW = 40, kH = 40;
    thin_output << R"({"width":40,"height":40,"resolution":0.05,"cells":[)";
    for (int i = 0; i < kW * kH; ++i) {
      const int col = i % kW, row = i / kW;
      // Seal rows 12..34; leave a 0.50 m door at rows 1..10 for the body.
      const bool wall = row == 0 || row == kH - 1 || col == 0 || col == kW - 1 ||
                        (col == 20 && row >= 12 && row <= 34);
      thin_output << (i ? "," : "") << (wall ? 100 : 0);
    }
    thin_output << "]}";
  }
  navigation2d::NavigationConfig thin_config;
  thin_config.robot_radius = .22;
  thin_config.inflation_radius = .30;
  navigation2d::LayeredCostmap thin_map(navigation2d::Grid2d::Load(thin_wall_path), thin_config);
  assert(thin_map.cost(20, 20) == navigation2d::kLethal);
  const auto& thin_grid = thin_map.grid();
  for (int row = 14; row <= 33; ++row) {
    const int from = row * thin_grid.width() + 8;
    const int to = row * thin_grid.width() + 32;
    assert(!navigation2d::planning_internal::SegmentTraversalCost(thin_map, from, to, .22));
  }
  // Start/goal far enough from the seal for a 0.22 m body; the only legal
  // route is through the open door north of the seal.
  const auto [left_x, left_y] = thin_grid.CellCenter(8, 20);
  const auto [right_x, right_y] = thin_grid.CellCenter(32, 20);
  navigation2d::ThetaStarPlanner thin_planner(.22);
  const auto thin_route = thin_planner.Plan(thin_map, navigation2d::MakePose2d(left_x, left_y, 0.),
                                            navigation2d::MakePose2d(right_x, right_y, 0.));
  assert(thin_route.size() > 2);
  bool went_through_door = false;
  for (const auto& sample : thin_route) {
    assert(!thin_map.lethal(navigation2d::X(sample), navigation2d::Y(sample), .22));
    const auto [cx, cy] = thin_grid.ToCell(navigation2d::X(sample), navigation2d::Y(sample));
    assert(!(cx == 20 && cy >= 12 && cy <= 34));
    if (cx == 20 && cy < 12) {
      went_through_door = true;
    }
  }
  assert(went_through_door);
}
