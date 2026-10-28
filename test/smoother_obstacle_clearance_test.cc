#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <limits>

#include "navigation2d/planning/collision_checker.h"
#include "navigation2d/planning/path_smoother.h"
#include "navigation2d/planning/theta_star_planner.h"

int main() {
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
  navigation2d::LayeredCostmap speck_map(navigation2d::Grid2d::Load(speck_path), speck_config);
  navigation2d::ThetaStarPlanner speck_planner(.22);
  auto speck_route = speck_planner.Plan(speck_map, navigation2d::MakePose2d(.40, 1.50, 0.),
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
    if (x < 1.20 || x > 2.80) {
      continue;
    }
    speck_mid_y_sum += y;
    ++speck_mid_count;
    const double clearance = speck_field.distance(x, y);
    if (std::hypot(x - kSpeckX, y - kSpeckY) < 1.20) {
      min_clearance_near_speck = std::min(min_clearance_near_speck, clearance);
    }
  }
  assert(speck_mid_count > 0);
  // Prefer the open lane toward room center (y≈1.5) over the speck rim.
  assert(speck_mid_y_sum / speck_mid_count > 1.25);
  assert(std::isfinite(min_clearance_near_speck));
  assert(min_clearance_near_speck >= .45);
}
