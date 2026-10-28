#pragma once

#include <fstream>
#include <string>
#include <utility>

#include "navigation2d/costmap/layered_costmap.h"

namespace navigation2d::test {
// Each independent process uses a distinct file. Geometry and obstacle update
// order match the original combined test, including the post-lattice revision.
inline LayeredCostmap CommonPlannerMap(const std::string& name, bool repaired = false) {
  const std::string path = "/tmp/navigation2d_" + name + "_common.json";
  std::ofstream output(path);
  output << R"({"width":50,"height":40,"resolution":0.1,"cells":[)";
  for (int index = 0; index < 2000; ++index) {
    output << (index ? ",0" : "0");
  }
  output << "]}";
  output.close();
  NavigationConfig config;
  LayeredCostmap map(Grid2d::Load(path), config);
  map.MarkObstacle(2., 2.);
  if (repaired) {
    map.MarkObstacle(3.5, 3.5);
  }
  return map;
}
inline std::pair<NavigationConfig, LayeredCostmap> CorridorPlannerMap(
    const std::string& name = "theta_clearance") {
  // Wide corridor: a wall-scraping densified chord must be lifted off the
  // south wall toward open space (soft 0.28 m target, not footprint scrape).
  const std::string corridor_path = "/tmp/navigation2d_" + name + "_corridor.json";
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
  navigation2d::LayeredCostmap corridor_map(navigation2d::Grid2d::Load(corridor_path),
                                            corridor_config);
  return {corridor_config, std::move(corridor_map)};
}
}  // namespace navigation2d::test
