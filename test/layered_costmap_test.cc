#include <cassert>
#include <fstream>
#include <limits>
#include <algorithm>
#include "navigation2d/costmap/layered_costmap.h"

int main() {
  const char* path = "/tmp/navigation2d_costmap_test.json";
  std::ofstream(path) << R"({"width":20,"height":20,"resolution":0.1,"cells":[)";
  std::ofstream output(path, std::ios::app);
  for (int i = 0; i < 400; ++i) output << (i ? "," : "") << ((i % 20 == 0) ? 254 : 0);
  output << "]}"; output.close();
  navigation2d::NavigationConfig config;
  navigation2d::LayeredCostmap map(navigation2d::Grid2d::Load(path), config);
  const auto initial_revision = map.revision();
  assert(map.cost(0, 10) == navigation2d::kLethal);
  map.MarkObstacle(1.0, 1.0);
  assert(map.revision() > initial_revision);
  const auto changed = map.ChangedCellsSince(initial_revision);
  assert(!changed.empty());
  assert(std::find(changed.begin(), changed.end(), 10 * 20 + 10) != changed.end());
  assert(map.cost(10, 10) == navigation2d::kLethal);
  assert(map.cost(11, 10) >= navigation2d::kInscribed);
  map.ClearObstacle(1.0, 1.0);
  assert(map.cost(10, 10) != navigation2d::kLethal);
  navigation2d::PointCloud2d cloud{2.0, {{0.5, 0.0}}};
  map.UpdateObstacleLayer(navigation2d::MakePose2d(1.0, 1.0, 0.0), cloud);
  assert(map.cost(15, 10) == navigation2d::kLethal);
  // No-return laser beams must clear all observed cells, including the endpoint;
  // otherwise a previously observed dynamic obstacle becomes a persistent ghost.
  navigation2d::NavigationConfig raytrace_config;
  raytrace_config.raytrace_max_range = 1.0;
  navigation2d::LayeredCostmap raytrace_map(navigation2d::Grid2d::Load(path), raytrace_config);
  raytrace_map.MarkObstacle(1.5, 1.0);
  navigation2d::LaserScan no_return_scan{0., 1., .05, 1.,
      {std::numeric_limits<double>::infinity()}};
  raytrace_map.UpdateObstacleLayer(navigation2d::MakePose2d(.5, 1., 0.), no_return_scan);
  assert(raytrace_map.cost(15, 10) == navigation2d::kFree);
  int width, height, x, y;
  const auto window = map.RollingWindow(navigation2d::MakePose2d(1., 1., 0.),
                                        &width, &height, &x, &y);
  assert(static_cast<int>(window.size()) == width * height);

  const char* unknown_path = "/tmp/navigation2d_unknown_test.json";
  std::ofstream unknown_output(unknown_path);
  unknown_output << R"({"width":20,"height":10,"resolution":0.05,"cells":[)";
  for (int i = 0; i < 200; ++i) {
    const int col = i % 20;
    const int value = col >= 16 ? 255 : (col == 2 && i / 20 == 2 ? 254 : 0);
    unknown_output << (i ? "," : "") << value;
  }
  unknown_output << "]}";
  unknown_output.close();
  navigation2d::LayeredCostmap unknown_map(
      navigation2d::Grid2d::Load(unknown_path), config);
  assert(unknown_map.cost(18, 4) == navigation2d::kUnknown);
  assert(unknown_map.cost(2, 2) == navigation2d::kLethal);
  // Unknown starts at x = 0.80. A pose 0.20 m in front of that edge is where
  // a frontier viewpoint sits, inside the 0.255 m planning radius of a wall
  // and still a valid goal.
  assert(!unknown_map.lethal(.60, .25, .255));
  assert(unknown_map.lethal(.125, .125, .22));
}
