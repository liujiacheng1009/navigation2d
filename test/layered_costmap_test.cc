#include <cassert>
#include <fstream>
#include "navigation2d/costmap/layered_costmap.h"

int main() {
  const char* path = "/tmp/navigation2d_costmap_test.json";
  std::ofstream(path) << R"({"width":20,"height":20,"resolution":0.1,"cells":[)";
  std::ofstream output(path, std::ios::app);
  for (int i = 0; i < 400; ++i) output << (i ? "," : "") << ((i % 20 == 0) ? 255 : 0);
  output << "]}"; output.close();
  navigation2d::NavigationConfig config;
  navigation2d::LayeredCostmap map(navigation2d::Grid2d::Load(path), config);
  assert(map.cost(0, 10) == navigation2d::kLethal);
  map.MarkObstacle(1.0, 1.0);
  assert(map.cost(10, 10) == navigation2d::kLethal);
  assert(map.cost(11, 10) >= navigation2d::kInscribed);
  map.ClearObstacle(1.0, 1.0);
  assert(map.cost(10, 10) != navigation2d::kLethal);
  navigation2d::PointCloud2d cloud{2.0, {{0.5, 0.0}}};
  map.UpdateObstacleLayer(navigation2d::MakePose2d(1.0, 1.0, 0.0), cloud);
  assert(map.cost(15, 10) == navigation2d::kLethal);
  int width, height, x, y;
  const auto window = map.RollingWindow(navigation2d::MakePose2d(1., 1., 0.),
                                        &width, &height, &x, &y);
  assert(static_cast<int>(window.size()) == width * height);
}
