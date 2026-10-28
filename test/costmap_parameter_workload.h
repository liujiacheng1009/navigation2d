#pragma once
#include <Eigen/Core>
#include <unistd.h>

#include <bit>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "navigation2d/costmap/layered_costmap.h"

namespace navigation2d::test {
inline NavigationConfig CostmapWorkloadConfig(int variant) {
  NavigationConfig config;
  if (variant & 1) {
    config.robot_radius = .09;
    config.inflation_radius = .32;
    config.inflation_cost_scaling = 3.7;
    config.obstacle_max_range = .45;
    config.raytrace_max_range = .7;
    config.local_window_width = .8;
    config.local_window_height = .6;
  }
  if (variant & 2) {
    config.inflation_radius = .12;
    config.obstacle_max_range = .9;
    config.local_window_width = 2.5;
  }
  return config;
}
inline Grid2d CostmapWorkloadGrid() {
  const auto path = std::filesystem::temp_directory_path() /
                    ("navigation2d_parameter_grid_" + std::to_string(getpid()) + ".json");
  {
    std::ofstream file(path);
    file << R"({"width":16,"height":12,"resolution":0.1,"cells":[)";
    for (int y = 0; y < 12; ++y) {
      for (int x = 0; x < 16; ++x) {
        file << (x || y ? "," : "") << (x == 0 || y == 0 ? 254 : (x > 12 ? 255 : 0));
      }
    }
    file << "]}";
  }
  auto grid = Grid2d::Load(path.string());
  std::filesystem::remove(path);
  return grid;
}
// Original public API only: safe to compile with the frozen core's own headers.
inline std::vector<std::uint64_t> CostmapParameterWorkload(LayeredCostmap& map) {
  std::vector<std::uint64_t> values;
  auto append = [&](const auto& sequence) {
    values.push_back(sequence.size());
    for (auto value : sequence) {
      values.push_back(static_cast<std::uint64_t>(value));
    }
  };
  auto snapshot = [&] {
    values.push_back(map.digest());
    values.push_back(map.revision());
    for (int y = -1; y <= map.grid().height(); ++y) {
      for (int x = -1; x <= map.grid().width(); ++x) {
        values.push_back(map.cost(x, y));
      }
    }
    for (std::uint64_t from : {std::uint64_t{0}, map.revision(), map.revision() + 1}) {
      append(map.ChangedCellsSince(from));
    }
    for (auto point : {Eigen::Vector2d(.65, .65), Eigen::Vector2d(1.35, .65)}) {
      values.push_back(map.lethal(point.x(), point.y(), .12));
      values.push_back(
          std::bit_cast<std::uint64_t>(map.nearestLethalDistance(point.x(), point.y())));
    }
    int width, height, x, y;
    append(map.RollingWindow(MakePose2d(.15, .25, .3), &width, &height, &x, &y));
    for (int value : {width, height, x, y}) {
      values.push_back(static_cast<std::uint64_t>(value));
    }
  };
  snapshot();
  map.MarkObstacle(.85, .65);
  snapshot();
  map.MarkObstacle(.85, .65);
  map.ClearObstacle(-1., -1.);
  snapshot();
  map.UpdateObstacleLayer(MakePose2d(.35, .65, 0.),
                          LaserScan{0., .2, .05, 2., {std::numeric_limits<double>::infinity()}});
  snapshot();
  map.UpdateObstacleLayer(MakePose2d(.35, .65, .2), PointCloud2d{2., {{.3, 0.}, {.8, 0.}}});
  snapshot();
  map.ClearObstacle(.85, .65);
  snapshot();
  return values;
}
}  // namespace navigation2d::test
