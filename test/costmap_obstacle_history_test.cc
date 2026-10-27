#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "navigation2d/costmap/layered_costmap.h"
namespace {
using namespace navigation2d;
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("costmap observation/history contract failed");
  }
}
template <class Values>
void Array(const Values& values) {
  std::cout << '[';
  bool first = true;
  for (const auto value : values) {
    if (!first) {
      std::cout << ',';
    }
    first = false;
    std::cout << +value;
  }
  std::cout << ']';
}
void Snapshot(const LayeredCostmap& map) {
  std::cout << "{\"digest\":" << map.digest() << ",\"revision\":" << map.revision();
  std::vector<int> costs;
  for (int y = -1; y <= map.grid().height(); ++y) {
    for (int x = -1; x <= map.grid().width(); ++x) {
      costs.push_back(map.cost(x, y));
    }
  }
  std::cout << ",\"costs\":";
  Array(costs);
  std::cout << ",\"changes\":[";
  const auto revision = map.revision();
  bool first = true;
  for (auto from : {std::uint64_t{0}, std::uint64_t{1}, revision > 64 ? revision - 64 : 0,
                    revision > 0 ? revision - 1 : 0, revision, revision + 1}) {
    if (!first) {
      std::cout << ',';
    }
    first = false;
    Array(map.ChangedCellsSince(from));
  }
  std::cout << "],\"queries\":[";
  first = true;
  for (auto p : {std::pair{.05, .05}, std::pair{1.15, 1.15}, std::pair{2.15, 1.55},
                 std::pair{3.25, 2.25}, std::pair{-1., 1.}}) {
    if (!first) {
      std::cout << ',';
    }
    first = false;
    const auto distance = map.nearestLethalDistance(p.first, p.second);
    std::cout << '[' << map.lethal(p.first, p.second, .12) << ',';
    if (std::isfinite(distance)) {
      std::cout << distance;
    } else {
      std::cout << "\"inf\"";
    }
    std::cout << ']';
  }
  int width, height, ox, oy;
  auto window = map.RollingWindow(MakePose2d(.15, .15, .3), &width, &height, &ox, &oy);
  std::cout << "],\"window_geometry\":[" << width << ',' << height << ',' << ox << ',' << oy
            << "],\"window\":";
  Array(window);
  std::cout << '}';
}
}  // namespace
int main() {
  std::cout.precision(17);
  const auto path = std::filesystem::temp_directory_path() /
                    ("navigation2d_costmap_contract_" + std::to_string(getpid()) + ".json");
  {
    std::ofstream file(path);
    file << R"({"width":40,"height":30,"resolution":0.1,"cells":[)";
    for (int y = 0; y < 30; ++y) {
      for (int x = 0; x < 40; ++x) {
        if (x || y) {
          file << ',';
        }
        file << (x == 0 || y == 0 ? 254 : (x > 32 ? 255 : 0));
      }
    }
    file << "]}";
  }
  NavigationConfig config;
  config.robot_radius = .12;
  config.inflation_radius = .25;
  config.raytrace_max_range = 1.;
  LayeredCostmap map(Grid2d::Load(path.string()), config);
  std::filesystem::remove(path);
  bool first = true;
  const auto snapshot = [&](const LayeredCostmap& value) {
    if (!first) {
      std::cout << ',';
    }
    first = false;
    Snapshot(value);
  };
  std::cout << "{\"snapshots\":[";
  snapshot(map);
  const auto initial = map.revision();
  map.MarkObstacle(1.55, 1.05);
  Require(map.cost(15, 10) == kLethal && map.revision() > initial);
  snapshot(map);
  const auto marked = map.revision();
  map.MarkObstacle(1.55, 1.05);
  map.MarkObstacle(-10., 0.);
  Require(map.revision() == marked);
  snapshot(map);
  LaserScan clear{0., 1., .05, 1., {std::numeric_limits<double>::infinity()}};
  map.UpdateObstacleLayer(MakePose2d(.55, 1.05, 0.), clear);
  Require(map.cost(15, 10) == kFree);
  snapshot(map);
  LaserScan scan{-.6,
                 .3,
                 .05,
                 2.,
                 {std::numeric_limits<double>::quiet_NaN(), -.1, 0., .75,
                  std::numeric_limits<double>::infinity(), 1.5}};
  map.UpdateObstacleLayer(MakePose2d(1.05, 1.05, .2), scan);
  snapshot(map);
  PointCloud2d cloud{2., {{.4, .1}, {0., 0.}, {3., 0.}, {-.3, .4}}};
  map.UpdateObstacleLayer(MakePose2d(2.05, 1.05, .4), cloud);
  snapshot(map);
  map.MarkObstacle(3.45, 1.55);
  Require(map.cost(34, 15) == kLethal);
  snapshot(map);
  map.ClearObstacle(3.45, 1.55);
  Require(map.cost(34, 15) == kUnknown);
  snapshot(map);
  const auto before_history = map.revision();
  for (int i = 0; i < 70; ++i) {
    if (i % 2) {
      map.ClearObstacle(1.15, 2.15);
    } else {
      map.MarkObstacle(1.15, 2.15);
    }
  }
  Require(map.revision() >= before_history + 70);
  Require(map.ChangedCellsSince(before_history).size() == 1200);
  Require(map.ChangedCellsSince(map.revision()).empty());
  snapshot(map);
  auto independent = map;
  const auto original = map.digest();
  independent.MarkObstacle(2.25, 2.25);
  Require(map.digest() == original && independent.digest() != original);
  snapshot(independent);
  auto moved = std::move(independent);
  moved.ClearObstacle(2.25, 2.25);
  Require(map.digest() == original);
  snapshot(moved);
  snapshot(map);
  std::ofstream(path) << R"({"width":2,"height":2,"resolution":0.1,"cells":[0,0,0,0]})";
  LayeredCostmap empty(Grid2d::Load(path.string()), config);
  std::filesystem::remove(path);
  Require(empty.revision() == 0 && empty.ChangedCellsSince(0).empty());
  snapshot(empty);
  std::cout << "]}\n";
}
