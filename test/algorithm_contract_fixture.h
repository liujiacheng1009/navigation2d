#pragma once
#include <fstream>
#include <stdexcept>

#include "navigation2d/costmap/layered_costmap.h"
#include "navigation2d/test/temporary_directory.h"
namespace navigation2d::test {
struct AlgorithmContractFixture {
  TemporaryDirectory directory;
  LayeredCostmap Map() const {
    const auto path = directory.path() / "map.json";
    std::ofstream file(path);
    file << R"({"width":40,"height":40,"resolution":0.1,"rectangles":[[0,0,40,40,0]]})";
    file.close();
    if (!file) {
      throw std::runtime_error("cannot write contract map");
    }
    NavigationConfig config;
    config.map_resolution = .1;
    return LayeredCostmap(Grid2d::Load(path.string()), config);
  }
};
}  // namespace navigation2d::test
