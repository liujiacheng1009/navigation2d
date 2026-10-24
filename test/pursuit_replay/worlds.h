#pragma once
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/costmap/grid_2d.h"
#include "navigation2d/geometry/path_2d.h"
#include "navigation2d/test/temporary_directory.h"
namespace navigation2d::test {
class PursuitWorlds {
 public:
  PursuitWorlds() {
    for (int map = 0; map < 4; ++map) {
      const auto path = directory_.path() / (std::to_string(map) + ".json");
      std::ofstream file(path);
      file << R"({"width":60,"height":60,"resolution":0.05,"cells":[)";
      for (int row = 0; row < 60; ++row) {
        for (int col = 0; col < 60; ++col) {
          const bool occupied = map == 3 || (map == 1 && col >= 30 && row < 24) ||
                                (map == 2 && col == 30 && (row < 24 || row > 34));
          file << (row || col ? "," : "") << (occupied ? 100 : 0);
        }
      }
      file << "]}";
      file.close();
      if (!file) {
        throw std::runtime_error("cannot write pursuit replay map");
      }
      paths_.push_back(path.string());
    }
  }
  Grid2d Load(int index) const { return Grid2d::Load(paths_.at(index)); }

 private:
  TemporaryDirectory directory_;
  std::vector<std::string> paths_;
};
inline std::vector<Path> PursuitPaths() {
  const auto a = MakePose2d(.5, 1.3, 0.);
  const auto b = MakePose2d(1.8, 1.3, 0.);
  const auto c = MakePose2d(1.8, 1.45, 0.);
  return {{a, MakePose2d(2.5, 1.3, 0.)},
          {a, MakePose2d(1.3, 1.3, 0.), MakePose2d(1.3, 2.5, 0.)},
          {a, b, c, MakePose2d(.5, 1.45, 0.)},
          {a, a, b, b, c},
          {a},
          {}};
}
}  // namespace navigation2d::test
