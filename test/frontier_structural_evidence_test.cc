#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/test/frontier_test_scenes.h"

int main() {
  // Structural free↔free wall stays occupied (no seal carve).
  {
    navigation2d::ExplorationGrid structural;
    structural.width = 60;
    structural.height = 40;
    structural.resolution = .05;
    structural.cells.assign(static_cast<std::size_t>(structural.width * structural.height), 100);
    const auto spaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row) {
        for (int col = col0; col < col1; ++col) {
          structural.cells[static_cast<std::size_t>(row * structural.width + col)] = value;
        }
      }
    };
    spaint(2, 58, 4, 18, 0);     // south room free
    spaint(2, 58, 19, 30, 0);    // north room free (wall at row 18)
    spaint(2, 58, 18, 19, 100);  // continuous structural wall
    spaint(2, 58, 30, 38, -1);   // unknown only deep past free north room
    assert(structural.cells[18 * structural.width + 30] == 100);
  }
}
