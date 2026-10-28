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
  // Pinched kitchen mouth: free speck in a 0.20 m unknown throat. Without
  // OpenPassable flood the speck stays free and unknown past it stays unknown.
  navigation2d::ExplorationGrid pinched;
  pinched.width = 60;
  pinched.height = 40;
  pinched.resolution = .05;
  pinched.cells.assign(static_cast<std::size_t>(pinched.width * pinched.height), 100);
  const auto ppaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row) {
      for (int col = col0; col < col1; ++col) {
        pinched.cells[static_cast<std::size_t>(row * pinched.width + col)] = value;
      }
    }
  };
  ppaint(2, 28, 4, 36, 0);     // hall
  ppaint(28, 48, 18, 22, -1);  // 0.20 m unknown throat (4 cells)
  ppaint(28, 30, 18, 20, 0);   // observed free speck in throat
  ppaint(48, 58, 4, 36, -1);   // kitchen unknown
  assert(pinched.cells[19 * pinched.width + 29] == 0);
  assert(pinched.cells[20 * pinched.width + 40] == -1);
}
