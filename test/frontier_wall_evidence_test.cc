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
  // Occupied speck on a free hall row stays occupied; unknown beyond is not
  // flooded into free for planning.
  navigation2d::ExplorationGrid wide_hall;
  wide_hall.width = 80;
  wide_hall.height = 40;
  wide_hall.resolution = .05;
  wide_hall.cells.assign(static_cast<std::size_t>(wide_hall.width * wide_hall.height), 100);
  const auto wpaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row) {
      for (int col = col0; col < col1; ++col) {
        wide_hall.cells[static_cast<std::size_t>(row * wide_hall.width + col)] = value;
      }
    }
  };
  wpaint(2, 78, 4, 21, 0);      // open hall including the bar row
  wpaint(2, 78, 21, 36, -1);    // room unknown immediately past the bar
  wpaint(30, 44, 20, 21, 100);  // occupied speck on the hall row
  assert(wide_hall.cells[20 * wide_hall.width + 37] == 100);
  assert(wide_hall.cells[24 * wide_hall.width + 37] == -1);
}
