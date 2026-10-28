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
  auto config = navigation2d::test::BasicFrontierConfig();
  {
    // Viewpoint 0.21 m past a door-post end: one more scanned cell makes it
    // "goal occupied". Settling moves it clear of the post by one cell.
    navigation2d::FrontierExplorer settle(config);
    navigation2d::ExplorationGrid open;
    open.width = open.height = 100;
    open.resolution = .03;
    open.cells.assign(static_cast<std::size_t>(open.width) * open.height, 0);
    for (int i = 0; i < 100; ++i) {
      open.cells[i] = open.cells[99 * 100 + i] = 100;
      open.cells[i * 100] = open.cells[i * 100 + 99] = 100;
    }
    for (int row = 30; row < 50; ++row) {
      for (int col = 48; col < 52; ++col) {
        open.cells[row * 100 + col] = 100;
      }
    }
    settle.UpdateMap(open);
    navigation2d::ExplorationGoal tight;
    tight.x = 1.50;
    tight.y = 1.50 + .21;
    tight.frontier_x = 1.5;
    tight.frontier_y = 2.4;
    const auto moved = settle.SettleGoal(tight);
    assert(moved);
    assert(moved->y > tight.y);
    assert(std::hypot(moved->x - tight.x, moved->y - tight.y) <= navigation2d::kSwingClearance());
    assert(moved->frontier_x == tight.frontier_x && moved->frontier_y == tight.frontier_y);
    navigation2d::ExplorationGoal roomy = tight;
    roomy.y = 2.40;
    const auto same = settle.SettleGoal(roomy);
    assert(same && same->x == roomy.x && same->y == roomy.y);
  }
}
