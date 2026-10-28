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
  // Wall-tip thrash: abandon only after repeated live-occupied cuts. A single
  // low-clearance graze near a passable door must replan, not leave the room.
  assert(!navigation2d::PreferTipAbandonAfterPathCut(0, .30));
  assert(!navigation2d::PreferTipAbandonAfterPathCut(1, .30));
  assert(navigation2d::PreferTipAbandonAfterPathCut(2, .30));
  assert(!navigation2d::PreferTipAbandonAfterPathCut(1, .08));  // tip graze → replan
  assert(!navigation2d::PreferTipAbandonAfterPathCut(0, .10));
  assert(!navigation2d::PreferTipAbandonAfterPathCut(0, 0.));
}
