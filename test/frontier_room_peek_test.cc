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
  // Doorway peek: the frontier steps farther than the 0.35 m association
  // disk, but a room of unknown is still beyond the mouth. That viewpoint
  // is not done.
  {
    navigation2d::FrontierExplorerConfig room_config;
    room_config.minimum_frontier_cells = 4;
    room_config.footprint_clearance = navigation2d::kPlanningRadius();
    room_config.required_frontier_observations = 1;
    navigation2d::ExplorationGrid room;
    room.width = 80;
    room.height = 70;
    room.resolution = .05;
    room.cells.assign(static_cast<std::size_t>(room.width * room.height), 100);
    const auto paint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row) {
        for (int col = col0; col < col1; ++col) {
          room.cells[static_cast<std::size_t>(row * room.width + col)] = value;
        }
      }
    };
    paint(8, 70, 8, 30, 0);     // hall
    paint(24, 56, 30, 40, 0);   // 1.6 m throat, peeked 0.50 m into the room
    paint(16, 64, 40, 62, -1);  // room still unknown
    navigation2d::FrontierExplorer rooms(room_config);
    rooms.UpdateMap(room);
    navigation2d::ExplorationGoal mouth;
    mouth.x = (40 + .5) * room.resolution;
    mouth.y = (20 + .5) * room.resolution;
    mouth.frontier_x = (40 + .5) * room.resolution;
    mouth.frontier_y = (30 + .5) * room.resolution;
    assert(!rooms.GoalRegionStillFrontier(mouth));
    assert(rooms.UnenteredRoomBeyond(mouth));
    paint(16, 64, 40, 62, 0);  // room fully observed
    rooms.UpdateMap(room);
    assert(!rooms.UnenteredRoomBeyond(mouth));
  }
}
