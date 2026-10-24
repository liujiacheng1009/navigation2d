#include "navigation2d/ros/exploration/planning_map_adapter.h"

#include <fstream>
#include <utility>
namespace navigation2d::exploration_io {
SnapshotWriteStatus PlanningMapAdapter::Prepare(const ExplorationGrid& map,
                                                const exploration::SnapshotRequest& request) {
  origin_x_ = map.origin_x;
  origin_y_ = map.origin_y;
  path_ = prefix_ + std::to_string(request.map_revision) + ".json";
  std::ofstream output(path_);
  output << "{\"width\":" << map.width << ",\"height\":" << map.height
         << ",\"resolution\":" << map.resolution << ",\"cells\":[";
  auto prepared = builder_.Build(map, request, std::move(snapshot_));
  snapshot_ = std::move(prepared.snapshot);
  for (std::size_t index = 0; index < prepared.cells.size(); ++index) {
    if (index != 0) {
      output << ',';
    }
    const auto cell = prepared.cells[index];
    output << (snapshot_.free_mask[index] ? 0 : (cell < 0 ? 255 : 254));
  }
  output << "]}";
  output.close();
  return output ? SnapshotWriteStatus::kWritten : SnapshotWriteStatus::kStreamFailed;
}
}  // namespace navigation2d::exploration_io
