#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <system_error>

#include "navigation2d/costmap/grid_2d.h"
#include "navigation2d/ros/exploration/planning_map_adapter.h"
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("planning map adapter contract failed");
  }
}
struct Directory {
  std::filesystem::path path;
  Directory() {
    char name[] = "/tmp/exploration-map-XXXXXX";
    const char* created = mkdtemp(name);
    if (!created) {
      throw std::runtime_error("mkdtemp failed");
    }
    path = created;
  }
  ~Directory() {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }
};
}  // namespace
int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  using namespace navigation2d::exploration_io;
  Directory directory;
  PlanningMapAdapter adapter((directory.path / "map-").string());
  ExplorationGrid map{5, 5, .5, -1.25, -1.25, std::vector<std::int8_t>(25, -1)};
  map.cells.front() = 100;
  const auto original = map.cells;
  SnapshotRequest request;
  request.robot = MakePose2d(0., 0., 0.);
  request.map_revision = 17;
  Require(adapter.Prepare(map, request) == SnapshotWriteStatus::kWritten);
  Require(adapter.origin_x() == -1.25 && adapter.origin_y() == -1.25);
  Require(std::filesystem::path(adapter.path()).filename() == "map-17.json");
  const auto loaded = Grid2d::Load(adapter.path());
  Require(loaded.width() == 5 && loaded.height() == 5 && loaded.resolution() == .5);
  Require(loaded.occupied(0, 0));
  Require(loaded.unknown(4, 0));
  Require(!loaded.occupied(2, 2) && !loaded.unknown(2, 2));
  Require(map.cells == original);
  const auto* mask = adapter.snapshot().free_mask.data();
  request.map_revision = 18;
  Require(adapter.Prepare(map, request) == SnapshotWriteStatus::kWritten);
  Require(adapter.snapshot().map_revision == 18 && adapter.snapshot().free_mask.data() == mask);
  PlanningMapAdapter broken("/proc/exploration-map-unwritable-");
  Require(broken.Prepare(map, request) == SnapshotWriteStatus::kStreamFailed);
  Require(broken.snapshot().map_revision == 18);  // Preserve prepared evidence on I/O failure.
}
