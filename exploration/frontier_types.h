#pragma once
#include <cstdint>
#include <vector>
namespace navigation2d {
struct ExplorationGrid {
  int width = 0;
  int height = 0;
  double resolution = 0.;
  double origin_x = 0.;
  double origin_y = 0.;
  std::vector<std::int8_t> cells;
};

struct ExplorationGoal {
  double x = 0.;
  double y = 0.;
  double yaw = 0.;
  double frontier_x = 0.;
  double frontier_y = 0.;
  int frontier_cells = 0;
  // Number of frontier cells represented by this observation task.  Kept
  // separately from the raw component size so tour ordering can favour one
  // informative viewpoint over several tiny fragments of the same boundary.
  double information_gain = 0.;
  double score = 0.;
  // True when this goal was synthesized from leftover specks that failed
  // the ordinary cluster/standoff tests.  Used only as one completing probe
  // after the exploring tour is empty.
  bool leftover_approach = false;
  // Assigned by RegionScheduler after frontier extraction. Kept at the end
  // so existing aggregate construction of goals remains source compatible.
  std::uint64_t region_id = 0;
};

}  // namespace navigation2d
