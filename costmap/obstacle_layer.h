#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

#include "navigation2d/costmap/cost_values.h"
#include "navigation2d/costmap/grid_2d.h"
#include "navigation2d/geometry/pose_2d.h"
#include "navigation2d/sensor/observations.h"
namespace navigation2d::costmap_internal {
struct ObstacleObservationLimits {
  double obstacle_max_range;
  double raytrace_max_range;
};
// Owns only dynamic obstacle marks. Grid geometry and observation limits are
// borrowed for a synchronous call; copying/moving retains no borrowed views.
// The grid dimensions must match the construction size for every call.
class ObstacleLayer {
 public:
  explicit ObstacleLayer(std::size_t cells) : cells_(cells, kFree) {}
  void Mark(const Grid2d&, double x, double y);
  void Clear(const Grid2d&, double x, double y);
  bool Update(const Grid2d&, const Pose2d&, const LaserScan&, ObstacleObservationLimits);
  bool Update(const Grid2d&, const Pose2d&, const PointCloud2d&, ObstacleObservationLimits);
  std::uint8_t at(std::size_t index) const { return cells_[index]; }

 private:
  bool Raytrace(const Grid2d&, double x0, double y0, double x1, double y1, bool mark_endpoint);
  std::vector<std::uint8_t> cells_;
};
}  // namespace navigation2d::costmap_internal
