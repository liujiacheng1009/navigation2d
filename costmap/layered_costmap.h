#pragma once

#include <cstdint>
#include <vector>
#include "navigation2d/application/navigation_config.h"
#include "navigation2d/costmap/grid_2d.h"
#include "navigation2d/types.h"

namespace navigation2d {

enum Cost : std::uint8_t {
  kFree = 0, kInscribed = 253, kLethal = 254, kUnknown = 255,
};

class LayeredCostmap {
 public:
  LayeredCostmap(Grid2d static_map, NavigationConfig config);
  void UpdateObstacleLayer(const Pose2d& sensor_pose, const LaserScan& scan);
  void UpdateObstacleLayer(const Pose2d& sensor_pose, const PointCloud2d& cloud);
  void MarkObstacle(double x, double y);
  void ClearObstacle(double x, double y);
  std::uint8_t cost(int x, int y) const;
  bool lethal(double x, double y, double radius) const;
  bool PathBlocked(const Path& path, std::size_t begin = 0) const;
  std::vector<std::uint8_t> RollingWindow(const Pose2d& centre, int* width, int* height,
                                          int* origin_x, int* origin_y) const;
  const Grid2d& grid() const { return static_map_; }
  std::uint64_t digest() const;

 private:
  bool Raytrace(double x0, double y0, double x1, double y1, bool mark_endpoint);
  void Reinflate();
  Grid2d static_map_;
  NavigationConfig config_;
  std::vector<std::uint8_t> obstacles_;
  std::vector<std::uint8_t> master_;
};

}  // namespace navigation2d
