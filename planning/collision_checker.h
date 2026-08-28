#pragma once

#include <vector>

#include <Eigen/Core>

#include "navigation2d/costmap/layered_costmap.h"

namespace navigation2d::planning_internal {

class DistanceField {
 public:
  explicit DistanceField(const LayeredCostmap& costmap);
  double distance(int x, int y) const;
  double distance(double world_x, double world_y) const;
  bool CircleCollisionFree(double world_x, double world_y, double radius) const;

 private:
  const Grid2d* grid_;
  std::vector<double> distance_m_;
};

class FootprintLookup {
 public:
  FootprintLookup(std::vector<Eigen::Vector2d> footprint, int yaw_bins,
                  double resolution);
  int yaw_bins() const { return static_cast<int>(offsets_.size()); }
  bool CollisionFree(const LayeredCostmap& costmap, const Pose2d& pose) const;
  bool SweptCollisionFree(const LayeredCostmap& costmap,
                          const std::vector<Pose2d>& samples) const;

 private:
  int Bin(double yaw) const;
  std::vector<std::vector<std::pair<int, int>>> offsets_;
};

}  // namespace navigation2d::planning_internal
