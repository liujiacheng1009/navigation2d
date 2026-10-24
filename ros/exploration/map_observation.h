#pragma once
#include "nav_msgs/msg/occupancy_grid.hpp"
namespace navigation2d::exploration_ros {
// World-frame segment against observed occupied cells (>=50). Unknown cells
// are not walls; out-of-grid samples are blocked. A missing map is permissive,
// matching the original query; callers retain their independent input guards.
bool SegmentCutsWall(const nav_msgs::msg::OccupancyGrid* map, double x0, double y0, double x1,
                     double y1);
}  // namespace navigation2d::exploration_ros
