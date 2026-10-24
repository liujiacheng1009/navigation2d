#include "navigation2d/costmap/grid_2d.h"

#include <cmath>

namespace navigation2d {

bool Grid2d::occupied(int x, int y) const {
  if (x < 0 || y < 0 || x >= width_ || y >= height_) return true;
  const unsigned char value = cells_[static_cast<std::size_t>(y * width_ + x)];
  // 255 is unknown. It blocks entry, but it is not a wall.
  if (value == 255) return false;
  // Accept both conventions: ROS occupancy (100) and navigation snapshots
  // (254). Values below 100 are free / soft preference only.
  return value >= 100;
}

bool Grid2d::unknown(int x, int y) const {
  if (x < 0 || y < 0 || x >= width_ || y >= height_) return false;
  return cells_[static_cast<std::size_t>(y * width_ + x)] == 255;
}

std::pair<int, int> Grid2d::ToCell(double x, double y) const {
  return {static_cast<int>(std::floor(x / resolution_)),
          static_cast<int>(std::floor(y / resolution_))};
}

std::pair<double, double> Grid2d::CellCenter(int x, int y) const {
  return {(x + .5) * resolution_, (y + .5) * resolution_};
}

bool Grid2d::collides(double x, double y, double radius) const {
  if (x - radius < 0. || y - radius < 0. ||
      x + radius > width_ * resolution_ || y + radius > height_ * resolution_) return true;
  const auto [cx, cy] = ToCell(x, y);
  const int n = static_cast<int>(std::ceil(radius / resolution_));
  for (int gy = cy - n; gy <= cy + n; ++gy) {
    for (int gx = cx - n; gx <= cx + n; ++gx) {
      const auto [wx, wy] = CellCenter(gx, gy);
      if (std::hypot(wx - x, wy - y) <= radius && occupied(gx, gy)) return true;
    }
  }
  return false;
}

}  // namespace navigation2d
