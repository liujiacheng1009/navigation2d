#include "navigation2d/costmap/grid_2d.h"

#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace navigation2d {

Grid2d Grid2d::Load(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  Grid2d grid;
  grid.width_ = root["width"].as<int>();
  grid.height_ = root["height"].as<int>();
  grid.resolution_ = root["resolution"].as<double>();
  grid.cells_.reserve(grid.width_ * grid.height_);
  for (const auto& value : root["cells"]) grid.cells_.push_back(value.as<int>());
  if (grid.width_ <= 0 || grid.height_ <= 0 || grid.resolution_ <= 0. ||
      grid.cells_.size() != static_cast<size_t>(grid.width_ * grid.height_)) {
    throw std::runtime_error("invalid occupancy grid: " + path);
  }
  return grid;
}

bool Grid2d::occupied(int x, int y) const {
  return x < 0 || y < 0 || x >= width_ || y >= height_ ||
         cells_[y * width_ + x] >= 250;
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
