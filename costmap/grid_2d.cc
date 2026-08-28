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
  if (root["cells"]) {
    grid.cells_.reserve(grid.width_ * grid.height_);
    for (const auto& value : root["cells"]) grid.cells_.push_back(value.as<int>());
  } else if (root["rectangles"]) {
    grid.cells_.assign(grid.width_ * grid.height_, 0);
    for (const auto& rectangle : root["rectangles"]) {
      if (!rectangle.IsSequence() || rectangle.size() != 5)
        throw std::runtime_error("grid rectangles must be [x0,y0,x1,y1,value]: " + path);
      const int x0 = rectangle[0].as<int>(), y0 = rectangle[1].as<int>();
      const int x1 = rectangle[2].as<int>(), y1 = rectangle[3].as<int>();
      const int value = rectangle[4].as<int>();
      if (x0 < 0 || y0 < 0 || x1 <= x0 || y1 <= y0 || x1 > grid.width_ ||
          y1 > grid.height_ || value < 0 || value > 255)
        throw std::runtime_error("grid rectangle is out of bounds: " + path);
      for (int y = y0; y < y1; ++y) for (int x = x0; x < x1; ++x)
        grid.cells_[y * grid.width_ + x] = static_cast<unsigned char>(value);
    }
  }
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
