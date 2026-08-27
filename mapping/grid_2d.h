#pragma once

#include <string>
#include <utility>
#include <vector>

namespace navigation2d {

class Grid2d {
 public:
  static Grid2d Load(const std::string& path);
  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  bool occupied(int x, int y) const;
  bool collides(double x, double y, double radius) const;
  std::pair<int, int> ToCell(double x, double y) const;
  std::pair<double, double> CellCenter(int x, int y) const;

 private:
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.;
  std::vector<unsigned char> cells_;
};

}  // namespace navigation2d
