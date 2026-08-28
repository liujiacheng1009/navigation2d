#include "navigation2d/planning/collision_checker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <Eigen/Geometry>

namespace navigation2d::planning_internal {
namespace {
constexpr double kLarge = 1e20;

void DistanceTransform1d(const std::vector<double>& input, std::vector<double>* output) {
  const int size = static_cast<int>(input.size());
  std::vector<int> sites(size);
  std::vector<double> boundaries(size + 1);
  int count = 0;
  sites[0] = 0;
  boundaries[0] = -std::numeric_limits<double>::infinity();
  boundaries[1] = std::numeric_limits<double>::infinity();
  for (int q = 1; q < size; ++q) {
    double boundary;
    do {
      const int site = sites[count];
      boundary = ((input[q] + q * q) - (input[site] + site * site)) /
                 (2. * (q - site));
      if (boundary <= boundaries[count]) --count;
    } while (boundary <= boundaries[count]);
    ++count;
    sites[count] = q;
    boundaries[count] = boundary;
    boundaries[count + 1] = std::numeric_limits<double>::infinity();
  }
  count = 0;
  output->resize(size);
  for (int q = 0; q < size; ++q) {
    while (boundaries[count + 1] < q) ++count;
    const double delta = q - sites[count];
    (*output)[q] = delta * delta + input[sites[count]];
  }
}

bool PointInPolygon(const Eigen::Vector2d& point,
                    const std::vector<Eigen::Vector2d>& polygon) {
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto& first = polygon[i];
    const auto& second = polygon[j];
    if ((first.y() > point.y()) != (second.y() > point.y()) &&
        point.x() < (second.x() - first.x()) * (point.y() - first.y()) /
                            (second.y() - first.y()) + first.x())
      inside = !inside;
  }
  return inside;
}

double SegmentDistance(const Eigen::Vector2d& point, const Eigen::Vector2d& first,
                       const Eigen::Vector2d& second) {
  const Eigen::Vector2d segment = second - first;
  const double length2 = segment.squaredNorm();
  if (length2 < 1e-18) return (point - first).norm();
  const double ratio = std::clamp((point - first).dot(segment) / length2, 0., 1.);
  return (point - (first + ratio * segment)).norm();
}
}  // namespace

DistanceField::DistanceField(const LayeredCostmap& costmap) : grid_(&costmap.grid()) {
  const int width = grid_->width(), height = grid_->height();
  std::vector<double> row_input(width), row_output;
  std::vector<double> intermediate(width * height, kLarge);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x)
      row_input[x] = costmap.cost(x, y) == kLethal ? 0. : kLarge;
    DistanceTransform1d(row_input, &row_output);
    for (int x = 0; x < width; ++x) intermediate[y * width + x] = row_output[x];
  }
  std::vector<double> column_input(height), column_output;
  distance_m_.resize(width * height);
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) column_input[y] = intermediate[y * width + x];
    DistanceTransform1d(column_input, &column_output);
    for (int y = 0; y < height; ++y)
      distance_m_[y * width + x] = std::sqrt(column_output[y]) * grid_->resolution();
  }
}

double DistanceField::distance(int x, int y) const {
  if (x < 0 || y < 0 || x >= grid_->width() || y >= grid_->height()) return 0.;
  return distance_m_[y * grid_->width() + x];
}

double DistanceField::distance(double world_x, double world_y) const {
  const auto [x, y] = grid_->ToCell(world_x, world_y);
  return distance(x, y);
}

Eigen::Vector2d DistanceField::gradient(double world_x, double world_y) const {
  const auto [x, y] = grid_->ToCell(world_x, world_y);
  const double scale = .5 / grid_->resolution();
  return {scale * (distance(x + 1, y) - distance(x - 1, y)),
          scale * (distance(x, y + 1) - distance(x, y - 1))};
}

bool DistanceField::CircleCollisionFree(double world_x, double world_y, double radius) const {
  const auto [x, y] = grid_->ToCell(world_x, world_y);
  if (x < 0 || y < 0 || x >= grid_->width() || y >= grid_->height()) return false;
  const double boundary = std::min({world_x, world_y,
      grid_->width() * grid_->resolution() - world_x,
      grid_->height() * grid_->resolution() - world_y});
  return boundary >= radius && distance(x, y) > radius + std::sqrt(.5) * grid_->resolution();
}

FootprintLookup::FootprintLookup(std::vector<Eigen::Vector2d> footprint, int yaw_bins,
                                 double resolution) {
  if (footprint.size() < 3 || yaw_bins < 1 || resolution <= 0.)
    throw std::invalid_argument("invalid footprint lookup geometry");
  offsets_.resize(yaw_bins);
  for (int bin = 0; bin < yaw_bins; ++bin) {
    const double yaw = 2. * M_PI * bin / yaw_bins;
    const Eigen::Rotation2Dd rotation(yaw);
    std::vector<Eigen::Vector2d> rotated;
    double extent = 0.;
    for (const auto& point : footprint) {
      rotated.push_back(rotation * point);
      extent = std::max(extent, rotated.back().norm());
    }
    const int cells = static_cast<int>(std::ceil(extent / resolution)) + 1;
    const double conservative = std::sqrt(.5) * resolution;
    for (int y = -cells; y <= cells; ++y) for (int x = -cells; x <= cells; ++x) {
      const Eigen::Vector2d centre(x * resolution, y * resolution);
      bool covered = PointInPolygon(centre, rotated);
      for (std::size_t i = 0; !covered && i < rotated.size(); ++i)
        covered = SegmentDistance(centre, rotated[i], rotated[(i + 1) % rotated.size()]) <= conservative;
      if (covered) offsets_[bin].push_back({x, y});
    }
  }
}

int FootprintLookup::Bin(double yaw) const {
  const double normalized = std::fmod(std::fmod(yaw, 2. * M_PI) + 2. * M_PI, 2. * M_PI);
  return static_cast<int>(std::lround(normalized * offsets_.size() / (2. * M_PI))) % offsets_.size();
}

bool FootprintLookup::CollisionFree(const LayeredCostmap& costmap, const Pose2d& pose) const {
  const auto [cx, cy] = costmap.grid().ToCell(X(pose), Y(pose));
  for (const auto& [dx, dy] : offsets_[Bin(Yaw(pose))])
    if (costmap.cost(cx + dx, cy + dy) == kLethal) return false;
  return true;
}

bool FootprintLookup::SweptCollisionFree(const LayeredCostmap& costmap,
                                         const std::vector<Pose2d>& samples) const {
  return std::all_of(samples.begin(), samples.end(),
                     [&](const Pose2d& pose) { return CollisionFree(costmap, pose); });
}

}  // namespace navigation2d::planning_internal
