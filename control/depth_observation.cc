#include "navigation2d/control/depth_observation.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "navigation2d/control/collision_monitor_geometry.h"
namespace navigation2d::collision_internal {
void DepthObservation::CullPassed(const Pose2d& robot_pose, const NavigationConfig& config) {
  const double yaw = Yaw(robot_pose);
  const double cosine = std::cos(yaw), sine = std::sin(yaw);
  const double bumper = CommandedExtent(config, 1.);
  std::erase_if(held_depth_points_, [&](const Eigen::Vector2d& world) {
    const Eigen::Vector2d relative = world - robot_pose.translation();
    const double longitudinal = cosine * relative.x() + sine * relative.y();
    return longitudinal < bumper;
  });
}

void DepthObservation::Update(const Pose2d& robot_pose,
                              const std::vector<Eigen::Vector2d>& base_points, double timestamp,
                              const NavigationConfig& config) {
  const double yaw = Yaw(robot_pose);
  const double cosine = std::cos(yaw), sine = std::sin(yaw);
  const double half_width = TravelCorridorHalfWidth(config);
  const double reach = config.robot_radius + config.collision_monitor_slowdown_distance;
  constexpr double kMatchRadius = .08;
  constexpr int kStableHits = 3;
  std::vector<Eigen::Vector2d> fresh;
  fresh.reserve(base_points.size());
  for (const auto& point : base_points) {
    if (!point.allFinite()) {
      continue;
    }
    if (point.norm() > reach) {
      continue;
    }
    fresh.emplace_back(X(robot_pose) + cosine * point.x() - sine * point.y(),
                       Y(robot_pose) + sine * point.x() + cosine * point.y());
  }
  auto forward = [&](const Eigen::Vector2d& world) {
    const Eigen::Vector2d relative = world - robot_pose.translation();
    const double longitudinal = cosine * relative.x() + sine * relative.y();
    const double lateral = std::abs(-sine * relative.x() + cosine * relative.y());
    return longitudinal > 0. && lateral <= half_width && relative.norm() <= reach + 1e-6;
  };
  std::erase_if(held_depth_points_, [&](const Eigen::Vector2d& held) {
    return std::any_of(fresh.begin(), fresh.end(), [&](const Eigen::Vector2d& point) {
      return (point - held).norm() < kMatchRadius;
    });
  });
  std::vector<bool> matched(depth_tracks_.size(), false);
  std::vector<Track> next_tracks;
  next_tracks.reserve(fresh.size());
  for (const auto& point : fresh) {
    int best = -1;
    double best_distance = kMatchRadius;
    for (std::size_t index = 0; index < depth_tracks_.size(); ++index) {
      if (matched[index]) {
        continue;
      }
      const double distance = (depth_tracks_[index].point - point).norm();
      if (distance < best_distance) {
        best = static_cast<int>(index);
        best_distance = distance;
      }
    }
    Track track;
    track.point = point;
    if (best >= 0) {
      matched[static_cast<std::size_t>(best)] = true;
      track.hits = std::min(depth_tracks_[static_cast<std::size_t>(best)].hits + 1, kStableHits);
    } else {
      track.hits = 1;
    }
    next_tracks.push_back(track);
  }
  for (std::size_t index = 0; index < depth_tracks_.size(); ++index) {
    if (matched[index]) {
      continue;
    }
    const Track& track = depth_tracks_[index];
    if (track.hits >= kStableHits && forward(track.point)) {
      held_depth_points_.push_back(track.point);
    }
  }
  depth_tracks_ = std::move(next_tracks);
  depth_points_ = std::move(fresh);
  depth_stamp_ = timestamp;
  has_depth_stamp_ = true;
}

void DepthObservation::Refresh(const Pose2d& pose, double timestamp,
                               const NavigationConfig& config) {
  if (has_depth_stamp_ && timestamp - depth_stamp_ > .40) {
    depth_points_.clear();
    held_depth_points_.clear();
    depth_tracks_.clear();
    has_depth_stamp_ = false;
  } else {
    CullPassed(pose, config);
  }
}
}  // namespace navigation2d::collision_internal
