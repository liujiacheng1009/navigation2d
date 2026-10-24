#pragma once
#include <bit>
#include <cstdint>
#include <initializer_list>
#include "std_msgs/msg/header.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace navigation2d::test {
// Exact in-process message fields, including order, frames and simulated time.
// Separate from the historical overlay hash so frozen trace comparisons remain
// available. This is a regression fingerprint, not a cryptographic digest.
class MessageFingerprint {
 public:
  std::uint64_t value() const { return value_; }
  void Add(const nav_msgs::msg::Path& path) {
    Word(1);
    Header(path.header);
    Word(path.poses.size());
    for (const auto& pose : path.poses) {
      Header(pose.header);
      Pose(pose.pose);
    }
  }
  void Add(const geometry_msgs::msg::PoseStamped& pose) {
    Word(2);
    Header(pose.header);
    Pose(pose.pose);
  }
  void Add(const geometry_msgs::msg::PoseArray& poses) {
    Word(3);
    Header(poses.header);
    Word(poses.poses.size());
    for (const auto& pose : poses.poses) {
      Pose(pose);
    }
  }

 private:
  void Word(std::uint64_t word) {
    for (int byte = 0; byte < 8; ++byte) {
      value_ ^= (word >> (8 * byte)) & 255;
      value_ *= 1099511628211ULL;
    }
  }
  void Header(const std_msgs::msg::Header& header) {
    Word(static_cast<std::uint64_t>(header.stamp.sec));
    Word(header.stamp.nanosec);
    Word(header.frame_id.size());
    for (unsigned char byte : header.frame_id) {
      Word(byte);
    }
  }
  void Pose(const geometry_msgs::msg::Pose& pose) {
    for (double value : {pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z, pose.orientation.w}) {
      Word(std::bit_cast<std::uint64_t>(value));
    }
  }
  std::uint64_t value_ = 1469598103934665603ULL;
};
}  // namespace navigation2d::test
