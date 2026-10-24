#pragma once
#include <optional>

#include "navigation2d/application/exploration/planning_snapshot.h"
#include "navigation2d/sensor/observations.h"
#include "sensor_msgs/msg/laser_scan.hpp"
namespace navigation2d::exploration_ros {
// Borrowed ranges remain valid until the source message is replaced or mutated.
inline std::optional<exploration::ScanEvidenceView> ScanEvidence(
    const sensor_msgs::msg::LaserScan* scan) {
  if (!scan) {
    return std::nullopt;
  }
  return exploration::ScanEvidenceView{scan->angle_min, scan->angle_increment, scan->range_min,
                                       scan->range_max, scan->ranges};
}
// Navigation owns its scan; copy exactly, without filtering or normalization.
inline LaserScan CopyScan(const sensor_msgs::msg::LaserScan& scan) {
  LaserScan result;
  result.angle_min = scan.angle_min;
  result.angle_increment = scan.angle_increment;
  result.range_min = scan.range_min;
  result.range_max = scan.range_max;
  result.ranges.assign(scan.ranges.begin(), scan.ranges.end());
  return result;
}
}  // namespace navigation2d::exploration_ros
