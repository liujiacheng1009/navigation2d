#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"

namespace rclcpp {
class Node;
}
namespace navigation2d::exploration_ros {
enum class DebugPathChannel { kGlobal, kLocal, kTour };
enum class DebugMapChannel { kPlanningInput, kInflated };

// Synchronous borrowed messages. Implementations must consume before returning;
// no mission decisions or motion commands belong to this presentation boundary.
class DebugReportSink {
 public:
  virtual ~DebugReportSink() = default;
  virtual void SendPath(DebugPathChannel channel, const nav_msgs::msg::Path& message) = 0;
  virtual void SendMap(DebugMapChannel channel, const nav_msgs::msg::OccupancyGrid& message) = 0;
  virtual void SendCarrot(const geometry_msgs::msg::PoseStamped& message) = 0;
  virtual void SendFrontiers(const geometry_msgs::msg::PoseArray& message) = 0;
  virtual void SendFrontierDebug(const std_msgs::msg::String& message) = 0;
};

// Owns the debug publishers. Default topics and QoS match the explorer; prefix
// is for isolated integration tests. Node must outlive the returned owner.
std::unique_ptr<DebugReportSink> MakeRosDebugOutput(rclcpp::Node& node,
                                                    const std::string& prefix = "");
}  // namespace navigation2d::exploration_ros
