#pragma once
#include <string>

#include "navigation2d/application/exploration/entry_probe_executor.h"
#include "rclcpp/logger.hpp"
namespace navigation2d::exploration_ros {
// Status remains a node output so the same publisher/capture boundary is used.
class EntryStatusSink {
 public:
  virtual ~EntryStatusSink() = default;
  virtual void EntryStatus(const char* phase, const char* reason) = 0;
};
void ReportEntry(const rclcpp::Logger&, const exploration::EntryNotice&,
                 const std::string& planning_failure, EntryStatusSink&);
}  // namespace navigation2d::exploration_ros
